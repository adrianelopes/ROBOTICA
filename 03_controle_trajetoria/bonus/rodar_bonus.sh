#!/bin/bash
# Roda em sequência as configurações dos bônus e mede cada uma com o medir_rmse.py.
# Os resultados vão para resultados_rmse.csv (uma linha por volta).
# Uso: feche qualquer Gazebo aberto e rode   bash rodar_bonus.sh

VOLTAS=2
OMEGA_2_2=0.2                       # Ω usado na comparação do bônus 2.2
OMEGAS_2_1="0.2 0.5 0.6 0.7 0.8 0.9"        # Ω testados no bônus 2.1 (realimentado, K_ff = 1)

# cada item: "omega modo k_ff"
RODADAS=(
  "$OMEGA_2_2 malha_aberta 0.0"
  "$OMEGA_2_2 realimentado 0.0"
  "$OMEGA_2_2 realimentado 1.0"
)
for w in $OMEGAS_2_1; do RODADAS+=("$w realimentado 1.0"); done

source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.bash   # sem isso o Gazebo tenta baixar modelos e trava
source ~/ros2_ws/install/setup.bash
AQUI=$(cd "$(dirname "$0")" && pwd)
SHARE=$(ros2 pkg prefix controle_trajetoria)/share/controle_trajetoria

if pgrep -x gzserver > /dev/null || pgrep -f "^/usr/bin/python3 .*lib/controle_trajetoria/trajetoria_8" > /dev/null; then
  echo "Já existe um Gazebo ou um trajetoria_8 rodando. Feche-os antes (pkill -f trajetoria_8; pkill gzserver)."
  exit 1
fi

encerrar_rodada() {
  kill -INT -- -$no -$robo 2> /dev/null
  sleep 5
  kill -KILL -- -$no -$robo 2> /dev/null
  pkill -x gzserver; pkill -x gzclient   # o gzserver roda em outro grupo; só existe o desta rodada
  while pgrep -x gzserver > /dev/null; do sleep 1; done
}
# com Ctrl+C, encerra também o robô e o nó da rodada atual (senão eles ficam rodando sozinhos)
trap 'echo; echo "Interrompido, encerrando a rodada..."; encerrar_rodada; exit 130' INT TERM

set -m  # cada processo em segundo plano ganha o próprio grupo, para encerrar tudo junto
for r in "${RODADAS[@]}"; do
  read -r omega modo kff <<< "$r"
  echo; echo "=== omega=$omega modo=$modo k_ff=$kff ==="

  ros2 launch controle_trajetoria robo_gazebo.launch.py gui:=false \
    params_file:="$SHARE/config/gazebo_params.yaml" > "$AQUI/ultima_rodada_robo.log" 2>&1 &
  robo=$!
  ros2 run controle_trajetoria trajetoria_8 --ros-args --params-file "$SHARE/config/trajetoria_8.yaml" \
    -p use_sim_time:=true -p omega:=$omega -p modo:=$modo -p k_ff:=$kff > "$AQUI/ultima_rodada_no.log" 2>&1 &
  no=$!

  limite=$(awk "BEGIN {print int($VOLTAS * 6.2832 / $omega + 180)}")
  timeout $limite python3 "$AQUI/medir_rmse.py" --voltas $VOLTAS \
    || echo "rodada não terminou em $limite s (veja ultima_rodada_*.log)"

  encerrar_rodada
done

echo; echo "Pronto. Resultados em $AQUI/resultados_rmse.csv"
