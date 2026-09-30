# controle_trajetoria

Pacote ROS 2 (Humble) da prática 03: controle de trajetória do robô `my_robot` no Gazebo, usando o `diff_drive_controller` do `ros2_control`.

## Compilar

```bash
cd ~/ros2_ws
colcon build --packages-select controle_trajetoria
source install/setup.bash
```

## Executar

```bash
ros2 launch controle_trajetoria robo_gazebo.launch.py
```

O robô recebe comandos pelo `/cmd_vel` e publica a odometria em `/odom`.

Dependências Python: `python3 -m pip install simple-pid` (controle contínuo) e `sudo apt install python3-matplotlib` (gráficos da comparação)

## Organização do código

| Arquivo | Responsabilidade |
|---|---|
| `controle_trajetoria/manobras.py` | Controle de pose por manobras lineares (rotação → translação → rotação) |
| `controle_trajetoria/continuo.py` | Controle de pose contínuo: 3 PIDs em e_x^b, e_y^b e e_θ (9 ganhos) |
| `controle_trajetoria/seguidor_waypoints.py` | Nó ROS: lê o YAML, a odometria e percorre os waypoints com o controlador escolhido |
| `controle_trajetoria/comparar_missoes.py` | Lê os CSVs de duas missões e gera métricas, tabelas e gráficos (bônus 1.2) |
| `controle_trajetoria/cmd_vel_relay.py` | Repassa `/cmd_vel` para o `diff_drive_controller` |

`manobras.py` (`ControlePose`) e `continuo.py` (`PoseController`) mantêm a lógica das versões anteriores,
sem a parte de ROS. Os dois expõem a mesma interface para o seguidor: `definir_objetivo`,
`calcular(pose, dt) -> (v, w)` e `chegou`.

## Seguidor de waypoints

Tudo fica em `config/seguidor_waypoints.yaml`: controlador, waypoints e ganhos.

```yaml
seguidor_waypoints:
  ros__parameters:
    controlador: continuo           # continuo | manobras
    repetir: false                  # true: recomeça ao terminar
    waypoints: [p1, p2, p3, p4]     # ordem de visita (mínimo 3)
    p1: {x: 1.0, y: 0.0, theta: 1.5708}   # x, y em m; theta em rad (opcional)
    ...
```

```bash
# Terminal 1: simulador
ros2 launch controle_trajetoria robo_gazebo.launch.py
# Terminal 2: seguidor (um por vez)
ros2 launch controle_trajetoria waypoints.launch.py controller:=manobras
ros2 launch controle_trajetoria waypoints.launch.py controller:=continuo
# Outro arquivo de parâmetros
ros2 launch controle_trajetoria waypoints.launch.py params_file:=/caminho/meu.yaml
```

## Como o controle contínuo funciona

- **Waypoints intermediários** (`raio_aceitacao`, padrão 0.25 m): o waypoint conta como alcançado
  ao entrar nesse raio e o robô segue para o próximo sem parar (como o `ACC_RAD` do slide de missão).
  A orientação do waypoint não é exigida, então não há giro no lugar entre eles.
  Com `raio_aceitacao: 0` o contínuo volta a exigir a pose completa em cada waypoint.
- **Último waypoint**: exige a pose completa (posição e orientação).
- **Aproximação**: erro no referencial do robô, `e^b = R⁻¹·e`.
  `v = PID_x(e_x^b)` e `w = PID_y(e_y^b)`: anda e aponta para o alvo ao mesmo tempo.
- **Alinhamento** (só no último waypoint, ao chegar na posição): gira no lugar com
  `w = PID_theta(e_theta)`. Nessa fase o erro lateral é ignorado (perto do alvo ele muda de sinal
  com qualquer deslocamento e fazia o robô girar para o lado errado, dando voltas).
  `alignment_hysteresis` evita alternar entre as fases quando o robô se afasta um pouco por inércia.

## Menor caminho angular (wrap)

Todo erro de ângulo passa por `wrap(a) = (a + π) mod 2π − π` (em graus: soma 180, resto da
divisão por 360, subtrai 180), que devolve o ângulo em [-π, π). O sinal do resultado já indica
o sentido do giro mais curto. Está em `continuo.py`, `manobras.py`, `seguidor_waypoints.py`
(theta dos waypoints) e `comparar_missoes.py`.

## Bônus 1.2: comparar três manobras x contínuo

A comparação é justa quando os dois controladores têm os mesmos limites de velocidade
(`max_linear_vel`/`v_max` e `max_angular_vel`/`w_max`, já iguais no YAML) e partem da mesma pose.
Reinicie o simulador entre as duas execuções.

```bash
# Terminal 1: simulador (suba de novo antes de cada execução)
ros2 launch controle_trajetoria robo_gazebo.launch.py

# Terminal 2: 1ª execução; espere aparecer "Trajetória concluída."
ros2 launch controle_trajetoria waypoints.launch.py controller:=continuo log_csv:=~/missoes/continuo.csv
# (reinicie o simulador) e repita com o outro controlador
ros2 launch controle_trajetoria waypoints.launch.py controller:=manobras log_csv:=~/missoes/manobras.csv

# Comparação: tabela no terminal + comparacao.md + 2 gráficos
ros2 run controle_trajetoria comparar_missoes ~/missoes/continuo.csv ~/missoes/manobras.csv --saida ~/missoes/resultado
```

Segunda missão, em que a orientação dos waypoints não aponta para o próximo (as diferenças aparecem mais):

```bash
ros2 launch controle_trajetoria waypoints.launch.py controller:=continuo \
  waypoints_file:=$(ros2 pkg prefix controle_trajetoria)/share/controle_trajetoria/config/missao_zigzag.yaml \
  log_csv:=~/missoes/zz_continuo.csv
```
```bash
ros2 launch controle_trajetoria waypoints.launch.py controller:=manobras \
  waypoints_file:=$(ros2 pkg prefix controle_trajetoria)/share/controle_trajetoria/config/missao_zigzag.yaml \
  log_csv:=~/missoes/zz_manobras.csv
```


Métricas: tempo total e por waypoint, distância percorrida, rotação acumulada, tempo girando parado,
velocidades RMS, erro de posição/orientação na chegada e desvio máximo da reta entre waypoints.

### Tipos de chegada no CSV

- `chegada`: a pose completa foi atingida (as manobras sempre; o contínuo no último waypoint).
- `passagem`: o contínuo entrou no `raio_aceitacao` de um waypoint intermediário.
  Não há erro de chegada para esses waypoints (aparecem como `-` na tabela) e as médias de erro
  consideram só as chegadas. O `comparar_missoes` avisa quando os dois controladores usam
  critérios diferentes, porque isso também influencia o tempo total.
