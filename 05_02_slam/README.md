# Prática 05.2: SLAM

Pacote ROS 2 Humble `estimacao_estados` (pasta `05_02_slam`). O robô `my_robot` (Gazebo
Classic) tem um lidar 2D frontal e faz SLAM 2D com o
[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) numa sala fechada de 8 × 6 m
(`worlds/slam_simples.world`). Toda a odometria é relativa (dead reckoning), sem GPS.

| Exercício | O que pede | Como está implementado |
|---|---|---|
| 1.1 | Ruído no lidar 2D | Ruído gaussiano de 0,03 m no sensor do Gazebo (`ruido_lidar`) |
| 2.1 | Configurar o slam_toolbox | `config/slam_toolbox.yaml` + `launch/slam.launch.py` |
| 2.2 | Comparar 3 situações de ruído | `slam.launch.py cenario:=...`, métricas automáticas e `comparar_slam` |
| 3.1 | Robô sequestrado | `slam.launch.py cenario:=lidar_odom`, arrastar o robô no Gazebo pausado |

## Preparação (uma vez)

Dependências (Ubuntu 22.04 com ROS 2 Humble):

```bash
sudo apt install -y ros-humble-slam-toolbox ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control ros-humble-ros2-controllers ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-rviz2 ros-humble-teleop-twist-keyboard \
  python3-pytest python3-yaml python3-numpy python3-matplotlib
```

Compilar:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src/05_02_slam --packages-select estimacao_estados
```

> **Por que `--base-paths`?** As pastas `05_01_estimacao_estados` e `05_02_slam` têm um pacote
> com o mesmo nome (`estimacao_estados`), e o colcon recusa nomes repetidos mesmo com
> `--packages-select`. O `--base-paths` faz ele procurar pacotes só nesta pasta.

**Em cada terminal novo:**

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && source /usr/share/gazebo/setup.bash
```

## Testes automáticos (sem simulador)

```bash
cd ~/ros2_ws/src/05_02_slam
python3 -m pytest test -q
```

Resultado esperado: todos passam (`121 passed, 1 skipped`). Os testes que cobrem esta atividade:

| Teste | O que confere |
|---|---|
| `test_modelo.py` | Lidar com ruído gaussiano, padrão de 0,03 m, `ruido_lidar` só altera o lidar, `escala_ruido` multiplica |
| `test_slam_config.py` | Frames do slam_toolbox (`map`, `odom`, `chassis`), `/scan` e alcance iguais aos do lidar, sala fechada, sem TF `world → odom` no SLAM |
| `test_slam_cenarios.py` | Odometria com erro (sem ruído = groundtruth, erro independe da taxa, teletransporte ignorado), percurso sem colisão, métricas do mapa, os 3 cenários no launch |

Os demais arquivos de teste são das práticas anteriores, que continuam no pacote.

## Exercício 1.1: ruído no lidar 2D

O ruído está no próprio sensor do Gazebo, em `models/my_robot/sensores.xacro`
(`<noise><type>gaussian</type>`), com desvio-padrão `ruido_lidar × escala_ruido`. O padrão é
0,03 m × 1,0, próximo de um lidar de baixo custo. O lidar tem 360 raios em 180° à frente,
alcance de 0,12 a 10 m, a 10 Hz, publicado em `/scan` (frame `lidar_link`).

**Como testar.** Terminal 1, robô parado na sala, olhando para a parede leste:

```bash
ros2 launch estimacao_estados robo_gazebo.launch.py \
  world:=$(ros2 pkg prefix estimacao_estados)/share/estimacao_estados/worlds/slam_simples.world
```

Terminal 2: mede 50 leituras do raio central (o que aponta para a frente):

```bash
python3 - <<'EOF'
import statistics
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
rclpy.init(); n = rclpy.create_node('medir_ruido'); v = []
n.create_subscription(LaserScan, '/scan', lambda m: v.append(m.ranges[len(m.ranges) // 2]), qos_profile_sensor_data)
while len(v) < 50:
    rclpy.spin_once(n)
print(f'raio central: média {statistics.mean(v):.3f} m, desvio-padrão {statistics.stdev(v):.3f} m')
EOF
```

Repita com o terminal 1 sem ruído (`... ruido_lidar:=0.0`). Valores medidos:

| Terminal 1 | Média | Desvio-padrão |
|---|---:|---:|
| padrão (`ruido_lidar:=0.03`) | 5,912 m | **0,029 m** |
| `ruido_lidar:=0.0` | 5,909 m | 0,001 m |

A média é a distância do lidar (x = 0,08 m) à parede leste (x = 6,0 m). Para ver o ruído no
RViz, rode também `ros2 launch estimacao_estados sensores.launch.py`: os pontos do `/scan`
"tremem" em volta das paredes.

## Exercício 2.1: configuração do slam_toolbox

`config/slam_toolbox.yaml` parte do `mapper_params_online_async.yaml` do slam_toolbox (modo
online assíncrono). Só mudaram os parâmetros marcados com `<- ajustado`:

| Parâmetro | Valor | Por quê |
|---|---|---|
| `use_sim_time` | `true` | Relógio do Gazebo (`/clock`) |
| `base_frame` | `chassis` | Link base do `my_robot` |
| `scan_topic` | `/scan` | Lidar 2D do robô |
| `min_laser_range` / `max_laser_range` | `0.12` / `10.0` | Alcance do lidar em `sensores.xacro` |
| `map_update_interval` | `2.0` | Atualiza o `/map` a cada 2 s |
| `minimum_travel_distance` / `minimum_travel_heading` | `0.3` / `0.3` | Sala pequena: nova varredura a cada 30 cm ou ~17° |

Árvore TF com SLAM:

```
map ──(slam_toolbox)──> odom ──(odometria_slam)──> chassis ──(robot_state_publisher)──> lidar_link
```

O `slam.launch.py` ajusta a simulação para o SLAM:

* **Sem a TF estática `world → odom`** (`publicar_world_odom:=false`): o pai do `odom` passa a
  ser o `map`, e um frame só pode ter um pai.
* **`odom → chassis` vem do nó `odometria_slam`**, não do `diff_drive_controller`
  (`config/controllers_sem_tf_odom.yaml`). Ele integra os incrementos da pose exata do Gazebo
  somando um erro controlado (`config/slam_cenarios.yaml`), o que torna o erro da odometria uma
  variável do experimento. Com `fonte_odom:=rodas` a TF volta a ser a das rodas.
* **`/clock` a 1 kHz** (`config/gazebo_params_slam.yaml`). No padrão de 10 Hz o tempo simulado
  anda em degraus de 100 ms, a TF da odometria fica fora de sincronia com o `/scan` e o SLAM vê
  erro mesmo sem ruído.
* **Percurso automático** (`percurso_sala`): o robô faz sozinho ~31 m pela sala, fechando a
  volta, sempre o mesmo trajeto (guiado pelo groundtruth). Ao terminar, libera o `/cmd_vel`.

**Como testar.**

```bash
ros2 launch estimacao_estados slam.launch.py cenario:=sem_ruido
```

No RViz o mapa (`/map`) vai sendo construído enquanto o robô anda. Num segundo terminal:

```bash
ros2 node list | grep slam              # /slam_toolbox
ros2 topic hz /map                      # ~0,5 Hz (map_update_interval: 2.0)
ros2 run tf2_ros tf2_echo map odom      # correção publicada pelo SLAM
```

O `tf2_echo` pode mostrar `frame does not exist` nos primeiros segundos. Espere ele imprimir a
transformada.

## Exercício 2.2: comparação dos três cenários

| `cenario:=` | Lidar 2D | Odometria entregue ao SLAM |
|---|---|---|
| `sem_ruido` | sem ruído | groundtruth |
| `lidar` | ruído gaussiano de 0,03 m | groundtruth |
| `lidar_odom` | ruído gaussiano de 0,03 m | com erro (valores abaixo) |

Erro da odometria no `lidar_odom` (`config/slam_cenarios.yaml`, seção `odometria_slam`):

| Parâmetro | Valor | Significado |
|---|---|---|
| `sigma_avanco` | 0,05 m/√m | Desvio do avanço após 1 m (cresce com √distância) |
| `sigma_giro` | 0,05 rad/√rad | Desvio do giro após 1 rad girado |
| `sigma_giro_avanco` | 0,03 rad/√m | Desvio da orientação após 1 m em linha reta |
| `vies_avanco` / `vies_giro` | 0,01 / 0,02 | Erro sistemático: mede 1 % a mais de distância e 2 % a mais de giro |
| `semente` | 42 | Mesma semente = mesmo ruído em repetições |

**Como testar.** Rode um de cada vez. Cada um leva de 3 a 5 min e fecha sozinho
(`encerrar:=true`). **Não use Ctrl+C antes do fim**, senão os resultados ficam com o percurso
incompleto.

```bash
ros2 launch estimacao_estados slam.launch.py cenario:=sem_ruido encerrar:=true
ros2 launch estimacao_estados slam.launch.py cenario:=lidar encerrar:=true
ros2 launch estimacao_estados slam.launch.py cenario:=lidar_odom encerrar:=true
```

O fim de cada execução é o resumo `=== <cenario>: ... amostras` e `Arquivos em ...`. Os
arquivos ficam em `resultados/<cenario>/`:

| Arquivo | Conteúdo |
|---|---|
| `mapa.pgm`, `mapa.yaml` | Mapa final |
| `trajetoria.csv` | Groundtruth, SLAM (`map → chassis`) e odometria (`odom → chassis`) no mesmo instante, a 5 Hz |
| `metricas.json` | Erros de trajetória (RMSE, máximo, final) e do mapa (distância das paredes ocupadas às paredes reais do `.world`) |

Depois dos três:

```bash
ros2 run estimacao_estados comparar_slam
```

Gera `resultados/comparacao.md` (tabela) e `resultados/comparacao.png` (mapas com as
trajetórias e o erro de posição no tempo). Confira que a linha "Percurso (m)" mostra cerca de
**31,5** nos três; valor menor significa execução interrompida.

Sem janelas (mais leve, mesmo resultado): acrescente `gui:=false rviz:=false`.

**Resultados obtidos** (detalhes em `resultados/comparacao.md`):

| | Sem ruído | Ruído no lidar | Ruído no lidar e na odometria |
|---|---:|---:|---:|
| SLAM: RMSE de posição | 1,2 cm | 1,8 cm | 4,9 cm |
| Odometria: RMSE de posição | 0 | 0 | 70 cm |
| Maior correção `map → odom` | 2,5 cm | 5,5 cm | 57 cm |
| Células ocupadas sobre a parede real | 99 % | 82 % | 90 % |

* **Sem ruído:** paredes finas (1 célula), a correção `map → odom` quase não se move.
* **Ruído no lidar:** paredes mais grossas e "borradas", pontos soltos no espaço livre. A
  trajetória continua boa, porque a odometria é exata.
* **Ruído no lidar e na odometria:** a odometria sozinha chega a errar 1,6 m, mas o SLAM corrige
  com o casamento de varreduras e o fechamento de laço, e mantém o erro abaixo de 13 cm
  (`ros2 run tf2_ros tf2_echo map odom` mostra a correção crescendo).

## Exercício 3.1: robô sequestrado

Para o experimento valer, **a odometria não pode acompanhar o teletransporte**. O
`odometria_slam` integra os incrementos da pose exata do Gazebo, então o arrasto entraria como
um movimento de metros e o SLAM nunca ficaria perdido. Um salto maior que `salto_maximo`
(0,25 m) ou `giro_maximo` (0,5 rad) entre duas amostras (50 Hz) é ignorado, como numa
odometria de rodas real (as rodas não giraram), e o terminal mostra
`Salto da pose exata ...: robô sequestrado`. Andando normalmente o robô se move só ~6 mm
entre amostras, então isso não afeta o exercício 2.

**Como testar.** Terminal 1 (`avaliar:=false` para não sobrescrever o resultado do
`lidar_odom` do exercício 2):

```bash
ros2 launch estimacao_estados slam.launch.py cenario:=lidar_odom avaliar:=false
```

1. Espere `percurso concluído` no terminal: o mapa está construído e o robô parado em ~(3, 0).
2. Pause o Gazebo (botão ▶/❚❚ na barra inferior).
3. Ative o *Translation Mode* (tecla `T`), clique no robô e arraste-o pelas setas para o
   outro lado da sala, por exemplo **(−1, 2)**, a ~4,5 m. `Esc` volta ao modo de seleção.
4. Retome a simulação (▶). O terminal 1 deve mostrar `Salto da pose exata ...`.

Para um salto exato, no lugar dos passos 2 a 4 (terminal 2):

```bash
ros2 service call /pause_physics std_srvs/srv/Empty
ros2 service call /gazebo/set_entity_state gazebo_msgs/srv/SetEntityState \
  "{state: {name: my_robot, pose: {position: {x: -1.0, y: 2.0, z: 0.1}, orientation: {w: 1.0}}, reference_frame: world}}"
ros2 service call /unpause_physics std_srvs/srv/Empty
```

Terminal 2: gire o robô no lugar (`j`/`l`). Parado, o slam_toolbox não processa varreduras
novas (`minimum_travel_distance/heading`):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Terminal 3: onde o SLAM acha que o robô está (espere alguns segundos):

```bash
ros2 run tf2_ros tf2_echo map chassis
```

**Como verificar:**

* **Não recuperou:** o `tf2_echo` continua mostrando ~(3, 0), enquanto o robô está em (−1, 2)
  no Gazebo. No RViz o lidar (vermelho) não encaixa nas paredes e surgem **paredes fantasmas**:
  as varreduras novas são inseridas no mapa na pose errada.
* **Recuperou:** o `tf2_echo` passa a mostrar ~(−1, 2) e o lidar volta a encaixar no mapa.

**Resultado obtido:** com saltos de 4,4 a 4,6 m o SLAM **não se recuperou**: apareceram
paredes fantasmas e o erro continuou em ~4,4 m após 90 s com o robô girando no lugar. Um salto
pequeno (~0,4 m) foi corrigido em ~6 s. O slam_toolbox não tem relocalização global: o
casamento de varreduras procura a pose só ±0,25 m em torno da previsão da odometria
(`correlation_search_space_dimension: 0.5`), e o fechamento de laço só procura nós a até 3 m da
pose estimada (`loop_search_maximum_distance`). Recuperar de um sequestro exige relocalização
global, como a de um filtro de partículas (por exemplo, o AMCL).

## Argumentos do `slam.launch.py`

| Argumento | Padrão | Descrição |
|---|---|---|
| `cenario` | `sem_ruido` | `sem_ruido`, `lidar` ou `lidar_odom` |
| `encerrar` | `false` | Fecha tudo depois de salvar os resultados do percurso |
| `avaliar` | `true` | Grava mapa, trajetória e métricas em `saida/<cenario>/` |
| `saida` | `~/ros2_ws/src/05_02_slam/resultados` | Pasta dos resultados |
| `percurso` | `true` | `false`: dirija pelo teleop e salve com Ctrl+C |
| `ruido_lidar` | (do cenário) | Desvio-padrão do lidar em m |
| `ruido_odom` | (do cenário) | Multiplica o erro de odometria do YAML (0 = groundtruth) |
| `escala_ruido` | (do cenário) | Multiplica o ruído de todos os sensores |
| `fonte_odom` | `groundtruth` | `rodas`: odometria real do `diff_drive_controller` |
| `gui` / `rviz` | `true` / `true` | Janela do Gazebo / RViz |
| `world` | `worlds/slam_simples.world` | Mundo do Gazebo |
| `slam_params` | `config/slam_toolbox.yaml` | Parâmetros do slam_toolbox |

Teleop: `i` frente, `j`/`l` girar, `k` parar, `z`/`x` diminuir/aumentar a velocidade.

## Arquivos da atividade

| Arquivo | Responsabilidade |
|---|---|
| `models/my_robot/sensores.xacro` | Lidar 2D e seu ruído (`ruido_lidar`, `escala_ruido`) |
| `worlds/slam_simples.world` | Sala fechada de 8 × 6 m com 4 obstáculos |
| `launch/slam.launch.py` | Gazebo + slam_toolbox + odometria + percurso + avaliação + RViz |
| `launch/robo_gazebo.launch.py` | Gazebo e robô (incluído pelo `slam.launch.py`) |
| `config/slam_toolbox.yaml`, `config/slam.rviz` | Parâmetros do slam_toolbox e RViz do SLAM |
| `config/slam_cenarios.yaml` | Erro da odometria, limite de salto, pontos do percurso |
| `config/gazebo_params_slam.yaml`, `config/controllers_sem_tf_odom.yaml` | `/clock` a 1 kHz; desliga a TF das rodas |
| `estimacao_estados/odometria.py`, `odometria_slam.py` | Odometria com erro controlado e detecção de teletransporte |
| `estimacao_estados/percurso.py`, `percurso_sala.py` | Percurso automático pela sala |
| `estimacao_estados/mapa.py`, `avaliacao_slam.py` | Métricas de mapa e trajetória; gravação dos resultados |
| `estimacao_estados/comparar_slam.py` | Tabela e figura comparando os cenários |
| `resultados/` | Resultados dos três cenários e da comparação |

## Problemas comuns

* **`Duplicate package names not supported`:** compile com `--base-paths src/05_02_slam`
  (ver Preparação).
* **`tf2_echo`: `frame does not exist`:** espere alguns segundos; se continuar, confira se o
  terminal fez o `source` do bloco "Em cada terminal novo".
* **Resultados com percurso menor que 31,5 m:** a execução foi interrompida antes de
  `percurso concluído`. Rode de novo com `encerrar:=true`.
* **`test_flake8`/`test_pep257` apontando `build/` ou `install/` dentro de `src/05_02_slam`:**
  são sobras de um `colcon build` rodado na pasta do pacote; podem ser apagadas.
