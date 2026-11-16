# estimacao_estados

Pacote ROS 2 (Humble) da prática 04, seção 1: adiciona ao robô `my_robot` (Gazebo Classic) uma **IMU**, um **GPS**, uma **câmera RGB frontal** e um **lidar 2D frontal**. Todos aparecem no modelo do robô, publicam em tópicos ROS 2 e usam ruído gaussiano.

## Sensores e tópicos

| Sensor | Tópicos | Mensagem | Frame | Taxa | Ruído (desvio-padrão, escala 1.0) |
|---|---|---|---|---|---|
| IMU | `/imu/data` | `sensor_msgs/Imu` (orientação, velocidade angular, aceleração linear) | `imu_link` | 100 Hz | gyro 0.01 rad/s (+ bias), acel. 0.05 m/s² (+ bias), orientação 0.005/0.005/0.01 rad (rpy) |
| GPS | `/gps/fix`, `/gps/vel` | `sensor_msgs/NavSatFix` (lat, lon, alt), `geometry_msgs/Vector3Stamped` | `gps_link` | 10 Hz | horizontal 0.1 m, vertical 0.2 m, velocidade 0.02 m/s |
| GPS → (X, Y, Z) | `/gps/xyz` | `geometry_msgs/PointStamped` (m, ENU) | `odom` | 10 Hz | herdado de `/gps/fix` |
| Câmera RGB | `/camera/image_raw`, `/camera/camera_info` | `sensor_msgs/Image`, `CameraInfo` (640x480) | `camera_optical_link` | 15 Hz | 0.007 (fração da intensidade) |
| Lidar 2D | `/scan`, `/scan/points` | `sensor_msgs/LaserScan`, `PointCloud2` (180° à frente, 360 raios, 0.12–10 m) | `lidar_link` | 10 Hz | 0.03 m (`ruido_lidar`) |

`/imu/data_bruto` é a saída direta do Gazebo (a orientação ainda sem ruído); `/imu/data` é a que deve ser usada.

## Compilar

Dependências no Ubuntu 22.04 (ROS 2 Humble já instalado):

```bash
sudo apt update
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-rviz2 ros-humble-plotjuggler-ros \
  ros-humble-tf2-tools ros-humble-teleop-twist-keyboard python3-pytest python3-yaml
```

Este repositório é uma pasta de pacotes. Coloque-o em `src/` de um workspace:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <url-deste-repositorio> ROBOTICA      # ou copie a pasta
cd ~/ros2_ws
colcon build --packages-select estimacao_estados
source install/setup.bash
```

## Executar

Três terminais (em cada um: `source ~/ros2_ws/install/setup.bash`).

```bash
# Terminal 1: Gazebo + robô com sensores + controladores
ros2 launch estimacao_estados robo_gazebo.launch.py
```

```bash
# Terminal 2: nós de processamento (IMU, GPS em XYZ, nuvem de pontos) + RViz
ros2 launch estimacao_estados sensores.launch.py
```

```bash
# Terminal 3: mover o robô (setas do teleop publicam em /cmd_vel)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Opções:

```bash
ros2 launch estimacao_estados sensores.launch.py rviz:=false          # sem RViz
ros2 launch estimacao_estados robo_gazebo.launch.py world:=/caminho/outro.world
```

## Exercício 1.2: visualizar os dados

**RViz** (câmera e lidar): abre sozinho no `sensores.launch.py` com `config/sensores.rviz`
(RobotModel, TF, `LaserScan` em `/scan`, `PointCloud2` em `/scan/points`, `Image` em `/camera/image_raw`
e o ponto do GPS). No `/scan` a confiabilidade é *Best Effort*, a mesma do sensor.

**PlotJuggler** (IMU e GPS):

```bash
ros2 run plotjuggler plotjuggler
```

`Streaming` → `ROS2 Topic Subscriber` → Start → selecione `/imu/data`, `/gps/fix` e `/gps/xyz`.
Arraste para o gráfico, por exemplo: `/imu/data/angular_velocity/z`, `/imu/data/linear_acceleration/x`,
`/imu/data/orientation/*`, `/gps/xyz/point/x` contra `/gps/xyz/point/y` (gráfico XY: botão direito → *Convert to XY*).

Conferência rápida pelo terminal:

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /gps/xyz --once
ros2 run tf2_tools view_frames       # árvore: world → odom → chassis → sensores
```

## Árvore de transformações (tf2)

O modelo do robô já define todos os frames dos sensores, então a árvore é montada
automaticamente — não é preciso publicar TF no código dos nós de sensores:

```
world
└── odom                                  static_transform_publisher (no robo_gazebo.launch.py)
    ├── chassis                           diff_drive_controller (enable_odom_tf: true)
    │   ├── left_wheel                    joint_state_broadcaster + robot_state_publisher
    │   ├── right_wheel
    │   ├── imu_link                      robot_state_publisher (juntas fixas do .xacro)
    │   ├── gps_link
    │   ├── camera_link
    │   │   └── camera_optical_link       (convenção óptica: z frente, x direita, y baixo)
    │   └── lidar_link
    └── pose_estimada                     estimador_pose (GPS + IMU, só com a trajetória em 8)
```

| Elo | Quem publica | Onde é definido |
|---|---|---|
| `world` → `odom` | `static_transform_publisher` (nó `world_para_odom`) | `launch/robo_gazebo.launch.py` |
| `odom` → `chassis` | `diff_drive_controller` | `config/controllers.yaml`, `enable_odom_tf: true` |
| `chassis` → sensores e rodas | `robot_state_publisher` | juntas do `models/my_robot/*.xacro` |
| `odom` → `pose_estimada` | `estimador_pose` | `config/trajetoria_8.yaml`, `frame_estimado` |

O `world` é a única frame que ninguém publicaria sozinho: o `odom` vem do controlador
e os links do robô do `robot_state_publisher`. Como o robô nasce na origem do mundo
(`spawn_entity` com `x=y=0`), o `odom` começa coincidindo com o `world`, daí a
transformada identidade. É também o motivo de o `/gps/xyz` publicar em `odom`
(ENU do mundo = `odom`) e de o RViz usar `odom` como *Fixed Frame*.

O `pose_estimada` é irmão do `chassis`, não pai: quem manda no `odom` → `chassis` é o
controlador, então sobrescrevê-lo com a pose dos sensores quebraria a árvore. Assim dá
para ver no RViz a pose estimada (GPS + IMU) ao lado da odometria das rodas (`/odom`).
O groundtruth do modelo é publicado em `/groundtruth/odom`. Desligue com
`frame_estimado: ''` no YAML. Note que a árvore do ROS não lista a raiz como chave
(ela só aparece como `parent`), por isso o `arvore_tf` a mostra sem taxa.

Se um consumidor precisar de outro referencial (por exemplo, o ponto do GPS no
`chassis` em vez de no `odom`), converta em tempo de execução com o `tf2_ros`
(`Buffer.lookup_transform`) em vez de criar mais juntas fixas.

## Ruído dos sensores

O ruído do Gazebo está em `models/my_robot/sensores.xacro` (IMU: velocidade angular e aceleração;
GPS: posição e velocidade; câmera: pixels; lidar: distância). A orientação da IMU não tem ruído no
plugin do Gazebo, então o nó `imu_orientacao` soma ruído gaussiano em roll, pitch e yaw
(`config/sensores.yaml`).

Um único parâmetro multiplica todos os desvios-padrão (útil para o exercício 2.1). Use o mesmo valor nos dois launches:

```bash
ros2 launch estimacao_estados robo_gazebo.launch.py escala_ruido:=5.0
ros2 launch estimacao_estados sensores.launch.py escala_ruido:=5.0
```

`escala_ruido:=0.0` desliga o ruído.

O lidar tem um ajuste próprio, `ruido_lidar` (desvio-padrão em metros, padrão 0.03 m, próximo de
um lidar de baixo custo). Ele também é multiplicado por `escala_ruido` e existe em
`robo_gazebo.launch.py`, `estimacao.launch.py` e `trajetoria_8.launch.py`:

```bash
ros2 launch estimacao_estados robo_gazebo.launch.py ruido_lidar:=0.05   # 5 cm só no lidar
ros2 launch estimacao_estados robo_gazebo.launch.py ruido_lidar:=0.0    # lidar sem ruído
```

## GPS: de (lat, lon, alt) para (X, Y, Z)

O mundo `worlds/sensores.world` define a origem geográfica em `<spherical_coordinates>`
(-7.1396°, -34.8450°, 5 m). O nó `gps_xyz` converte cada `NavSatFix` para o referencial local ENU
dessa origem (WGS84 → ECEF → ENU, em `estimacao_estados/geodesia.py`): X leste, Y norte, Z para cima. Como o robô nasce na origem do mundo, esse
referencial coincide com o `odom`.

O mundo usa `<heading_deg>180</heading_deg>` de propósito. O Gazebo Classic tem um bug: com heading 0,
ele trata o mundo como oeste-sul-cima, e as leituras do GPS saem espelhadas pela origem (robô em (2, 1)
→ GPS em (−2, −1)). Com 180, o mundo do Gazebo fica em ENU (X leste, Y norte). A origem do YAML (`origem_lat/lon/alt`) deve ser igual à do mundo; com
`origem_automatica: true` a origem passa a ser a primeira leitura do GPS.

## Organização do código

| Arquivo | Responsabilidade |
|---|---|
| `models/my_robot/model.urdf.xacro` | Robô (chassi, rodas, `ros2_control`); inclui `sensores.xacro` |
| `models/my_robot/sensores.xacro` | Links, juntas, corpos visuais, plugins e ruído dos quatro sensores |
| `worlds/sensores.world` | Chão, obstáculos coloridos e coordenadas esféricas (GPS) |
| `worlds/slam_simples.world` | Sala fechada de 8 × 6 m com 4 obstáculos, para o SLAM |
| `estimacao_estados/geodesia.py` | Lat/lon/alt → ECEF → ENU (sem ROS) |
| `estimacao_estados/ruido.py` | Quaternion ↔ rpy e ruído gaussiano na orientação (sem ROS) |
| `estimacao_estados/nuvem.py` | Varredura do lidar → pontos 3D (sem ROS) |
| `estimacao_estados/gps_xyz.py`, `imu_orientacao.py`, `scan_para_nuvem.py` | Nós ROS finos que usam os módulos acima |
| `estimacao_estados/arvore.py` | Leitura do YAML do `tf2` e desenho da árvore em texto/DOT (sem ROS) |
| `estimacao_estados/arvore_tf.py` | Nó que imprime a árvore de transformações e, opcionalmente, grava o PDF |
| `estimacao_estados/cmd_vel_relay.py` | Repassa `/cmd_vel` para o `diff_drive_controller` |
| `launch/robo_gazebo.launch.py`, `launch/sensores.launch.py` | Simulação e processamento dos sensores |
| `launch/slam.launch.py`, `config/slam_toolbox.yaml`, `config/slam.rviz` | SLAM 2D com o slam_toolbox |

A lógica fica em módulos sem dependência de ROS, para testar sem simulador (mesmo padrão de `manobras.py`/`continuo.py` na prática 03).

## Testes

Só com Python (sem ROS nem Gazebo; os testes do modelo usam o `xacro` se estiver instalado):

```bash
cd ~/ros2_ws/src/05_02_slam
python3 -m pytest test -v --ignore=test/test_flake8.py --ignore=test/test_pep257.py --ignore=test/test_copyright.py
```

Pelo colcon (inclui flake8 e pep257):

```bash
cd ~/ros2_ws
colcon build --packages-select estimacao_estados
colcon test --packages-select estimacao_estados --event-handlers console_direct+
colcon test-result --verbose
```

| Teste | O que confere |
|---|---|
| `test_geodesia.py` | Origem vira (0, 0, 0); deslocamentos leste/norte contra o raio de curvatura do WGS84; altitude → Z |
| `test_ruido.py` | rpy ↔ quaternion; quaternion unitário; sem ruído não altera; semente reproduz; desvio-padrão medido |
| `test_nuvem.py` | Ângulos e distâncias dos pontos; leituras `inf`/`NaN`/fora do alcance são descartadas |
| `test_modelo.py` | 4 sensores, plugins e tópicos, ruído gaussiano em todos, `escala_ruido` multiplica os desvios, corpo visual, árvore de links |
| `test_configuracao.py` | Origem do GPS no YAML igual à do mundo; arquivos instalados existem |
| `test_tf.py` | Frames dos sensores no URDF, todos os links chegam ao `chassis`, `world` e `odom` publicados |
| `test_arvore.py` | Leitura do YAML do `tf2`, desenho em texto, raiz só como `parent`, frames faltando |
| `test_trajetoria.py` | Referência em 8 (fecha a volta, feedforward = derivada), lei de controle, malha fechada simulada com pose ruidosa |
| `test_estimacao_config.py` | Todo `.py` compila; EKF x tópicos x frames x `controllers.yaml`; controle usa `/odometry/filtered`; launch sobe o EKF |
| `test_slam_config.py` | Frames do slam_toolbox (`map`, `odom`, `chassis`), `/scan` e alcance iguais aos do lidar, sala fechada, `slam.launch.py` desliga a TF `world → odom` |

## Exercício 2.1 (tópico 1): trajetória em 8 com o estado estimado pelo EKF

O `trajetoria_8` agora controla com `/odometry/filtered` (saída do `ekf_node` do `robot_localization`,
`config/ekf.yaml`). Fluxo dos dados:

```
/odom (rodas: vx, vyaw) ─┐
/gps/xyz + /imu/data ──> estimador_pose ──> /pose_estimada (x, y) ─┼─> ekf_node ──> /odometry/filtered ──> trajetoria_8 ──> /cmd_vel
/imu/data (yaw, vyaw) ───┘
```

`/pose_estimada` continua existindo: é a **medida bruta** (GPS + IMU) que entra no filtro.
`topico_pose` escolhe o que o controle enxerga: `/odometry/filtered` (padrão), `/pose_estimada` (GPS + IMU) ou `/odom`.
O groundtruth é obtido diretamente do plugin `gazebo_ros_state` em `/gazebo/model_states` e republicado como `/groundtruth/odom`; ele não usa os sensores nem a odometria das rodas. O erro `/trajetoria/erro_real` e o RMSE impresso pelo controle usam esse groundtruth.

Para conferir os tópicos e preparar a comparação no PlotJuggler:

```bash
ros2 topic echo /groundtruth/odom --once
ros2 topic echo /pose_estimada --once
ros2 topic echo /odometry/filtered --once
ros2 topic echo /trajetoria/desejada --once
```

No PlotJuggler, assine `/groundtruth/odom`, `/pose_estimada`, `/odometry/filtered` e `/trajetoria/desejada`. Para o gráfico XY, use posição `x` contra `y` de cada mensagem: groundtruth `pose.pose.position`, medida bruta `pose.pose.position`, EKF `pose.pose.position` e referência `pose.position`. Para yaw, use `orientation` em quaternion convertido para yaw ou compare trajetórias XY, como pedido no exercício.

```bash
python3 -m pip install simple-pid
cd ~/ros2_ws && colcon build --packages-select estimacao_estados && source install/setup.bash
source /usr/share/gazebo/setup.bash

# Gazebo + sensores + estimador + EKF + controle (um terminal só)
ros2 launch estimacao_estados trajetoria_8.launch.py
ros2 launch estimacao_estados trajetoria_8.launch.py escala_ruido:=5.0              # mais ruído
ros2 launch estimacao_estados trajetoria_8.launch.py topico_pose:=/pose_estimada    # sem filtro, para comparar
```

## Árvore de Transformação
```bash
ros2 run tf2_ros tf2_echo world lidar_link      # estimado x real, lado a lado
ros2 topic echo /tf_static --once               # os elos fixos, com os números do xacro

ros2 run tf2_tools view_frames                  # grava frames.pdf
```

## Exercício 05_02: SLAM 2D com o slam_toolbox

Mapeamento 2D da sala fechada (`worlds/slam_simples.world`) com o
[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) no modo online assíncrono.

### Instalar o slam_toolbox a partir do GitHub (uma vez só)

O pacote é compilado a partir do código-fonte em `~/ros2_ws/src/slam_toolbox` (branch `humble`):

```bash
cd ~/ros2_ws/src
git clone -b humble https://github.com/SteveMacenski/slam_toolbox.git
cd ~/ros2_ws
rosdep install --from-paths src/slam_toolbox --ignore-src -y
source /opt/ros/humble/setup.bash
MAKEFLAGS="-j2" colcon build --packages-select slam_toolbox \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> **Computador travando ao compilar?** Por padrão o `colcon` compila um arquivo C++ por núcleo
> ao mesmo tempo, e cada arquivo do slam_toolbox (Karto/Ceres) usa 1 a 1,5 GB de RAM. Com 8 GB
> e sem swap, a memória acaba. O `MAKEFLAGS="-j2"` limita a 2 arquivos por vez (use `-j1` se
> ainda travar) e o `--executor sequential` compila um pacote por vez. Feche Gazebo, RViz e
> navegador antes de compilar.

Para conferir se o ROS está usando a versão compilada (e não a do apt, em `/opt/ros/humble`):

```bash
source ~/ros2_ws/install/setup.bash
ros2 pkg prefix slam_toolbox    # deve mostrar ~/ros2_ws/install/slam_toolbox
```

Depois disso, compile só o pacote do exercício (`--packages-select estimacao_estados`), para
não recompilar o slam_toolbox a cada `colcon build`.

### Árvore TF com SLAM

```
map ──(slam_toolbox)──> odom ──(diff_drive_controller)──> chassis ──(robot_state_publisher)──> lidar_link
```

Nos outros exercícios o `robo_gazebo.launch.py` publica uma TF estática `world → odom`. Com SLAM,
o pai do `odom` passa a ser o `map`, e um frame só pode ter um pai. Por isso o `slam.launch.py`
passa `publicar_world_odom:=false`.

### Parâmetros (`config/slam_toolbox.yaml`)

Partem do `config/mapper_params_online_async.yaml` do repositório. Os que mudaram estão marcados
com `<- ajustado`:

| Parâmetro | Valor | Por quê |
|---|---|---|
| `use_sim_time` | `true` | Relógio do Gazebo (`/clock`) |
| `base_frame` | `chassis` | Link base do `my_robot` (mesmo do `diff_drive_controller`) |
| `scan_topic` | `/scan` | Lidar 2D do robô (frame `lidar_link`) |
| `min_laser_range` / `max_laser_range` | `0.12` / `10.0` | Alcance do lidar em `sensores.xacro` |
| `map_update_interval` | `2.0` | Atualiza o `/map` a cada 2 s |
| `minimum_travel_distance` / `minimum_travel_heading` | `0.3` / `0.3` | Sala pequena: nova varredura a cada 30 cm ou ~17° |
| `resolution` | `0.05` | Células de 5 cm |

### Executar

```bash
# Terminal 1: compilar e subir Gazebo (sala) + slam_toolbox + RViz
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select estimacao_estados
source install/setup.bash
ros2 launch estimacao_estados slam.launch.py
```

```bash
# Terminal 2: dirigir o robô (clique no terminal antes de usar as teclas)
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Teclas: `i` frente, `j`/`l` girar, `k` parar, `z`/`x` diminuir/aumentar a velocidade. Ande devagar
(~0,3 m/s) e dê uma volta completa pela sala para o slam_toolbox fechar o laço (*loop closure*).

Argumentos do `slam.launch.py`:

| Argumento | Padrão | Descrição |
|---|---|---|
| `world` | `worlds/slam_simples.world` | Mundo do Gazebo |
| `escala_ruido` | `1.0` | Multiplica o ruído dos sensores (0 = sem ruído) |
| `slam_params` | `config/slam_toolbox.yaml` | YAML de parâmetros do slam_toolbox |
| `rviz` | `true` | Abre o RViz (mapa, lidar, robô e TF) |

### Conferir e salvar o mapa

```bash
# Terminal 3
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 node list | grep slam                      # /slam_toolbox
ros2 run tf2_ros tf2_echo map odom              # correção publicada pelo SLAM
ros2 topic hz /map                              # ~0,5 Hz (map_update_interval: 2.0)
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/05_02_slam/mapa_sala
```

O `map_saver_cli` grava `mapa_sala.pgm` (imagem) e `mapa_sala.yaml` (resolução e origem). Também dá
para salvar pelo painel **SlamToolboxPlugin** no RViz (*Save Map*).

### Testes do SLAM

Só os testes de configuração do SLAM, sem ROS nem Gazebo:

```bash
cd ~/ros2_ws/src/05_02_slam
python3 -m pytest test/test_slam_config.py -v
```

Todos os testes do pacote, pelo colcon:

```bash
cd ~/ros2_ws
colcon build --packages-select estimacao_estados
colcon test --packages-select estimacao_estados
colcon test-result --verbose
```

> Se `test_flake8` e `test_pep257` falharem apontando arquivos em `install/sensores/` ou
> `build/sensores/`, a culpa é das pastas `build/`, `install/` e `log/` que ficaram **dentro**
> de `src/05_02_slam`, sobras de um `colcon build` rodado na pasta do pacote. Elas não fazem
> parte do código e podem ser apagadas.
