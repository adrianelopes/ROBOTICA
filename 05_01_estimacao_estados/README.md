# estimacao_estados

Pacote ROS 2 (Humble) da prática 04, seção 1: adiciona ao robô `my_robot` (Gazebo Classic) uma **IMU**, um **GPS**, uma **câmera RGB frontal** e um **lidar 2D frontal**. Todos aparecem no modelo do robô, publicam em tópicos ROS 2 e usam ruído gaussiano.

## Sensores e tópicos

| Sensor | Tópicos | Mensagem | Frame | Taxa | Ruído (desvio-padrão, escala 1.0) |
|---|---|---|---|---|---|
| IMU | `/imu/data` | `sensor_msgs/Imu` (orientação, velocidade angular, aceleração linear) | `imu_link` | 100 Hz | gyro 0.01 rad/s (+ bias), acel. 0.05 m/s² (+ bias), orientação 0.005/0.005/0.01 rad (rpy) |
| GPS | `/gps/fix`, `/gps/vel` | `sensor_msgs/NavSatFix` (lat, lon, alt), `geometry_msgs/Vector3Stamped` | `gps_link` | 10 Hz | horizontal 0.1 m, vertical 0.2 m, velocidade 0.02 m/s |
| GPS → (X, Y, Z) | `/gps/xyz` | `geometry_msgs/PointStamped` (m, ENU) | `odom` | 10 Hz | herdado de `/gps/fix` |
| Câmera RGB | `/camera/image_raw`, `/camera/camera_info` | `sensor_msgs/Image`, `CameraInfo` (640x480) | `camera_optical_link` | 15 Hz | 0.007 (fração da intensidade) |
| Lidar 2D | `/scan`, `/scan/points` | `sensor_msgs/LaserScan`, `PointCloud2` (180° à frente, 360 raios, 0.12–10 m) | `lidar_link` | 10 Hz | 0.01 m |

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
para ver no RViz a pose estimada (GPS + IMU) ao lado da real (`/odom`). Desligue com
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
| `estimacao_estados/geodesia.py` | Lat/lon/alt → ECEF → ENU (sem ROS) |
| `estimacao_estados/ruido.py` | Quaternion ↔ rpy e ruído gaussiano na orientação (sem ROS) |
| `estimacao_estados/nuvem.py` | Varredura do lidar → pontos 3D (sem ROS) |
| `estimacao_estados/gps_xyz.py`, `imu_orientacao.py`, `scan_para_nuvem.py` | Nós ROS finos que usam os módulos acima |
| `estimacao_estados/arvore.py` | Leitura do YAML do `tf2` e desenho da árvore em texto/DOT (sem ROS) |
| `estimacao_estados/arvore_tf.py` | Nó que imprime a árvore de transformações e, opcionalmente, grava o PDF |
| `estimacao_estados/cmd_vel_relay.py` | Repassa `/cmd_vel` para o `diff_drive_controller` |
| `launch/robo_gazebo.launch.py`, `launch/sensores.launch.py` | Simulação e processamento dos sensores |

A lógica fica em módulos sem dependência de ROS, para testar sem simulador (mesmo padrão de `manobras.py`/`continuo.py` na prática 03).

## Testes

Só com Python (sem ROS nem Gazebo; os testes do modelo usam o `xacro` se estiver instalado):

```bash
cd ~/ros2_ws/src/ROBOTICA/04_sensores
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

## Exercício 2.1 (tópico 1): trajetória em 8 com o estado estimado pelo EKF

O `trajetoria_8` agora controla com `/odometry/filtered` (saída do `ekf_node` do `robot_localization`,
`config/ekf.yaml`). Fluxo dos dados:

```
/odom (rodas: vx, vyaw) ─┐
/gps/xyz + /imu/data ──> estimador_pose ──> /pose_estimada (x, y) ─┼─> ekf_node ──> /odometry/filtered ──> trajetoria_8 ──> /cmd_vel
/imu/data (yaw, vyaw) ───┘
```

`/pose_estimada` continua existindo: é a **medida bruta** (GPS + IMU) que entra no filtro.
`topico_pose` escolhe o que o controle enxerga: `/odometry/filtered` (padrão), `/pose_estimada` (bruto) ou `/odom`.
O erro "real" publicado em `/trajetoria/erro_real` ainda usa `/odom` (odometria das rodas, não é o groundtruth do Gazebo).

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
