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

Instalar: python3 -m pip install simple-pid

## Controle de pose

Dois controladores com a mesma interface: recebem a pose desejada em `/goal_pose`
(`geometry_msgs/PoseStamped`, frame `odom`), leem a odometria em `/odom` e publicam em `/cmd_vel`.

| `controller:=` | Nó | Estratégia |
|---|---|---|
| `continuo` (padrão) | `pose_controller` | Movimento contínuo: 3 PIDs em e_x^b, e_y^b e e_θ, 9 ganhos (requer `pip install simple-pid`) |
| `manobras` | `controle_pose_manobras` | Três manobras: rotação, translação, rotação |

```bash
# Terminal 1: simulador
ros2 launch controle_trajetoria robo_gazebo.launch.py
# Terminal 2: um controlador por vez (ambos publicam em /cmd_vel)
ros2 launch controle_trajetoria pose_control.launch.py controller:=continuo
# ou
ros2 launch controle_trajetoria pose_control.launch.py controller:=manobras
# Terminal 3: pose desejada
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: odom}, pose: {position: {x: 1.0, y: 1.0}, orientation: {z: 0.7071, w: 0.7071}}}"
```
ex:

ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: odom}, pose: {position: {x: 1.0, y: 1.0}, orientation: {z: 0.7071, w: 0.7071}}}"

  
Ganhos e tolerâncias: `config/pose_controller.yaml` e `config/controle_pose_manobras.yaml`.
