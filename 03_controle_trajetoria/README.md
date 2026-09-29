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
