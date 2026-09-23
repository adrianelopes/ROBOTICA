# controle_movimento

Pacote ROS 2 (Humble) que abre o Gazebo, carrega o robô `my_robot` e controla sua velocidade linear e angular pelo tópico `/cmd_vel`, usando o `diff_drive_controller` do `ros2_control`.

## Compilar

```bash
cd ~/ros2_ws
colcon build --packages-select controle_movimento
source install/setup.bash
```

## Executar

```bash
ros2 launch controle_movimento robo_gazebo.launch.py
```

## Testar

sudo apt install xterm

Em outro terminal (lembre de rodar `source ~/ros2_ws/install/setup.bash`):

### 1. Conferir os controladores

```bash
ros2 control list_controllers
```

Os dois devem aparecer como `active`:

```
joint_state_broadcaster  ... active
diff_drive_controller    ... active
```

### 2. Enviar velocidade pelo `/cmd_vel`

```bash
# Para frente a 0,3 m/s
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"

# Girar no lugar a 0,5 rad/s
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"

# Curva (linear + angular)
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.5}}"
```

Ctrl+C interrompe o envio. O robô para sozinho em 0,5 s (`cmd_vel_timeout`).

Pelo teclado:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 3. Acompanhar o movimento

```bash
ros2 topic echo /diff_drive_controller/odom --field pose.pose.position   # odometria
ros2 topic echo /joint_states --field velocity                            # velocidade das rodas (rad/s)
```

Com `x: 0.3` e `z: 0.0`, cada roda deve girar a cerca de 3 rad/s (0,3 / 0,1).

