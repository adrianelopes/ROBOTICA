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

Dependência Python do controle contínuo: `python3 -m pip install simple-pid`

## Organização do código

| Arquivo | Responsabilidade |
|---|---|
| `controle_trajetoria/manobras.py` | Controle de pose por manobras lineares (rotação → translação → rotação) |
| `controle_trajetoria/continuo.py` | Controle de pose contínuo: 3 PIDs em e_x^b, e_y^b e e_θ (9 ganhos) |
| `controle_trajetoria/seguidor_waypoints.py` | Nó ROS: lê o YAML, a odometria e percorre os waypoints com o controlador escolhido |
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
