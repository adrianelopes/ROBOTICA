# model_description

Pacote ROS 2 (Humble) que abre o Gazebo e carrega o robô `my_robot`.

## Como funciona

- `models/my_robot/`: modelo SDF do robô e sua malha.
- `launch/robo_gazebo.launch.py`: inicia o Gazebo e insere o robô com o `spawn_entity.py`.
- `package.xml`: exporta a pasta `models/` para o `GAZEBO_MODEL_PATH`, permitindo que o Gazebo resolva as URIs `model://`.

## Executar

```bash
cd ~/ros2_ws
colcon build --packages-select model_description
source install/setup.bash
ros2 launch model_description robo_gazebo.launch.py
```
