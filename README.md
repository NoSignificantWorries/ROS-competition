`/competition`
```
colcon build
source install/setup.bash
ros2 launch robot_bringup autorace_2025.launch.py
```

```
ros2 run referee_console mission_autorace_2025_referee
```

`/src`
```
colcon build
```

`../src`
```
source install/setup.bash
ros2 run autorace_core_comand13 move
```

Либо 

```
ros2 launch autorace_core_comand13 autorace_core.launch.py
```
