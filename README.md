`/competition`
```
colcon build
source install/setup.bash
ros2 launch robot_bringup autorace_2025.launch.py
```

`/src`
```
colcon build --packages-select autorace_core_comand13
```

`../src`
```
source install/setup.bash
ros2 run autorace_core_comand13 sensors
source install/setup.bash
ros2 run autorace_core_comand13 drive
```

Либо 

```
ros2 launch autorace_core_comand13 autorace_core.launch.py
```

Запуск рефери
```
ros2 run referee_console mission_autorace_2025_referee
```

