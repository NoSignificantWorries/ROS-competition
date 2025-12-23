## Command#13

### Описание  

Курсовой проект по робототехнике. Система навигации для робота. Робот самостоятельно:

- Ждёт сигнал старта (зелёный свет)

- Распознаёт стрелки на развилках (левый/правый путь)

- Читает ArUco маркеры

- Избегает препятствия (датчик глубины)

- Движется по точкам из JSON-конфига

### Архитектура

| Входы                        | Обработка в sensors.py      | Навигация в drive.py                  | Выходы                       |
|-----------------------------|-----------------------------|---------------------------------------|------------------------------|
| `/color/image`              | HSV фильтры (GREEN/BLUE)    | Чтение `/team/start`, `/team/arrow`   | `/cmd_vel` → управление      |
| `/depth/points`             | DEPTH анализ препятствий    | Чтение `/team/depth`                  | `/robot_finish`              |
| `/team/commands`            | Включение режимов детекции  | Чтение `/team/aruco`                  | `/mission_aruco` (лог маркера) |
| `/odom` (позиция робота)    | —                           | ПИД-контроль и выбор точки            | —                            |


### Установка и запуск

`/src`
```
colcon build --packages-select autorace_core_comand13
source install/setup.bash
ros2 launch autorace_core_comand13 autorace_core.launch.py
```

### Структура проекта

autorace_core_command13/  
├── launch/  
│   └── autorace_core.launch.py     # Основной лаунч-файл  
├── src/  
│   ├── sensors.py                  # Обработка камеры + глубины  
│   └── drive.py                    # Навигация + ПИД-контроль  
├── resources/  
│   ├── path.json                   # Маршрут (точки + события)  
│   └── mask.png                    # Шаблон для стрелок  
└── package.xml / setup.py          # ROS2 конфигурация  

### Конфигурация

`path.json`   - маршрут робота:


### ROS Топики
- Входы sensors.py:

    - /color/image → HSV фильтрация

    - /depth/points → препятствия

    - /team/commands → управление режимами

- Выходы sensors.py:

    - /team/start → сигнал старта

    - /team/arrow → left/right

    - /team/aruco → √ID маркера

    - /team/depth → front/stop

- Навигация drive.py:

    - /odom ← позиция робота

    - /cmd_vel → скорость моторов

