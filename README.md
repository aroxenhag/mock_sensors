# Mock Sensors

This ROS 2 package provides a mock sensor node that generates synthetic sensor data. 
It is useful for testing ROS 2 applications without real hardware.

## Features

- Publishes simulated sensor data for common automation applications.
- Configurable update rate and sensor mode (`random`, `sine`, `step`, `constant`).
- Supports the following topics:

  | Topic Name               | Message Type                 | Description |
  |--------------------------|-----------------------------|-------------|
  | `/sensor/temperature`    | `sensor_msgs/msg/Temperature` | Simulated temperature data (Celsius). |
  | `/sensor/humidity`       | `sensor_msgs/msg/RelativeHumidity` | Simulated humidity percentage. |
  | `/sensor/pressure`       | `sensor_msgs/msg/FluidPressure` | Simulated pressure in Pascal. |
  | `/sensor/battery_voltage` | `std_msgs/msg/Float32` | Simulated battery voltage (V). |
  | `/sensor/door_open`      | `std_msgs/msg/Bool` | Simulated boolean indicating if a door is open. |
  | `/sensor/proximity`      | `std_msgs/msg/Float32` | Simulated distance to an object. |

## Installation

1. Copy the package into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src/
   unzip /path/to/mock_sensors.zip
   ```

2. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select mock_sensors
   ```

3. Source your workspace:
   ```bash
   source install/setup.bash
   ```

## Running the Node

To start the mock sensor node, run:

```bash
ros2 run mock_sensors mock_sensor
```

### Configuration

The node accepts the following parameters:

| Parameter Name | Type | Description |
|---------------|------|-------------|
| `sensor_mode` | `string` | Mode of operation (`random`, `sine`, `step`, `constant`). |
| `update_rate` | `float` | Rate at which sensor values are published (Hz). |

Example of running with custom parameters:

```bash
ros2 run mock_sensors mock_sensor --ros-args -p sensor_mode:=sine -p update_rate:=5.0
```

## License

This package is released under the Apache-2.0 license.
