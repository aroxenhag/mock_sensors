# Mock Sensors

This ROS 2 package provides a mock sensor node that generates synthetic sensor data.
It is useful for testing ROS 2 applications without real hardware.

## Features

- Publishes simulated sensor data for common automation applications.
- Configurable update rate and sensor mode (`random`, `sine`, `step`, `constant`).
- Supports the following topics:

  | Topic Name                | Message Type                       | Description                                     |
  | ------------------------- | ---------------------------------- | ----------------------------------------------- |
  | `/sensor/temperature`     | `sensor_msgs/msg/Temperature`      | Simulated temperature data (Celsius).           |
  | `/sensor/humidity`        | `sensor_msgs/msg/RelativeHumidity` | Simulated humidity percentage.                  |
  | `/sensor/pressure`        | `sensor_msgs/msg/FluidPressure`    | Simulated pressure in Pascal.                   |
  | `/sensor/battery_voltage` | `std_msgs/msg/Float32`             | Simulated battery voltage (V).                  |
  | `/sensor/door_open`       | `std_msgs/msg/Bool`                | Simulated boolean indicating if a door is open. |
  | `/sensor/proximity`       | `std_msgs/msg/Float32`             | Simulated distance to an object.                |

## Installation

1. Copy the package into your ROS 2 workspace (NOTE: Skip this step if the package was installed using the studio boilerplace installer.):

   ```bash
   cd ~/ros2_ws/src/
   unzip /path/to/mock_sensors.zip
   ```

2. Source the ROS 2 environment setup script:

   ```bash
   source /opt/ros/humble/setup.bash
   ```

   - This command sets up the necessary environment variables for ROS 2, including paths for executables, libraries, and package dependencies.

   - It ensures that ROS 2 commands such as `ros2`, `colcon`, and `ament` work correctly in the terminal.

   - This step is required every time a new terminal session is started unless it is added to the shell's profile (e.g., `.bashrc` or `.zshrc`).

3. Restrict ROS 2 communication to localhost:

   ```bash
   export ROS_LOCALHOST_ONLY=1
   ```

   - This setting ensures that ROS 2 nodes communicate only within the local machine and do not attempt to discover or communicate with nodes on a network.
   - It is particularly useful for development and debugging, especially when running nodes on a single machine or using tools like ros2 run and ros2 launch.
   - This helps prevent unintended network discovery issues and enhances security by restricting inter-process communication (IPC) to the localhost.
   - Without this setting, ROS 2 may use multicast-based discovery over the local network, which could cause unexpected behavior in certain environments.

4. Build the package:

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select mock_sensors
   ```

5. Source your workspace:
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

| Parameter Name | Type     | Description                                               |
| -------------- | -------- | --------------------------------------------------------- |
| `sensor_mode`  | `string` | Mode of operation (`random`, `sine`, `step`, `constant`). |
| `update_rate`  | `float`  | Rate at which sensor values are published (Hz).           |

Example of running with custom parameters:

```bash
ros2 run mock_sensors mock_sensor --ros-args -p sensor_mode:=sine -p update_rate:=5.0
```

## License

This package is released under the Apache-2.0 license.
