import rclpy
from rclpy.node import Node
import random
import math
from sensor_msgs.msg import Temperature, RelativeHumidity, FluidPressure
from std_msgs.msg import Float32, Bool

class MockSensorNode(Node):
    def __init__(self):
        super().__init__('mock_sensor')
        
        # Declare parameters
        self.declare_parameter('sensor_mode', 'random')
        self.declare_parameter('update_rate', 1.0)
        
        self.mode = self.get_parameter('sensor_mode').value
        self.rate = self.get_parameter('update_rate').value

        # Publishers
        self.temp_pub = self.create_publisher(Temperature, '/sensor/temperature', 10)
        self.hum_pub = self.create_publisher(RelativeHumidity, '/sensor/humidity', 10)
        self.press_pub = self.create_publisher(FluidPressure, '/sensor/pressure', 10)
        self.battery_pub = self.create_publisher(Float32, '/sensor/battery_voltage', 10)
        self.door_pub = self.create_publisher(Bool, '/sensor/door_open', 10)
        self.proximity_pub = self.create_publisher(Float32, '/sensor/proximity', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.rate, self.publish_sensors)

        self.counter = 0  # Used for step function mode

    def generate_value(self):
        if self.mode == 'random':
            return random.uniform(0.0, 100.0)
        elif self.mode == 'sine':
            self.counter += 1
            return 50.0 + 50.0 * math.sin(self.counter * 0.1)
        elif self.mode == 'step':
            self.counter += 1
            return 0.0 if self.counter % 2 == 0 else 100.0
        elif self.mode == 'constant':
            return 50.0
        return 0.0

    def publish_sensors(self):
        t = Temperature()
        t.temperature = self.generate_value()
        self.temp_pub.publish(t)

        h = RelativeHumidity()
        h.relative_humidity = self.generate_value() / 100.0
        self.hum_pub.publish(h)

        p = FluidPressure()
        p.fluid_pressure = self.generate_value() * 10
        self.press_pub.publish(p)

        b = Float32()
        b.data = self.generate_value() / 10.0
        self.battery_pub.publish(b)

        d = Bool()
        d.data = bool(random.getrandbits(1))
        self.door_pub.publish(d)

        pr = Float32()
        pr.data = self.generate_value() / 10.0
        self.proximity_pub.publish(pr)

def main(args=None):
    rclpy.init(args=args)
    node = MockSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
