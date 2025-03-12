import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult

# Definición de la clase del nodo
class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('Input_deepsilk')  # Cambio aquí

        self.declare_parameter('speed_rpm', 0.0)
        self.declare_parameter('signal_type', 'sine')
        self.declare_parameter('frequency', 1.0)

        self.timer_period = 0.0002439

        # Publicador en el tópico /set_point_deepsilk
        self.signal_publisher = self.create_publisher(Float32, 'set_point_deepsilk', 100)  # Cambio aquí

        self.timer = self.create_timer(self.timer_period, self.timer_cb)

        # Suscriptor al tópico /motor_output_deepsilk
        self.motor_output_subscriber = self.create_subscription(
            Float32,
            'motor_output_deepsilk',  # Cambio aquí
            self.motor_output_callback,
            100
        )

        # Variables internas
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now().nanoseconds / 1e9  # Obtener tiempo inicial con mayor precisión
        self.get_logger().info("SetPoint Node Started 🚀")

        # Callback para cambios en parámetros
        self.add_on_set_parameters_callback(self.parameters_callback)

    # Callback del temporizador: genera y publica la señal
    def timer_cb(self):
        elapsed_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        speed_rpm = self.get_parameter('speed_rpm').value
        frequency = self.get_parameter('frequency').value
        signal_type = self.get_parameter('signal_type').value

        # Validación de parámetros
        if -22 <= speed_rpm <= 22:
         speed_rpm = 0
        elif not -166 <= speed_rpm <= 166:
         self.get_logger().warn(f"speed_rpm {speed_rpm} out of range. Clamping to [-166, 166].")
         speed_rpm = max(-166, min(166, speed_rpm))


        
        if frequency <= 0:
            self.get_logger().warn("Frequency must be greater than 0. Using default value of 1.0 Hz.")
            frequency = 1.0
        
        period = 1.0 / frequency
        amplitude = abs(speed_rpm)  # La amplitud es el valor absoluto de la velocidad RPM

        # Generación de la señal en RPM
        if signal_type == 'sine':
            raw_signal = amplitude * np.sin(2 * np.pi * frequency * elapsed_time)
        elif signal_type == 'square':
         raw_signal = amplitude if (elapsed_time % period) < (period / 2) else -amplitude    
        elif signal_type == 'step':
            raw_signal = amplitude
        else:
            self.get_logger().warn(f"Unknown signal type: {signal_type}, defaulting to sine.")
            raw_signal = amplitude * np.sin(2 * np.pi * frequency * elapsed_time)

        # Ajustar la señal según el signo de la velocidad deseada
        if speed_rpm < 0:
            raw_signal = -raw_signal

        # Convertir a float para evitar error de tipo
        self.signal_msg.data = float(raw_signal)
        self.signal_publisher.publish(self.signal_msg)

    # Callback para el suscriptor de motor_output
    def motor_output_callback(self, msg):
        self.get_logger().info(f"Received motor output: {msg.data}")

    # Callback para cambios en parámetros en tiempo real
    def parameters_callback(self, params):
        for param in params:
            if param.name in ["speed_rpm", "signal_type", "frequency"]:
                setattr(self, param.name, param.value)
                self.get_logger().info(f"Parameter {param.name} changed to {param.value}")
        return SetParametersResult(successful=True)

# Función principal
def main(args=None):
    rclpy.init(args=args)
    node = SetPointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()
