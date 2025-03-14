from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Nodo principal de generación de señales
        Node(
            package='finalchallenge_deepsilk',
            executable='Input_deepsilk',  # Cambio aquí
            name='input_deepsilk',  # Cambio aquí
            output='screen',
            parameters=[
                {'speed_rpm': 150.0},
                {'signal_type': 'sine'},
                {'frequency': 0.1}
            ],
        ),

        Node(
            package='rqt_plot',
            executable='rqt_plot',
            name='plot',
            output='screen',
            arguments=['/set_point_deepsilk/data', '/motor_output_deepsilk/data'],  # Cambio aquí
        ),

        # Nodo para reconfiguración dinámica (rqt_reconfigure)
        Node(
            package='rqt_reconfigure',
            executable='rqt_reconfigure',
            name='reconfigure',
            output='screen',
        ),

        # Nodo para mostrar el gráfico de los nodos y conexiones (rqt_graph)
        Node(
            package='rqt_graph',
            executable='rqt_graph',
            name='graph',
            output='screen',
        ),
    ])
