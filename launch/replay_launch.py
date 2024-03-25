#!/usr/bin/env python3
from simple_launch import SimpleLauncher

# circles: 0.0332
# chessboard: 0.03


def generate_launch_description():
    sl = SimpleLauncher()
    size = sl.declare_arg('size', 0.0332)
    x = sl.declare_arg('x', 6)
    y = sl.declare_arg('y', 6)
    topic = sl.declare_arg('topic', 'image_raw')

    sl.node('camera_calibration', 'cameracalibrator',
            remappings = {'image': topic},
            arguments = ['-s', x+'x'+y,
                         '-p', 'circles',
                         '--square', size,
                         '--no-service-check'],
            output='screen')

    return sl.launch_description()
