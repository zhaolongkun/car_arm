#!/usr/bin/env python3
import math


DEFAULT_WHEEL_CENTERS = {
    'front_left': (0.20, 0.20),
    'front_right': (0.20, -0.20),
    'rear_left': (-0.20, 0.20),
    'rear_right': (-0.20, -0.20),
}


def declare_vehicle_safety_parameters(node):
    node.declare_parameter('vehicle_safety_scale', 1.6)
    node.declare_parameter('front_left_wheel_x', DEFAULT_WHEEL_CENTERS['front_left'][0])
    node.declare_parameter('front_left_wheel_y', DEFAULT_WHEEL_CENTERS['front_left'][1])
    node.declare_parameter('front_right_wheel_x', DEFAULT_WHEEL_CENTERS['front_right'][0])
    node.declare_parameter('front_right_wheel_y', DEFAULT_WHEEL_CENTERS['front_right'][1])
    node.declare_parameter('rear_left_wheel_x', DEFAULT_WHEEL_CENTERS['rear_left'][0])
    node.declare_parameter('rear_left_wheel_y', DEFAULT_WHEEL_CENTERS['rear_left'][1])
    node.declare_parameter('rear_right_wheel_x', DEFAULT_WHEEL_CENTERS['rear_right'][0])
    node.declare_parameter('rear_right_wheel_y', DEFAULT_WHEEL_CENTERS['rear_right'][1])


def read_vehicle_safety_geometry(node):
    scale = float(node.get_parameter('vehicle_safety_scale').value)
    centers = {
        'front_left': (
            float(node.get_parameter('front_left_wheel_x').value),
            float(node.get_parameter('front_left_wheel_y').value),
        ),
        'front_right': (
            float(node.get_parameter('front_right_wheel_x').value),
            float(node.get_parameter('front_right_wheel_y').value),
        ),
        'rear_left': (
            float(node.get_parameter('rear_left_wheel_x').value),
            float(node.get_parameter('rear_left_wheel_y').value),
        ),
        'rear_right': (
            float(node.get_parameter('rear_right_wheel_x').value),
            float(node.get_parameter('rear_right_wheel_y').value),
        ),
    }
    distances = {
        name: math.hypot(center[0], center[1])
        for name, center in centers.items()
    }
    max_distance = max(distances.values())
    return {
        'scale': scale,
        'wheel_centers': centers,
        'wheel_distances': distances,
        'max_distance': max_distance,
        'vehicle_safety_radius': scale * max_distance,
    }
