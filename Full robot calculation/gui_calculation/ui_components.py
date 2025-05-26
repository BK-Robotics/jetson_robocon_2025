"""
UI components and event handlers for the basketball simulator.
"""
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import math
import numpy as np
from constants import COLORS, MIN_ANGLE, MAX_VELOCITY, ROBOT_HEIGHT, BASKET_HEIGHT, BASKET_RADIUS, MIN_TRAJECTORY_HEIGHT

def create_sliders(fig, init_velocity=9.0, init_angle=MIN_ANGLE):
    """
    Create velocity and angle sliders.
    
    Args:
        fig: Matplotlib figure
        init_velocity: Initial velocity value
        init_angle: Initial angle value
        
    Returns:
        Tuple containing (velocity_slider, angle_slider)
    """
    slider_color = COLORS['slider_bg']
    
    # Changed width from 0.65 to 0.8, and adjusted x-position to 0.1 to center the sliders
    v0_slider_ax = plt.axes([0.1, 0.12, 0.8, 0.03], facecolor=slider_color)
    angle_slider_ax = plt.axes([0.1, 0.07, 0.8, 0.03], facecolor=slider_color)
    
    v0_slider = Slider(v0_slider_ax, 'Initial Velocity (m/s)', 1.0, MAX_VELOCITY, valinit=init_velocity, valstep=0.01)
    angle_slider = Slider(angle_slider_ax, f'Launch Angle (°) - Min: {MIN_ANGLE}°', 
                         MIN_ANGLE, 90, valinit=init_angle, valstep=0.5)
    
    return v0_slider, angle_slider

def create_calculate_button(update_callback):
    """
    Create the calculate parameters button.
    
    Args:
        update_callback: Function to call when button is clicked
        
    Returns:
        Button object
    """
    button_ax = plt.axes([0.7, 0.02, 0.25, 0.03])
    button = Button(button_ax, 'Calculate Parameters', color=COLORS['button_color'])
    button.on_clicked(update_callback)
    return button

def update_button_state(button, is_valid):
    """
    Update the button state based on position validity.
    
    Args:
        button: Button object
        is_valid: Boolean indicating if position is valid
        
    Returns:
        Boolean indicating new button state
    """
    button_enabled = is_valid
    button.color = COLORS['button_color'] if is_valid else COLORS['button_disabled']
    button.label.set_text('Calculate Parameters' if is_valid else 'Invalid Position')
    
    return button_enabled

def calculate_robot_angles(x, y, target_x, target_y):
    """
    Calculate vertical and horizontal angles for the robot.
    
    Args:
        x, y: Robot position coordinates
        target_x, target_y: Target basket coordinates
        
    Returns:
        Tuple containing (horizontal_dist, distance_3d, vertical_angle, robot_angle, closest_basket)
    """
    # Calculate 2D distance
    horizontal_dist = np.sqrt((x - target_x)**2 + (y - target_y)**2)
    
    # 3D distance including basket height
    distance_3d = np.sqrt(horizontal_dist**2 + (BASKET_HEIGHT - ROBOT_HEIGHT)**2)
    
    # Calculate vertical angle for throwing (elevation)
    vertical_angle = math.degrees(math.atan2(BASKET_HEIGHT - ROBOT_HEIGHT, horizontal_dist))
    vertical_angle = max(vertical_angle, MIN_ANGLE)  # Ensure minimum angle
    
    # Calculate horizontal angle (direction)
    angle_adjustment = math.degrees(math.atan2(target_y - y, target_x - x))
    
    # Determine which basket and set base angle
    if target_x > x:  # Right basket
        closest_basket = "Right basket"
        base_angle = 0
        robot_angle = angle_adjustment
    else:  # Left basket
        closest_basket = "Left basket"
        base_angle = 180
        robot_angle = base_angle - angle_adjustment
    
    # Normalize angle to 0-360 range
    if robot_angle < 0:
        robot_angle += 360
    elif robot_angle >= 360:
        robot_angle -= 360
        
    return horizontal_dist, distance_3d, vertical_angle, robot_angle, closest_basket

def format_results_text(params, min_angle):
    """
    Format parameter results for display.
    
    Args:
        params: Dictionary of parameter ranges
        min_angle: Minimum launch angle in degrees
        
    Returns:
        Formatted result string
    """
    # Extract parameters
    min_v0 = params['min_velocity']
    min_v0_angle = params['min_vel_angle']
    max_v0 = params['max_velocity']
    max_v0_angle = params['max_vel_angle']
    min_angle_val = params['min_angle']
    min_angle_v0 = params['min_angle_vel']
    max_angle_val = params['max_angle']
    max_angle_v0 = params['max_angle_vel']
    
    # Format the results for text display
    results_str = ""
    
    if min_v0 is not None:
        results_str += f"MIN VELOCITY: {min_v0:.2f} m/s at {min_v0_angle:.1f}°  |  "
    else:
        results_str += "MIN VELOCITY: Not found  |  "
        
    if max_v0 is not None:
        results_str += f"MAX VELOCITY: {max_v0:.2f} m/s at {max_v0_angle:.1f}°\n"
    else:
        results_str += "MAX VELOCITY: Not found\n"
        
    if min_angle_val is not None:
        results_str += f"MIN ANGLE: {min_angle_val:.1f}° at {min_angle_v0:.2f} m/s  |  "
    else:
        results_str += f"MIN ANGLE: Not found (min {min_angle}°)  |  "
        
    if max_angle_val is not None:
        results_str += f"MAX ANGLE: {max_angle_val:.1f}° at {max_angle_v0:.2f} m/s"
    else:
        results_str += "MAX ANGLE: Not found"
        
    return results_str
