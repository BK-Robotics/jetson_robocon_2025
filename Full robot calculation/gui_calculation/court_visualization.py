"""
Functions for drawing the basketball court visualization.
"""
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import numpy as np
from constants import COLORS, COURT_WIDTH, COURT_HEIGHT, BASKET_HEIGHT, HORIZONTAL_DISTANCE

def setup_court_axis(ax):
    """
    Set up the basketball court axis with proper styling.
    
    Args:
        ax: Matplotlib axis object
    """
    ax.set_facecolor(COLORS['court_bg'])
    ax.set_xlim(0, COURT_WIDTH)
    ax.set_ylim(0, COURT_HEIGHT)
    ax.set_xlabel('X (mm)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y (mm)', fontsize=12, fontweight='bold')
    ax.set_title('Basketball Court - Robot Throwing Positions', fontsize=16, fontweight='bold')
    ax.grid(True, linestyle='--', alpha=0.3)

def draw_court(ax):
    """
    Draw the basketball court with baskets and valid throwing areas.
    
    Args:
        ax: Matplotlib axis object
        
    Returns:
        Dict containing court elements (baskets, valid areas)
    """
    # Draw basketball court outline
    court = Rectangle((0, 0), COURT_WIDTH, COURT_HEIGHT, fill=False, 
                      color=COLORS['court_lines'], linewidth=2)
    ax.add_patch(court)
    
    # Set basket positions (one at each end of the court)
    basket1_x, basket1_y = COURT_WIDTH, COURT_HEIGHT/2
    basket2_x, basket2_y = 0, COURT_HEIGHT/2
    
    # Draw baskets
    basket1 = plt.Circle((basket1_x, basket1_y), BASKET_HEIGHT/10, fill=False, 
                        color=COLORS['basket_color'], linewidth=2)
    basket2 = plt.Circle((basket2_x, basket2_y), BASKET_HEIGHT/10, fill=False, 
                        color=COLORS['basket_color'], linewidth=2)
    ax.add_patch(basket1)
    ax.add_patch(basket2)
    
    # Draw the valid throwing positions circles
    valid_circle1 = Circle((basket1_x, basket1_y), HORIZONTAL_DISTANCE, 
                          fill=True, color=COLORS['valid_area'])
    valid_circle2 = Circle((basket2_x, basket2_y), HORIZONTAL_DISTANCE, 
                          fill=True, color=COLORS['valid_area'])
    ax.add_patch(valid_circle1)
    ax.add_patch(valid_circle2)
    
    # Add circle outlines
    valid_circle1_outline = Circle((basket1_x, basket1_y), HORIZONTAL_DISTANCE, 
                                  fill=False, color=COLORS['valid_border'], linestyle='--', linewidth=2)
    valid_circle2_outline = Circle((basket2_x, basket2_y), HORIZONTAL_DISTANCE, 
                                  fill=False, color=COLORS['valid_border'], linestyle='--', linewidth=2)
    ax.add_patch(valid_circle1_outline)
    ax.add_patch(valid_circle2_outline)
    
    # Court markings (center line, center circle)
    ax.axvline(x=COURT_WIDTH/2, color=COLORS['court_lines'], linestyle='-', alpha=0.7, linewidth=2)
    center_circle = Circle((COURT_WIDTH/2, COURT_HEIGHT/2), 1800, 
                          fill=False, color=COLORS['court_lines'], alpha=0.7, linewidth=2)
    ax.add_patch(center_circle)
    
    # Return important elements
    return {
        'basket1': (basket1_x, basket1_y),
        'basket2': (basket2_x, basket2_y),
        'valid_circle1': valid_circle1,
        'valid_circle2': valid_circle2
    }

def update_robot_position(ax, x, y, target_x, target_y, is_valid_position, robot_marker, target_line=None):
    """
    Update the robot position marker and target line on the court.
    
    Args:
        ax: Matplotlib axis object
        x, y: Robot position coordinates
        target_x, target_y: Target basket coordinates
        is_valid_position: Whether the position is valid
        robot_marker: Matplotlib plot object for robot marker
        target_line: Matplotlib line object for target line (or None)
        
    Returns:
        Updated target_line object
    """
    # Update the marker position
    robot_marker.set_data([x], [y])
    
    # Update target line
    line_color = COLORS['valid_color'] if is_valid_position else COLORS['invalid_color']
    
    if target_line is not None:
        target_line.remove()
        
    target_line = ax.plot([x, target_x], [y, target_y],
                         color=line_color, linestyle='-', alpha=0.8, linewidth=2)[0]
    
    return target_line

def update_info_text(info_text, x, y, closest_basket, horizontal_dist, distance_3d, 
                    vertical_angle, robot_angle, is_valid_position, max_throw_distance):
    """
    Update the information text box on the court.
    
    Args:
        info_text: Matplotlib text object
        x, y: Robot position coordinates
        closest_basket: Name of closest basket
        horizontal_dist: Horizontal distance to basket
        distance_3d: 3D distance to basket
        vertical_angle: Vertical angle for throwing
        robot_angle: Robot direction angle
        is_valid_position: Whether the position is valid
        max_throw_distance: Maximum throw distance
    """
    if is_valid_position:
        status = "VALID THROWING POSITION"
        color = COLORS['valid_color']
    else:
        status = "INVALID POSITION - OUTSIDE MAX THROW DISTANCE"
        color = COLORS['invalid_color']
        
    info_text.set_text(
        f"Robot Position: ({x:.0f}, {y:.0f}) mm\n"
        f"Target: {closest_basket}\n"
        f"2D Distance: {horizontal_dist:.0f} mm\n"
        f"3D Distance: {distance_3d:.0f} mm\n"
        f"Max 3D Distance: {max_throw_distance} mm\n"
        f"Vertical Angle: {vertical_angle:.1f}°\n"
        f"Robot Direction: {robot_angle:.1f}°\n"
        f"Status: {status}"
    )
    info_text.set_bbox(dict(facecolor='white', alpha=0.9, boxstyle='round,pad=0.5', edgecolor=color))