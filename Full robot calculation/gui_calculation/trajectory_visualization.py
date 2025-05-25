"""
Functions for visualizing the basketball trajectories.
"""
import matplotlib.pyplot as plt
from matplotlib.patches import Arc
import numpy as np
from constants import COLORS, MIN_TRAJECTORY_HEIGHT, X_OFFSET
from physics import generate_trajectory_data, calculate_max_height

def setup_trajectory_axis(ax):
    """
    Set up the trajectory axis with proper styling.
    
    Args:
        ax: Matplotlib axis object
    """
    ax.set_facecolor('white')
    ax.set_xlabel("Horizontal Distance (m)", fontsize=12, fontweight='bold')
    ax.set_ylabel("Height (m)", fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)

def plot_trajectory(ax, v0, angle_degrees, h0, target_distance, basket_height, basket_radius, min_height, min_angle):
    """
    Plot trajectory for the main trajectory panel.
    
    Args:
        ax: Matplotlib axis object
        v0: Initial velocity in m/s
        angle_degrees: Launch angle in degrees
        h0: Initial height in mm
        target_distance: Horizontal distance in mm
        basket_height: Basket height in mm
        basket_radius: Basket radius in mm
        min_height: Minimum trajectory height in mm
        min_angle: Minimum launch angle in degrees
        
    Returns:
        Tuple containing (result_text, ball_height_at_target, max_height, angle_indicator, angle_arc)
    """
    # Clear the trajectory axis
    ax.clear()
    ax.set_facecolor('white')
    
    # Convert to meters for trajectory calculation
    h0_m = h0 / 1000.0  # Robot height in meters
    target_distance_m = target_distance / 1000.0  # Horizontal distance in meters
    basket_height_m = basket_height / 1000.0  # Basket height in meters
    basket_radius_m = basket_radius / 1000.0  # Basket radius in meters
    min_height_m = min_height / 1000.0  # Minimum trajectory height in meters
    
    # Generate trajectory data
    x, y, ball_height_at_target, max_height, t_target, _ = generate_trajectory_data(
    v0, angle_degrees, h0_m, target_distance_m, x_offset=X_OFFSET)
    
    if x is None:
        ax.set_title("Invalid trajectory parameters", fontsize=14, fontweight='bold', color=COLORS['warning'])
        setup_trajectory_axis(ax)
        return "Invalid velocity", 0, 0, None, None
    
    # Plot trajectory
    ax.plot(x, y, '-', color=COLORS['trajectory'], linewidth=1)
    
    # Find where the ball touches the ground (y=0)
    landing_point_x = None
    for i in range(len(y)-1):
        if y[i] > 0 and y[i+1] <= 0:
            # Linear interpolation to find the exact point where y=0
            ratio = abs(y[i]) / abs(y[i+1] - y[i])
            landing_point_x = x[i] + ratio * (x[i+1] - x[i])
            break
    
    # Mark the landing point if found
    if landing_point_x is not None:
        ax.scatter(landing_point_x, 0, color='darkred', marker='v', s=100, zorder=5, label='Landing point')
        ax.plot([landing_point_x, landing_point_x], [0, 0.2], 'darkred', linestyle='--', alpha=0.7)
        ax.text(landing_point_x, 0.25, f"Landing: {landing_point_x:.2f} m",
               color='darkred', fontsize=10, ha='center',
               bbox=dict(facecolor='white', alpha=0.7, boxstyle='round,pad=0.2'))
    
    # Mark target position
    ax.axvline(x=target_distance_m, color='gray', linestyle='--', alpha=0.5)
    ax.scatter(target_distance_m, basket_height_m, color=COLORS['basket_color'], 
              marker='o', s=80, zorder=5, label='Basket')
    
    # Draw basket outline
    basket_circle = plt.Circle((target_distance_m, basket_height_m), basket_radius_m, 
                             fill=False, color=COLORS['basket_color'], linestyle='-', linewidth=2)
    ax.add_patch(basket_circle)
    
    # Check if the shot would go in
    if abs(ball_height_at_target - basket_height_m) < basket_radius_m:
        result = "BASKET! 🏀"
        result_color = COLORS['success']
        ax.scatter(target_distance_m, ball_height_at_target, color=COLORS['success'], 
                  marker='o', s=80, zorder=5)
    else:
        result = "MISS ❌"
        result_color = COLORS['warning']
        ax.scatter(target_distance_m, ball_height_at_target, color=COLORS['warning'], 
                  marker='x', s=80, zorder=5, linewidth=2)
    
    # Check if trajectory satisfies minimum height requirement
    height_requirement_met = max_height >= min_height_m
    
    # Draw a line indicating the height at target
    ax.plot([target_distance_m, target_distance_m], [0, ball_height_at_target],
            color='green', linestyle='--', alpha=0.5, linewidth=1.5)
    
    # Draw minimum height reference line
    ax.axhline(y=min_height_m, color=COLORS['min_height_line'], linestyle='--', alpha=0.7, linewidth=1.5)
    ax.text(target_distance_m/2, min_height_m+0.15, f"Min Height: {min_height} mm",
           color=COLORS['min_height_line'], fontsize=10, ha='center', 
           bbox=dict(facecolor='white', alpha=0.7, boxstyle='round,pad=0.2'))
    
    # Update the display
    title = f"Projectile Trajectory - {result}"
    if not height_requirement_met:
        title += " - WARNING: Min height requirement not met!"
        title_color = COLORS['warning']
    else:
        title_color = result_color
        
    ax.set_title(title, fontsize=14, fontweight='bold', color=title_color)
    
    # Add text for maximum height
    max_height_text = f"Max Height: {max_height*1000:.0f} mm"
    # Check if max height meets requirement
    if max_height < min_height_m:
        max_height_text += " (Below min height)"
        color = COLORS['warning']
    else:
        max_height_text += " (Above min height)"
        color = COLORS['success']
    
    ax.text(target_distance_m * 0.3, max_height, max_height_text,
          color=color, fontsize=10, ha='center', va='bottom',
          bbox=dict(facecolor='white', alpha=0.7, boxstyle='round,pad=0.2'))
    
    # Mark maximum height point
    angle_rad = np.radians(angle_degrees)
    vy0 = v0 * np.sin(angle_rad)
    if vy0 > 0:  # Only if we have a rising trajectory
        g = 9.81
        t_max = vy0 / g
        if t_max < t_target:  # Only if max height is reached before the target
            vx0 = v0 * np.cos(angle_rad)
            x_max = vx0 * t_max + X_OFFSET
            ax.scatter(x_max, max_height, color='purple', marker='o', s=50)
            ax.plot([x_max, x_max], [0, max_height], 'purple', linestyle='--', alpha=0.3)
    
    # Draw the angle indicator
    arrow_length = target_distance_m / 5
    angle_x_end = arrow_length * np.cos(angle_rad)
    angle_y_end = h0_m + arrow_length * np.sin(angle_rad)
    
    # Plot the indicator arrow
    angle_indicator = ax.arrow(X_OFFSET, h0_m, angle_x_end, angle_y_end - h0_m,
                             head_width=0.2, head_length=0.3,
                             fc='red', ec='red', width=0.05,
                             length_includes_head=True)
    
    # Add an arc to show the angle
    r = arrow_length / 3
    angle_arc = Arc((X_OFFSET, h0_m), r*2, r*2,
                   theta1=0, theta2=angle_degrees,
                   color='red', lw=2)
    ax.add_patch(angle_arc)
    
    # Add text to show the angle value
    ax.text(r/2+X_OFFSET, h0_m + r/2, f"{angle_degrees:.1f}°",
            color='red', fontsize=10, ha='center', va='center',
            bbox=dict(facecolor='white', alpha=0.7, boxstyle='round,pad=0.2'))
    
    # Add launch point
    ax.scatter(X_OFFSET, h0_m, color=COLORS['robot_color'], marker='o', s=80, zorder=5, label='Launch point')
    
    # Add velocity information
    ax.text(target_distance_m/2, h0_m/2, f"Velocity: {v0:.2f} m/s",
            color='blue', fontsize=12, ha='center', va='center',
            bbox=dict(facecolor='white', alpha=0.9, boxstyle='round,pad=0.3'))
    
    # Add ball height information
    ax.text(target_distance_m, ball_height_at_target + 0.3,
          f"Ball height: {ball_height_at_target:.3f} m",
          color='black', fontsize=10, ha='center',
          bbox=dict(facecolor='white', alpha=0.7, boxstyle='round,pad=0.2'))
    
    # Set reasonable y axis limits
    ax.set_xlim(0, max(target_distance_m * 1.2, landing_point_x * 1.1 if landing_point_x else target_distance_m * 3))
    ax.set_ylim(0, max(min_height_m * 1.2, max(y) * 1.2))
    
    ax.legend(loc='best')
    setup_trajectory_axis(ax)
    
    # Return the result and ball height
    return result, ball_height_at_target, max_height, angle_indicator, angle_arc

def plot_subplot_trajectory(ax, v0, angle_degrees, h0_m, target_distance_m,
                           basket_height_m, basket_radius_m, min_height_m, title):
    """
    Plot trajectory in a subplot for comparison.
    
    Args:
        ax: Matplotlib axis object
        v0: Initial velocity in m/s
        angle_degrees: Launch angle in degrees
        h0_m: Initial height in meters
        target_distance_m: Horizontal distance in meters
        basket_height_m: Basket height in meters
        basket_radius_m: Basket radius in meters
        min_height_m: Minimum trajectory height in meters
        title: Title for the subplot
    """
    # Get trajectory data
    x, y, ball_height, max_height, t_target = generate_trajectory_data(
        v0, angle_degrees, h0_m, target_distance_m)
    
    if x is None:
        ax.text(0.5, 0.5, "Invalid trajectory", ha='center', va='center', transform=ax.transAxes)
        return
    
    # Plot trajectory
    ax.plot(x, y, '-', color=COLORS['trajectory'], linewidth=2.5)
    
    # Mark target position
    ax.axvline(x=target_distance_m, color='gray', linestyle='--', alpha=0.5)
    ax.scatter(target_distance_m, basket_height_m, color=COLORS['basket_color'], marker='o', s=50, label='Basket')
    
    # Draw basket
    basket_circle = plt.Circle((target_distance_m, basket_height_m), basket_radius_m,
                             fill=False, color=COLORS['basket_color'], linestyle='-', linewidth=1.5)
    ax.add_patch(basket_circle)
    
    # Check if basket
    is_basket_result = abs(ball_height - basket_height_m) < basket_radius_m
    if is_basket_result:
        ax.scatter(target_distance_m, ball_height, color=COLORS['success'], marker='o', s=30)
    else:
        ax.scatter(target_distance_m, ball_height, color=COLORS['warning'], marker='x', s=30, linewidth=2)
    
    # Draw min height line
    ax.axhline(y=min_height_m, color=COLORS['min_height_line'], linestyle='--', alpha=0.5, linewidth=1.5)
    
    # Set limits
    ax.set_xlim(0, target_distance_m * 1.1)
    ax.set_ylim(0, max(min_height_m * 1.1, max(y) * 1.1))
    
    # Set title and labels
    ax.set_title(title, fontsize=12, fontweight='bold')
    
    # Add info text
    info_text = f"V: {v0:.2f} m/s, θ: {angle_degrees:.1f}°\n"
    info_text += f"Max Height: {max_height*1000:.0f} mm"
    
    ax.text(0.05, 0.95, info_text, transform=ax.transAxes,
           fontsize=10, va='top', ha='left',
           bbox=dict(facecolor='white', alpha=0.9, boxstyle='round,pad=0.3'))
    
    # Set grid
    ax.grid(True, alpha=0.3)
