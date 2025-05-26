"""
Functions for finding optimal trajectory parameters.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from constants import COLORS, MIN_ANGLE
from physics import is_basket, calculate_max_height
from trajectory_visualization import plot_subplot_trajectory

def find_parameter_ranges(h0_m, target_distance_m, basket_height_m, basket_radius_m, min_height_m, min_angle, max_velocity):
    """
    Find the range of valid parameters for a successful basket.
    
    Args:
        h0_m: Initial height in meters
        target_distance_m: Horizontal distance in meters
        basket_height_m: Basket height in meters
        basket_radius_m: Basket radius in meters
        min_height_m: Minimum trajectory height in meters
        min_angle: Minimum launch angle in degrees
        max_velocity: Maximum initial velocity in m/s
        
    Returns:
        Dictionary of parameter ranges or None values if no valid parameters found
    """
    # Use finer step sizes for more accurate results
    vel_step = 0.05  # m/s - finer steps
    angle_step = 0.1  # degrees - finer steps
    
    # Create arrays to store all valid combinations
    valid_velocities = []
    valid_angles = []
    valid_combinations = []
    
    # Exhaustively test all combinations within our limits
    # Ensure we exactly reach max_velocity in our search
    for v0 in np.arange(1.0, max_velocity + 0.01, vel_step):  # Include max_velocity exactly
        for angle in np.arange(min_angle, 90.0, angle_step): # Test all the way to 90 degrees
            # Calculate if this combination meets height requirement
            max_height = calculate_max_height(v0, angle, h0_m)
            
            # Check if trajectory satisfies all requirements
            if (max_height >= min_height_m and
                is_basket(v0, angle, h0_m, target_distance_m, basket_height_m, basket_radius_m)):
                # Add this combination to our lists
                valid_velocities.append(v0)
                valid_angles.append(angle)
                valid_combinations.append((v0, angle))
    
    # If no valid combinations found, return None for all parameters
    if not valid_combinations:
        return {
            'min_velocity': None, 'min_vel_angle': None,
            'max_velocity': None, 'max_vel_angle': None,
            'min_angle': None, 'min_angle_vel': None,
            'max_angle': None, 'max_angle_vel': None
        }
    
    # Find minimum and maximum velocities
    min_v_idx = np.argmin(valid_velocities)
    max_v_idx = np.argmax(valid_velocities)
    min_velocity = valid_velocities[min_v_idx]
    max_velocity_found = valid_velocities[max_v_idx]
    min_vel_angle = valid_angles[min_v_idx]
    max_vel_angle = valid_angles[max_v_idx]
    
    # Find minimum and maximum angles
    min_a_idx = np.argmin(valid_angles)
    max_a_idx = np.argmax(valid_angles)
    min_angle_val = valid_angles[min_a_idx]
    max_angle_val = valid_angles[max_a_idx]
    
    # Find the minimum velocity for each extreme angle
    min_angle_combos = [(v, a) for v, a in valid_combinations if abs(a - min_angle_val) < 0.001]
    max_angle_combos = [(v, a) for v, a in valid_combinations if abs(a - max_angle_val) < 0.001]
    
    min_angle_vel = min([v for v, a in min_angle_combos]) if min_angle_combos else None
    max_angle_vel = min([v for v, a in max_angle_combos]) if max_angle_combos else None
    
    # Return all parameters
    return {
        'min_velocity': min_velocity,
        'min_vel_angle': min_vel_angle,
        'max_velocity': max_velocity_found,
        'max_vel_angle': max_vel_angle,
        'min_angle': min_angle_val,
        'min_angle_vel': min_angle_vel,
        'max_angle': max_angle_val,
        'max_angle_vel': max_angle_vel
    }

def display_parameter_comparison(params, robot_position, h0_m, target_distance_m, 
                                basket_height_m, basket_radius_m, min_height_m, min_angle, max_velocity):
    """
    Create a figure showing comparison of different parameter combinations.
    
    Args:
        params: Dictionary of parameter ranges
        robot_position: (x, y) coordinates of robot
        h0_m: Initial height in meters
        target_distance_m: Horizontal distance in meters
        basket_height_m: Basket height in meters
        basket_radius_m: Basket radius in meters
        min_height_m: Minimum trajectory height in meters
        min_angle: Minimum launch angle in degrees
        max_velocity: Maximum allowed velocity in m/s
        
    Returns:
        Matplotlib figure with comparison plots
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
    
    # Create a new figure to display all four cases
    results_fig = plt.figure(figsize=(12, 8))
    results_fig.patch.set_facecolor('white')
    x, y = robot_position
    results_fig.suptitle(f"Comparison of Optimal Throwing Parameters (Robot Position: {x:.0f}, {y:.0f})", 
                      fontsize=16, fontweight='bold')
    
    # Create subplots for each case
    gs = GridSpec(2, 2, figure=results_fig, wspace=0.3, hspace=0.3)
    
    # Plot Min Velocity
    ax1 = results_fig.add_subplot(gs[0, 0])
    ax1.set_facecolor('white')
    if min_v0 is not None:
        plot_subplot_trajectory(ax1, min_v0, min_v0_angle, h0_m, target_distance_m,
                               basket_height_m, basket_radius_m, min_height_m,
                               f"Minimum Velocity: {min_v0:.2f} m/s at {min_v0_angle:.1f}°")
    else:
        ax1.text(0.5, 0.5, "No solution found for minimum velocity",
               ha='center', va='center', transform=ax1.transAxes,
               color=COLORS['warning'], fontsize=12, fontweight='bold')
    ax1.set_ylabel("Height (m)", fontsize=12, fontweight='bold')
    
    # Plot Max Velocity
    ax2 = results_fig.add_subplot(gs[0, 1])
    ax2.set_facecolor('white')
    if max_v0 is not None:
        plot_subplot_trajectory(ax2, max_v0, max_v0_angle, h0_m, target_distance_m,
                               basket_height_m, basket_radius_m, min_height_m,
                               f"Maximum Velocity: {max_v0:.2f} m/s at {max_v0_angle:.1f}°")
    else:
        ax2.text(0.5, 0.5, "No solution found for maximum velocity",
               ha='center', va='center', transform=ax2.transAxes,
               color=COLORS['warning'], fontsize=12, fontweight='bold')
    
    # Plot Min Angle
    ax3 = results_fig.add_subplot(gs[1, 0])
    ax3.set_facecolor('white')
    if min_angle_val is not None:
        plot_subplot_trajectory(ax3, min_angle_v0, min_angle_val, h0_m, target_distance_m,
                               basket_height_m, basket_radius_m, min_height_m,
                               f"Minimum Angle: {min_angle_val:.1f}° at {min_angle_v0:.2f} m/s")
    else:
        ax3.text(0.5, 0.5, f"No solution found for minimum angle (min {min_angle}°)",
               ha='center', va='center', transform=ax3.transAxes,
               color=COLORS['warning'], fontsize=12, fontweight='bold')
    ax3.set_xlabel("Distance (m)", fontsize=12, fontweight='bold')
    ax3.set_ylabel("Height (m)", fontsize=12, fontweight='bold')
    
    # Plot Max Angle
    ax4 = results_fig.add_subplot(gs[1, 1])
    ax4.set_facecolor('white')
    if max_angle_val is not None:
        plot_subplot_trajectory(ax4, max_angle_v0, max_angle_val, h0_m, target_distance_m,
                               basket_height_m, basket_radius_m, min_height_m,
                               f"Maximum Angle: {max_angle_val:.1f}° at {max_angle_v0:.2f} m/s")
    else:
        ax4.text(0.5, 0.5, "No solution found for maximum angle",
               ha='center', va='center', transform=ax4.transAxes,
               color=COLORS['warning'], fontsize=12, fontweight='bold')
    ax4.set_xlabel("Distance (m)", fontsize=12, fontweight='bold')
    
    # Add common labels or annotations
    results_fig.text(0.02, 0.5,
                   f"Requirements:\n• Minimum angle: {min_angle}°\n• Minimum height: {min_height_m*1000:.0f} mm\n• Max velocity: {max_velocity} m/s",
                   fontsize=11, va='center', 
                   bbox=dict(facecolor='white', alpha=0.9, boxstyle='round,pad=0.5', edgecolor='gray'))
    
    # Format layout
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    
    return results_fig