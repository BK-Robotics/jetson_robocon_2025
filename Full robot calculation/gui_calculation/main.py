"""
Main application for the basketball simulator.
"""
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import numpy as np
import math
import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend for interactive plots

# Import local modules
from constants import (
    set_plot_style, COURT_WIDTH, COURT_HEIGHT, BASKET_HEIGHT, BASKET_RADIUS, 
    ROBOT_HEIGHT, MIN_TRAJECTORY_HEIGHT, MAX_THROW_DISTANCE, COLORS, MIN_ANGLE, MAX_VELOCITY
)
from physics import is_valid_trajectory
from court_visualization import setup_court_axis, draw_court, update_robot_position, update_info_text
from trajectory_visualization import setup_trajectory_axis, plot_trajectory
from parameter_optimization import find_parameter_ranges, display_parameter_comparison
from ui_components import (
    create_sliders, create_calculate_button, update_button_state,
    calculate_robot_angles, format_results_text
)

def combined_basketball_simulator():
    """
    Interactive visualization for determining robot positions to throw a basketball
    with trajectory simulation.
    
    Requirements:
    - Robot throwing distance: under 6000 mm (3D distance)
    - Basket height: 2430 mm
    - Basketball court: 15000x8000 mm
    - Minimum launch angle: 60 degrees
    - Minimum trajectory peak height: 6000 mm
    - Maximum initial velocity: 15 m/s
    """
    # Apply plot style
    set_plot_style()
    
    # Create figure with multiple areas
    fig = plt.figure(figsize=(15, 12))
    fig.patch.set_facecolor('white')
    
    # Create a GridSpec for better layout control
    gs = GridSpec(4, 1, height_ratios=[2, 2, 1.5, 0.5], figure=fig)
    
    # Court view
    court_ax = fig.add_subplot(gs[0:2, 0])
    setup_court_axis(court_ax)
    
    # Trajectory view
    traj_ax = fig.add_subplot(gs[2, 0])
    setup_trajectory_axis(traj_ax)
    
    # Results view (for displaying the 4 results)
    results_ax = fig.add_subplot(gs[3, 0])
    results_ax.axis('off')  # Turn off axis for text area
    
    # Adjust the layout
    plt.subplots_adjust(bottom=0.2, hspace=0.5)
    
    # Draw court elements
    court_elements = draw_court(court_ax)
    basket1_pos = court_elements['basket1']
    basket2_pos = court_elements['basket2']
    
    # Add custom legend for court view
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker='o', color='w', markerfacecolor=COLORS['basket_color'], 
              markersize=10, label='Basket'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor=COLORS['robot_color'], 
              markersize=10, label='Robot'),
        Line2D([0], [0], color=COLORS['valid_border'], linestyle='--', linewidth=2,
              label=f'Max throw distance ({MAX_THROW_DISTANCE} mm)')
    ]
    court_ax.legend(handles=legend_elements, loc='upper right', framealpha=0.9)
    
    # Selected position marker
    robot_marker, = court_ax.plot([], [], 'o', color=COLORS['robot_color'], 
                                markersize=12, label='Robot Position')
    
    # Information text box
    info_text = court_ax.text(0.02, 0.02, '', transform=court_ax.transAxes, 
                       bbox=dict(facecolor='white', alpha=0.9, boxstyle='round,pad=0.5', 
                                edgecolor='gray'), fontsize=10)
    
    # Results text
    results_text = results_ax.text(0.5, 0.5, 'Click on the court to select a robot position first.',
                                 ha='center', va='center', transform=results_ax.transAxes,
                                 fontsize=12, fontweight='bold',
                                 bbox=dict(facecolor='white', alpha=0.9, boxstyle='round,pad=0.5', 
                                          edgecolor='gray'))
    
    # Initialize state variables
    selected_position = None
    is_valid_position = False
    angle_indicator = None
    angle_arc = None
    target_line = None
    is_dragging = False
    
    # Create UI components with default values
    v0_slider, angle_slider = create_sliders(fig)
    
    # Function to update trajectory
    def update_trajectory(val):
        if selected_position is not None and is_valid_position:
            x, y, robot_angle, vertical_angle, horizontal_dist = selected_position
            
            # Get current angle from slider (ensure it's at least min_angle)
            current_angle = max(angle_slider.val, MIN_ANGLE)
            if current_angle != angle_slider.val:
                angle_slider.set_val(current_angle)
            
            # Plot trajectory with current slider values
            result, ball_height, max_height, new_angle_indicator, new_angle_arc = plot_trajectory(
                traj_ax,
                v0_slider.val,
                current_angle,
                ROBOT_HEIGHT,
                horizontal_dist,
                BASKET_HEIGHT,
                BASKET_RADIUS,
                MIN_TRAJECTORY_HEIGHT,
                MIN_ANGLE
            )
            
            # Update angle indicators
            nonlocal angle_indicator, angle_arc
            angle_indicator = new_angle_indicator
            angle_arc = new_angle_arc
            
            fig.canvas.draw_idle()
    
    # Create calculate button
    def on_calculate_button(event):
        if not button_enabled or selected_position is None or not is_valid_position:
            return
            
        x, y, robot_angle, vertical_angle, horizontal_dist = selected_position
        
        # Convert to meters for calculation
        h0_m = ROBOT_HEIGHT / 1000.0
        target_distance_m = horizontal_dist / 1000.0
        basket_height_m = BASKET_HEIGHT / 1000.0
        basket_radius_m = BASKET_RADIUS / 1000.0
        min_height_m = MIN_TRAJECTORY_HEIGHT / 1000.0
        
        # Show searching message
        traj_ax.clear()
        traj_ax.set_facecolor('white')
        traj_ax.text(0.5, 0.5,
                   "Searching for optimal parameters...\n"
                   f"Requirements:\n"
                   f"• Minimum angle: {MIN_ANGLE}°\n"
                   f"• Minimum height: {MIN_TRAJECTORY_HEIGHT} mm\n"
                   f"• Maximum velocity: {MAX_VELOCITY} m/s\n"
                   f"• Finding min/max velocities and angles",
                   ha='center', va='center', transform=traj_ax.transAxes,
                   fontsize=12, color='blue',
                   bbox=dict(facecolor='white', alpha=0.9, boxstyle='round,pad=0.5'))
        
        results_text.set_text("Calculating... please wait")
        fig.canvas.draw_idle()
        
        # Find all parameter ranges
        params = find_parameter_ranges(
            h0_m, target_distance_m, basket_height_m, basket_radius_m, 
            min_height_m, MIN_ANGLE, MAX_VELOCITY
        )
        
        # Update results text
        results_str = format_results_text(params, MIN_ANGLE)
        results_text.set_text(results_str)
        results_text.set_bbox(dict(facecolor='white', alpha=0.9, boxstyle='round,pad=0.5',
                                 edgecolor=COLORS['success']))
        
        # Set the sliders to the minimum velocity solution (often most energy efficient)
        min_v0 = params['min_velocity']
        min_v0_angle = params['min_vel_angle']
        if min_v0 is not None:
            v0_slider.set_val(min_v0)
            angle_slider.set_val(min_v0_angle)
            
            # Update the main trajectory display with these values
            update_trajectory(None)
        else:
            traj_ax.set_title("No valid solutions found with the given constraints", 
                            color=COLORS['warning'], fontsize=14, fontweight='bold')
        
        # Create and show the comparison figure
        results_fig = display_parameter_comparison(
            params, (x, y), h0_m, target_distance_m, basket_height_m, 
            basket_radius_m, min_height_m, MIN_ANGLE, MAX_VELOCITY
        )
        results_fig.show()
        
        fig.canvas.draw_idle()
    
    optimal_button = create_calculate_button(on_calculate_button)
    button_enabled = False
    update_button_state(optimal_button, False)  # Initial state: disabled
    
    # Click handler for court
    def on_court_click(event):
        if event.inaxes == court_ax:
            x, y = event.xdata, event.ydata
            
            # Calculate distances to both baskets
            basket1_x, basket1_y = basket1_pos
            basket2_x, basket2_y = basket2_pos
            
            dist1 = np.sqrt((x - basket1_x)**2 + (y - basket1_y)**2)
            dist2 = np.sqrt((x - basket2_x)**2 + (y - basket2_y)**2)
            
            # Find closest basket
            if dist1 <= dist2:
                target_x, target_y = basket1_x, basket1_y
            else:
                target_x, target_y = basket2_x, basket2_y
            
            # Calculate robot angles and distances
            horizontal_dist, distance_3d, vertical_angle, robot_angle, closest_basket = calculate_robot_angles(
                x, y, target_x, target_y
            )
            
            # Determine if position is valid for throwing
            nonlocal is_valid_position
            is_valid_position = distance_3d <= MAX_THROW_DISTANCE
            
            # Update the robot position and target line
            nonlocal target_line
            target_line = update_robot_position(
                court_ax, x, y, target_x, target_y, is_valid_position, robot_marker, target_line
            )
            
            # Update information text
            update_info_text(
                info_text, x, y, closest_basket, horizontal_dist, distance_3d,
                vertical_angle, robot_angle, is_valid_position, MAX_THROW_DISTANCE
            )
            
            # Update button state
            nonlocal button_enabled
            button_enabled = update_button_state(optimal_button, is_valid_position)
            
            # Store the selected position for trajectory calculation
            nonlocal selected_position
            selected_position = (x, y, robot_angle, vertical_angle, horizontal_dist)
            
            # Set the initial angle to the calculated vertical angle
            angle_slider.set_val(vertical_angle)
            
            # Reset results if position changes
            if is_valid_position:
                results_text.set_text('Click "Calculate Parameters" to find optimal values.')
            else:
                results_text.set_text('Position is invalid. Move robot within the blue circles.')
            
            # Update the trajectory if position is valid
            if is_valid_position:
                update_trajectory(None)
            else:
                # Clear trajectory plot if position is invalid
                traj_ax.clear()
                traj_ax.set_facecolor('white')
                traj_ax.set_title("Select a valid position first", fontsize=14, fontweight='bold')
                traj_ax.set_xlabel("Horizontal Distance (m)", fontsize=12, fontweight='bold')
                traj_ax.set_ylabel("Height (m)", fontsize=12, fontweight='bold')
                traj_ax.grid(True, alpha=0.3)
            
            fig.canvas.draw_idle()
    
    # Function to handle mouse press for dragging angle
    def on_press(event):
        if event.inaxes == traj_ax and angle_indicator is not None:
            nonlocal is_dragging
            is_dragging = True
    
    # Function to handle mouse motion for dragging angle
    def on_motion(event):
        if is_dragging and event.inaxes == traj_ax and selected_position is not None and is_valid_position:
            # Calculate new angle based on mouse position
            h0_m = ROBOT_HEIGHT / 1000.0
            dx = event.xdata
            dy = event.ydata - h0_m
            
            if dx > 0:  # Prevent negative x values
                new_angle = math.degrees(math.atan2(dy, dx))
                if MIN_ANGLE <= new_angle <= 90:  # Restrict to min_angle-90 degrees
                    angle_slider.set_val(new_angle)
                elif new_angle < MIN_ANGLE:  # If below minimum, set to minimum
                    angle_slider.set_val(MIN_ANGLE)
    
    # Function to handle mouse release for dragging angle
    def on_release(event):
        nonlocal is_dragging
        is_dragging = False
    
    # Connect sliders to update function
    v0_slider.on_changed(update_trajectory)
    angle_slider.on_changed(update_trajectory)
    
    # Connect event handlers
    fig.canvas.mpl_connect('button_press_event', on_court_click)
    fig.canvas.mpl_connect('button_press_event', on_press)
    fig.canvas.mpl_connect('motion_notify_event', on_motion)
    fig.canvas.mpl_connect('button_release_event', on_release)
    
    # Add instruction text with better styling
    instruction_text = plt.figtext(0.5, 0.01,
                                "Click on court to select position. Min angle: 60°. Min trajectory height: 6000mm. Click 'Calculate Parameters' for visual comparison.",
                                ha="center", fontsize=11, fontweight='bold',
                                bbox=dict(facecolor='#E8E8E8', alpha=0.8, boxstyle='round,pad=0.5', edgecolor='gray'))
    
    # Initialize the trajectory view with a message
    traj_ax.text(0.5, 0.5, "Select a robot position on the court to begin",
               ha='center', va='center', transform=traj_ax.transAxes,
               fontsize=14, fontweight='bold', color='gray')
    
    plt.tight_layout(rect=[0, 0.15, 1, 0.98])
    plt.show()

# Run the simulator
if __name__ == "__main__":
    combined_basketball_simulator()
