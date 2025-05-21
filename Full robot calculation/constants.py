"""
Constants and configuration for the basketball simulator.
"""
import matplotlib.pyplot as plt

# Court dimensions and basketball parameters (in mm)
COURT_WIDTH = 15000
COURT_HEIGHT = 8000
BASKET_HEIGHT = 2430
MAX_THROW_DISTANCE = 7000
MIN_TRAJECTORY_HEIGHT = 3000  # Minimum height for trajectory
ROBOT_HEIGHT = 1000  # Height of the robot's shooting mechanism
BASKET_RADIUS = 450  # Radius of the basketball hoop
MIN_ANGLE = 60  # Minimum launch angle in degrees
MAX_VELOCITY = 15.0  # Maximum initial velocity in m/s
X_OFFSET = 0.5  # X offset for the robot's position

# Colors for visualization
COLORS = {
    'court_bg': '#F0F0F0',
    'court_lines': '#333333',
    'valid_area': '#DEEAFC',
    'valid_border': '#4285F4',
    'invalid_color': '#FA8072',
    'valid_color': '#90EE90',
    'basket_color': '#FF4500',
    'robot_color': '#228B22',
    'trajectory': '#0066CC',
    'warning': '#FF4500',
    'success': '#28A745',
    'slider_bg': '#EFEFEF',
    'button_color': '#4285F4',
    'button_disabled': '#B0B0B0',
    'min_height_line': '#FF8C00'
}

# Calculate 2D distance for a 3D distance of 6000mm with height 2430mm
import math
HORIZONTAL_DISTANCE = math.sqrt(MAX_THROW_DISTANCE**2 - BASKET_HEIGHT**2)

# Plot style settings
def set_plot_style():
    plt.style.use('seaborn-whitegrid')
