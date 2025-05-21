import math
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon
from matplotlib.widgets import Button, Slider

# Robocon 2025 Basketball Court dimensions (in mm)
# Adjust these values to match the official rulebook specifications.
COURT_WIDTH = 15000     # e.g., 15 meters
COURT_HEIGHT = 2430*3   # e.g., 2.430 meters

# Fixed basket position; typically placed at the center of the far end per rulebook.
BASKET_POS = (COURT_WIDTH - 600, COURT_HEIGHT/3)

# Assumed camera FOV parameters (horizontal angles in degrees)
DEPTH_FOV = 40      # Depth camera field-of-view horizontally
RGB_FOV = 45        # RGB camera field-of-view horizontally

# Global variables to store robot position and camera orientation (in degrees)
robot_pos = (COURT_WIDTH/2, 1120)  # initialize with fixed y
calculated_cam_angle = None  # current camera angle (manual or auto) in degrees
camera_basket_line = None    # handle for line from camera (robot) to basket

# Create figure and axis for court view
fig, ax = plt.subplots(figsize=(10, 6))
plt.subplots_adjust(bottom=0.25)
ax.set_xlim(0, COURT_WIDTH)
ax.set_ylim(0, COURT_HEIGHT)
ax.set_xlabel("X (mm)")
ax.set_ylabel("Y (mm)")
ax.set_title("Robocon 2025 Basketball Court & Camera Calibration")
ax.grid(True)

# Draw court outline
court_rect = Rectangle((0, 0), COURT_WIDTH, COURT_HEIGHT, fill=False, edgecolor='black', linewidth=2)
ax.add_patch(court_rect)

# Draw center line of the court
center_line = plt.Line2D([COURT_WIDTH/2, COURT_WIDTH/2], [0, COURT_HEIGHT],
                           color='gray', linestyle='--', linewidth=2, label='Center Line')
ax.add_line(center_line)

# Draw basket as a circle marker
basket_marker = ax.plot(BASKET_POS[0], BASKET_POS[1], 'o', color='orange', markersize=10, label='Basket')[0]

# Hold handles for the robot and FOV drawing
robot_marker = None
depth_fov_patch = None
rgb_fov_patch = None
cam_text = ax.text(0.5, 0.95, "", transform=ax.transAxes, fontsize=12, va='top', ha='center',
                   bbox=dict(facecolor="white", alpha=0.8, boxstyle="round"))

def update_calibration():
    global depth_fov_patch, rgb_fov_patch, cam_text, calculated_cam_angle, camera_basket_line, robot_pos
    if robot_pos is None:
        return

    # Update x-axis from slider while keeping y constant.
    new_x = slider_x.val
    robot_pos = (new_x, robot_pos[1])
    
    # Compute auto angle (for reference) but then use manual slider value as the final angle.
    auto_angle = math.degrees(math.atan2(BASKET_POS[1] - robot_pos[1], BASKET_POS[0] - robot_pos[0]))
    final_angle = slider_manual.val  # keep previously set manual angle
    
    calculated_cam_angle = final_angle

    # Remove previous patches if they exist.
    if depth_fov_patch is not None:
        depth_fov_patch.remove()
    if rgb_fov_patch is not None:
        rgb_fov_patch.remove()
    if camera_basket_line is not None:
        camera_basket_line.remove()
    
    # Draw a dashed line from camera (robot) to basket.
    camera_basket_line, = ax.plot([robot_pos[0], BASKET_POS[0]], [robot_pos[1], BASKET_POS[1]],
                                  linestyle='--', color='purple', label='Camera -> Basket')
    
    # Compute FOV lines for RGB cam using final_angle.
    half_rgb = RGB_FOV / 2
    rgb_left_angle = math.radians(final_angle - half_rgb)
    rgb_right_angle = math.radians(final_angle + half_rgb)
    rgb_cone_length = 1e6
    rgb_left_pt = (robot_pos[0] + rgb_cone_length * math.cos(rgb_left_angle),
                   robot_pos[1] + rgb_cone_length * math.sin(rgb_left_angle))
    rgb_right_pt = (robot_pos[0] + rgb_cone_length * math.cos(rgb_right_angle),
                    robot_pos[1] + rgb_cone_length * math.sin(rgb_right_angle))
    rgb_poly_pts = [robot_pos, rgb_left_pt, rgb_right_pt]
    rgb_fov_patch = Polygon(rgb_poly_pts, closed=True, color='green', alpha=0.2, label='RGB FOV')
    ax.add_patch(rgb_fov_patch)
    
    # Compute FOV lines for depth cam.
    depth_lower_angle = math.radians(final_angle - half_rgb)
    depth_right_angle = math.radians(final_angle + DEPTH_FOV / 2)
    depth_cone_length = 6000
    depth_lower_pt = (robot_pos[0] + depth_cone_length * math.cos(depth_lower_angle),
                      robot_pos[1] + depth_cone_length * math.sin(depth_lower_angle))
    depth_right_pt = (robot_pos[0] + depth_cone_length * math.cos(depth_right_angle),
                      robot_pos[1] + depth_cone_length * math.sin(depth_right_angle))
    depth_poly_pts = [robot_pos, depth_lower_pt, depth_right_pt]
    depth_fov_patch = Polygon(depth_poly_pts, closed=True, color='blue', alpha=0.2, label='Depth FOV')
    ax.add_patch(depth_fov_patch)
    
    # Update text info.
    cam_text.set_text(f"Camera X: {robot_pos[0]:.1f} mm\nAngle: {final_angle:.1f}°")
    fig.canvas.draw_idle()

def on_click(event):
    global robot_pos, robot_marker
    if event.inaxes != ax:
        return

    new_x = event.xdata
    robot_pos = (new_x, robot_pos[1])
    if robot_marker is not None:
        robot_marker.remove()
    robot_marker = ax.plot(robot_pos[0], robot_pos[1], 'o', color='red', markersize=8, label='Robot')[0]
    slider_x.set_val(new_x)
    update_calibration()

# Create a slider for camera x-axis input.
slider_ax = plt.axes([0.15, 0.15, 0.65, 0.03])
slider_x = Slider(slider_ax, 'Camera X', 0, COURT_WIDTH, valinit=robot_pos[0], valstep=1)
slider_x.on_changed(lambda val: update_calibration())

# Add a manual angle slider.
auto_angle_init = math.degrees(math.atan2(BASKET_POS[1] - robot_pos[1], BASKET_POS[0] - robot_pos[0]))
slider_manual_ax = plt.axes([0.15, 0.10, 0.65, 0.03])
slider_manual = Slider(slider_manual_ax, 'Manual Angle', -180, 180, valinit=auto_angle_init, valstep=0.1)
slider_manual.on_changed(lambda val: update_calibration())

# Modify reset() to update both sliders.
def reset(event):
    global robot_pos, robot_marker, depth_fov_patch, rgb_fov_patch, cam_text, camera_basket_line
    robot_pos = (COURT_WIDTH/2, COURT_HEIGHT/2)
    if robot_marker is not None:
        robot_marker.remove()
    if depth_fov_patch is not None:
        depth_fov_patch.remove()
    if rgb_fov_patch is not None:
        rgb_fov_patch.remove()
    if camera_basket_line is not None:
        camera_basket_line.remove()
    cam_text.set_text("")
    slider_x.set_val(robot_pos[0])
    new_auto_angle = math.degrees(math.atan2(BASKET_POS[1] - robot_pos[1], BASKET_POS[0] - robot_pos[0]))
    slider_manual.set_val(new_auto_angle)
    fig.canvas.draw_idle()

reset_ax = plt.axes([0.82, 0.05, 0.12, 0.04])
reset_button = Button(reset_ax, 'Reset', color='lightgray', hovercolor='0.975')
reset_button.on_clicked(reset)

# Connect the click event
fig.canvas.mpl_connect('button_press_event', on_click)

ax.legend(loc='lower left')
plt.show()