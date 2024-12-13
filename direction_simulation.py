import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Constants
NUM_SENSORS = 3
PHOTORESISTOR_ANGLES = [np.pi / 3, np.pi, 5 * np.pi / 3]  # Angles in radians
WHEEL_ANGLE_OFFSET = [0, 2 * np.pi / 3, 4 * np.pi / 3]
ROBOT_RADIUS = 1.0
LIGHT_THRESHOLD = 0.1  # Adjusted for normalized light values

def generate_light_values(light_source_angle):
    """
    Simulate light sensor readings based on light source angle.
    
    Args:
        light_source_angle (float): Angle of the light source in radians
    
    Returns:
        list: Normalized light values for each sensor
    """
    light_values = []
    for angle in PHOTORESISTOR_ANGLES:
        # Calculate the x, y position of the light source
        light_x = 1.5 * ROBOT_RADIUS * np.cos(light_source_angle)
        light_y = 1.5 * ROBOT_RADIUS * np.sin(light_source_angle)

        # Calculate the x, y position of the photoresistor
        photo_x = ROBOT_RADIUS * np.cos(angle)
        photo_y = ROBOT_RADIUS * np.sin(angle)

        # Compute the Euclidean distance between light source and photoresistor
        distance = np.sqrt((light_x - photo_x) ** 2 + (light_y - photo_y) ** 2) 

        # Calculate intensity inversely proportional to distance squared
        intensity = 1 / (distance ** 2) if distance != 0 else float('inf')
        light_values.append(intensity)

    # Normalize light values to range [0, 1]
    max_intensity = max(light_values) if light_values else 1
    light_values = [value / max_intensity for value in light_values]
    return light_values

def find_brightest_light_angle(light_values):
    """
    Find the brightest light angle using weighted average method.
    
    Args:
        light_values (list): Normalized light sensor readings
    
    Returns:
        float or None: Angle of the brightest light, or None if light is uniform
    """
    max_value = max(light_values)
    min_value = min(light_values)

    # Check if light values are too close to distinguish
    if max_value - min_value < LIGHT_THRESHOLD:
        return None

    # Find indices of the two highest light values
    sorted_indices = np.argsort(light_values)[-2:]  # Get two largest indices
    max_index1, max_index2 = sorted_indices[1], sorted_indices[0]

    # Get corresponding angles and weights
    angle1 = PHOTORESISTOR_ANGLES[max_index1]
    angle2 = PHOTORESISTOR_ANGLES[max_index2]
    weight1 = light_values[max_index1]
    weight2 = light_values[max_index2]

    if weight1 + weight2 == 0:
        return 0  # Default angle if no light detected

    # Convert to Cartesian coordinates
    x1, y1 = weight1 * np.cos(angle1), weight1 * np.sin(angle1)
    x2, y2 = weight2 * np.cos(angle2), weight2 * np.sin(angle2)

    # Average in Cartesian space
    avg_x = x1 + x2
    avg_y = y1 + y2

    # Convert back to polar angle
    brightest_angle = np.arctan2(avg_y, avg_x)

    # Ensure angle is in [0, 2π] range
    return brightest_angle if brightest_angle >= 0 else brightest_angle + 2 * np.pi

def vector_to_wheel_speeds(vx, vy):
    """
    Convert robot movement vector to individual wheel speeds.
    
    Args:
        vx (float): X component of movement vector
        vy (float): Y component of movement vector
    
    Returns:
        list: Calculated wheel speeds
    """
    wheel_speeds = []
    for angle in WHEEL_ANGLE_OFFSET:
        # Inverse kinematics for holonomic wheel configuration
        wheel_speed = -np.sin(angle) * vx + np.cos(angle) * vy
        wheel_speeds.append(wheel_speed)
    return wheel_speeds

def move_robot_towards_angle(angle):
    """
    Calculate wheel speeds to move robot towards a specific angle.
    
    Args:
        angle (float): Desired movement angle in radians
    
    Returns:
        list: Wheel speeds for each motor
    """
    # Convert angle to vector components
    vx = np.cos(angle)
    vy = np.sin(angle)
    
    # Calculate wheel speeds based on vector
    return vector_to_wheel_speeds(vx, vy)

def plot_robot_movement(ax, light_source_angle, light_values, brightest_angle):
    """
    Visualize robot, light source, sensors, and movement.
    
    Args:
        ax (matplotlib.axes.Axes): Matplotlib axes to plot on
        light_source_angle (float): Angle of light source
        light_values (list): Light sensor readings
        brightest_angle (float): Calculated brightest light angle
    """
    ax.clear()

    # Draw the robot as a circle
    circle = plt.Circle((0, 0), ROBOT_RADIUS, color='blue', fill=False)
    ax.add_patch(circle)

    # Plot photoresistors
    for angle, light_value in zip(PHOTORESISTOR_ANGLES, light_values):
        x = ROBOT_RADIUS * np.cos(angle)
        y = ROBOT_RADIUS * np.sin(angle)
        ax.plot(x, y, 'ro')  # Red dots for photoresistors
        ax.text(x, y, f"{light_value:.2f}", color='red', fontsize=8, ha='center')

    # Plot wheels
    for angle in WHEEL_ANGLE_OFFSET:
        x = 0.9 * ROBOT_RADIUS * np.cos(angle)
        y = 0.9 * ROBOT_RADIUS * np.sin(angle)
        ax.plot(x, y, 'go')  # Green dots for wheels

    # Plot light source
    light_x = 1.5 * ROBOT_RADIUS * np.cos(light_source_angle)
    light_y = 1.5 * ROBOT_RADIUS * np.sin(light_source_angle)
    ax.plot(light_x, light_y, 'yo', markersize=10, label="Light Source")

    # Plot brightest angle
    if brightest_angle is not None:
        brightest_x = 1.5 * ROBOT_RADIUS * np.cos(brightest_angle)
        brightest_y = 1.5 * ROBOT_RADIUS * np.sin(brightest_angle)
        ax.arrow(0, 0, brightest_x, brightest_y, color='purple', head_width=0.1, label="Brightest Angle")
        
        # Calculate and display angle in degrees
        angle_deg = np.degrees(brightest_angle)
        ax.text(brightest_x, brightest_y, f"{angle_deg:.1f}°", color='purple')

        # Calculate and display wheel speeds
        wheel_speeds = move_robot_towards_angle(brightest_angle)
        speed_text = "Wheel Speeds:\n"
        for i, speed in enumerate(wheel_speeds, 1):
            speed_text += f"Wheel {i}: {speed:.2f}\n"
        ax.text(2, 1.5, speed_text, bbox=dict(facecolor='white', alpha=0.7))

    # Configure plot
    ax.legend()
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.axhline(0, color='gray', linestyle='--', linewidth=0.5)
    ax.axvline(0, color='gray', linestyle='--', linewidth=0.5)
    ax.set_aspect('equal', adjustable='box')
    ax.set_title("Robot Movement Simulation")

# Interactive simulation
def update(val):
    """
    Update plot based on slider value.
    
    Args:
        val (float): Current slider value (light source angle)
    """
    light_source_angle = slider.val
    light_values = generate_light_values(light_source_angle)
    brightest_angle = find_brightest_light_angle(light_values)
    plot_robot_movement(ax, light_source_angle, light_values, brightest_angle)
    plt.draw()

# Create figure and slider
fig, ax = plt.subplots(figsize=(10, 8))
plt.subplots_adjust(bottom=0.2)

slider_ax = plt.axes([0.2, 0.05, 0.65, 0.03], facecolor='lightgoldenrodyellow')
slider = Slider(slider_ax, 'Light Angle', 0, 2 * np.pi, valinit=0, valstep=0.01)

slider.on_changed(update)

# Initial plot
update(0)
plt.show()

