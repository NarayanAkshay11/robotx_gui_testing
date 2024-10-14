import dearpygui.dearpygui as dpg
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import datetime
import numpy as np

# Initialize ROS Node
rospy.init_node('robotx_gui_node', anonymous=True)

# Global variables
bridge = CvBridge()
video_frame_1 = np.zeros((400, 600, 3), np.uint8)  # Placeholder for video feed 1
video_frame_2 = np.zeros((400, 600, 3), np.uint8)  # Placeholder for video feed 2
nmea_log = []
log_file_path = "robotx_logs.txt"  # Path to the log file
MAX_LOGS = 100  # Maximum number of logs to keep

# Callback for video feed 1
def video_feed_callback_1(data):
    global video_frame_1
    video_frame_1 = bridge.imgmsg_to_cv2(data, "bgr8")

# Callback for video feed 2
def video_feed_callback_2(data):
    global video_frame_2
    video_frame_2 = bridge.imgmsg_to_cv2(data, "bgr8")

# Subscribe to ROS topics for video feeds
rospy.Subscriber("/camera1/image_raw", Image, video_feed_callback_1)
rospy.Subscriber("/camera2/image_raw", Image, video_feed_callback_2)

# GPS callback to update latitude and longitude
def gps_callback(data):
    lat, lon = data.data.split(',')
    log_nmea_message("GPS", [lat, lon])
    dpg.set_value("gps_lat", f"Latitude: {lat}")
    dpg.set_value("gps_lon", f"Longitude: {lon}")

# Task 1 callback for obstacle avoidance
def task1_callback(data):
    distance = data.data  # Assuming this is a string with distance
    log_nmea_message("TASK1", [distance])
    dpg.set_value("obstacle_distance", float(distance))  # Update the UI element

# Task 2 callback for gate status
def task2_callback(data):
    status = data.data  # Assuming this is a string representing gate status
    log_nmea_message("TASK2", [status])
    dpg.set_value("gate_status", f"Gate Status: {status}")  # Update the UI element

# Subscribe to GPS topic (update as needed)
rospy.Subscriber("/vehicle/gps", String, gps_callback)
rospy.Subscriber("/vehicle/task1/distance", String, task1_callback)  # Placeholder for task 1 distance data
rospy.Subscriber("/vehicle/task2/status", String, task2_callback)  # Placeholder for task 2 gate status data

# NMEA Log Format
def generate_nmea_message(message_type, data_fields):
    checksum = 0
    message = f"${message_type},{','.join(data_fields)}"
    for char in message[1:]:
        checksum ^= ord(char)
    message += f"*{checksum:02X}\r\n"
    return message

# Log the NMEA messages
def log_nmea_message(message_type, data_fields):
    message = generate_nmea_message(message_type, data_fields)
    nmea_log.append(f"{datetime.datetime.now()} - {message}")
    
    # Maintain a log size of MAX_LOGS
    if len(nmea_log) > MAX_LOGS:
        nmea_log.pop(0)
    
    # Update the log window
    dpg.set_value("log_window", "\n".join(nmea_log))
    
    # Write to log file
    with open(log_file_path, "a") as log_file:
        log_file.write(f"{message}\n")  # Write only the NMEA message

# GUI Initialization
dpg.create_context()

# Set up the GUI theme
with dpg.font_registry():
    roboto_font = dpg.add_font("Roboto-Regular.ttf", 20)

with dpg.theme() as theme:
    with dpg.theme_component(dpg.mvAll):
        dpg.add_theme_color(dpg.mvThemeCol_WindowBg, (25, 25, 112, 255))  # Navy Blue
        dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255, 255))    # White
        dpg.add_theme_color(dpg.mvThemeCol_Border, (0, 0, 0, 255))        # Black
        dpg.bind_item_font(dpg.mvAll, roboto_font)

# Function to create task-specific UI components
def update_task_ui(task_number):
    dpg.delete_item("task_container", children=True)  # Clear previous task UI

    # Create task-specific UI elements based on the task number
    if task_number == 1:
        dpg.add_text("Task 1: Obstacle Avoidance", color=(255, 255, 255))
        dpg.add_button(label="Start Task 1", callback=lambda: log_nmea_message("TASK1", ["Starting", "Task 1"]))
        dpg.add_text("Obstacle Distance: ")
        dpg.add_drag_float(label="Distance to Obstacle", tag="obstacle_distance", default_value=0.0)

    elif task_number == 2:
        dpg.add_text("Task 2: Entrance and Exit Gates", color=(255, 255, 255))
        dpg.add_button(label="Check Gates", callback=lambda: log_nmea_message("TASK2", ["Checking", "Gates"]))
        dpg.add_text("Gate Status: ", tag="gate_status")
    
    elif task_number == 3:
        dpg.add_text("Task 3: Follow the Path", color=(255, 255, 255))
        dpg.add_button(label="Follow Path", callback=lambda: log_nmea_message("TASK3", ["Following", "Path"]))
        dpg.add_text("Current Path Position: ", tag="current_path_position")

    elif task_number == 4:
        dpg.add_text("Task 4: Wildlife Encounter", color=(255, 255, 255))
        dpg.add_button(label="Start Wildlife Task", callback=lambda: log_nmea_message("TASK4", ["Starting", "Wildlife Task"]))
        dpg.add_text("Detected Colors: ", tag="detected_colors")

    elif task_number == 5:
        dpg.add_text("Task 5: Scan the Code", color=(255, 255, 255))
        dpg.add_button(label="Scan Code", callback=lambda: log_nmea_message("TASK5", ["Scanning", "Code"]))
        dpg.add_text("Scanned Colors: ", tag="scanned_colors")

    elif task_number == 6:
        dpg.add_text("Task 6: Dock and Deliver", color=(255, 255, 255))
        dpg.add_button(label="Dock", callback=lambda: log_nmea_message("TASK6", ["Docking", "Now"]))
        dpg.add_text("Docking Status: ", tag="docking_status")

    elif task_number == 7:
        dpg.add_text("Task 7: UAV Replenishment", color=(255, 255, 255))
        dpg.add_button(label="Replenish", callback=lambda: log_nmea_message("TASK7", ["Replenishing", "Now"]))
        dpg.add_text("Replenishment Status: ", tag="replenishment_status")

    elif task_number == 8:
        dpg.add_text("Task 8: UAV Search and Report", color=(255, 255, 255))
        dpg.add_button(label="Start Search", callback=lambda: log_nmea_message("TASK8", ["Starting", "Search"]))
        dpg.add_text("Search Status: ", tag="search_status")

# Main GUI Layout
with dpg.window(label="RobotX Ground Station", width=1200, height=800):

    # Task Selection Dropdown
    dpg.add_text("Select Task", color=(255, 255, 255))
    task_selector = dpg.add_combo(["Task 1", "Task 2", "Task 3", "Task 4", "Task 5", "Task 6", "Task 7", "Task 8"],
                                   label="Tasks", callback=lambda s, a: update_task_ui(a + 1))

    # GPS Status Section
    dpg.add_text("GPS Position", color=(255, 255, 255))
    dpg.add_text("Latitude: ", tag="gps_lat", color=(255, 255, 255))
    dpg.add_text("Longitude: ", tag="gps_lon", color=(255, 255, 255))

    # NMEA Message Logs
    dpg.add_text("NMEA Log", color=(255, 255, 255))
    dpg.add_text("\n".join(nmea_log), tag="log_window", wrap=500)

    # Task Container
    with dpg.child_window(tag="task_container", width=1200, height=200):
        update_task_ui(1)  # Initialize with Task 1 UI

    # Two Video Feeds for Cameras
    with dpg.child_window(width=600, height=400):
        dpg.add_text("Video Feed 1", color=(255, 255, 255))
        dpg.add_image("video_texture_1", width=600, height=400)
    
    with dpg.child_window(width=600, height=400):
        dpg.add_text("Video Feed 2", color=(255, 255, 255))
        dpg.add_image("video_texture_2", width=600, height=400)

# Helper to convert OpenCV image to DearPyGui texture
def convert_cv_to_dpg_texture(image):
    image = cv2.flip(image, 0)  # Flip the image for OpenGL
    return image.ravel()  # Flatten the image

# Updating the GUI with video frames and GPS
def update_gui():
    if video_frame_1 is not None:
        dpg.set_value("video_texture_1", convert_cv_to_dpg_texture(video_frame_1))
    if video_frame_2 is not None:
        dpg.set_value("video_texture_2", convert_cv_to_dpg_texture(video_frame_2))

# Set up the viewport and callbacks
dpg.create_viewport(title='RobotX Ground Station', width=1280, height=1024)
dpg.setup_dearpygui()
dpg.set_viewport_clear_color((25, 25, 112, 255))  # Navy blue background
dpg.show_viewport()

# Continuous GUI Update Loop
while not rospy.is_shutdown():
    update_gui()
    dpg.render_dearpygui_frame()

# Destroy the GUI when done
dpg.destroy_context()
