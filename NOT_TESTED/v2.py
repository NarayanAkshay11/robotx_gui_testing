import dearpygui.dearpygui as dpg
import rospy
from std_msgs.msg import ColorRGBA
import time

# Initialize colors
black = (0, 0, 0)
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)

def create_task5_light_pattern(viewport_width, viewport_height, ros_topic="/color_sequence"):
    """Creates Task 5 Light Pattern component and subscribes to ROS for color updates."""
    
    box_size_pixels = int(3 * 96 / 2.54)  # Convert 3 cm to pixels (approximately 113 pixels)
    spacing = int(box_size_pixels * 0.2)  # Set spacing as 20% of box size

    # Initialize Task 5 GUI Elements
    def init_task5_gui():
        dpg.add_group(tag="task5", parent="main_window")
        dpg.set_item_width("task5", int(viewport_width * 0.25))
        dpg.set_item_height("task5", int(viewport_height * 0.1))
        dpg.set_item_pos("task5", [int(viewport_width * 0.665), int(viewport_height * 0.624)])

        # Initialize the visual display for Task 5
        dpg.add_group(tag="task5vd", parent="main_window")
        dpg.set_item_width("task5vd", int(viewport_width * 0.25))
        dpg.set_item_height("task5vd", int(viewport_height * 0.2))
        dpg.set_item_pos("task5vd", [int(viewport_width * 0.665), int(viewport_height * 0.732)])

        # Setting text and color controls
        window_width = dpg.get_item_width("task5vd")
        total_width = (box_size_pixels * 3) + (spacing * 2)
        x_pos = (window_width - total_width) / 2
        y_pos = 50
        tx_pos = (window_width - 91) / 2

        with dpg.group(parent="task5vd"):
            dpg.add_text("Scan the Code", pos=[tx_pos, 25], tag="vdtext")
            dpg.add_drag_int(tag="vdheader0", format="Red", enabled=False, width=box_size_pixels, pos=[x_pos, y_pos])
            dpg.add_drag_int(tag="vdheader1", format="Green", enabled=False, width=box_size_pixels, pos=[x_pos + box_size_pixels + spacing, y_pos])
            dpg.add_drag_int(tag="vdheader2", format="Blue", enabled=False, width=box_size_pixels, pos=[x_pos + 2 * (box_size_pixels + spacing), y_pos])

    # Function to update colors
    def update_colors(colors):
        # Check if two or more consecutive colors are the same
        if len(colors) >= 2 and any(colors[i] == colors[i+1] for i in range(len(colors)-1)):
            # Set all squares to black if repeated colors found
            colors = [black] * 10
            rospy.sleep(1)  # Pause for 1 second before next attempt
        else:
            # Set colors based on sequence
            color_tags = ["s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9"]
            for i, color in enumerate(colors[:10]):  # Only update the first 10 items
                dpg.configure_item(color_tags[i], fill=color)

    # ROS Subscriber Callback
    def ros_callback(msg):
        # Assuming message contains a list of ColorRGBA
        color_sequence = [(int(c.r * 255), int(c.g * 255), int(c.b * 255)) for c in msg.data]
        update_colors(color_sequence)

    # Set up the ROS subscriber
    rospy.init_node("task5_light_pattern_node", anonymous=True)
    rospy.Subscriber(ros_topic, ColorRGBA, ros_callback)

    # Initialize GUI
    init_task5_gui()

# Usage example:
# Assume main viewport dimensions are set
viewport_width = 800
viewport_height = 600
create_task5_light_pattern(viewport_width, viewport_height)

