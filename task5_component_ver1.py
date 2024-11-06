#!/usr/bin/env python
import dearpygui.dearpygui as dpg
import rospy
from std_msgs.msg import ColorRGBA
import csv
from datetime import datetime
import threading
import time

class ColorDisplayNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('color_display_node', anonymous=True)
        
        # Define colors (normalized for ROS ColorRGBA)
        self.RED = (255, 0, 0, 255)
        self.GREEN = (0, 255, 0, 255)
        self.BLUE = (0, 0, 255, 0)
        self.BLACK = (0, 0, 0, 255)
        
        # Initialize variables
        self.box_size_pixels = int(3 * 96 / 2.54)  # 3cm to pixels at 96 DPI
        self.current_colors = [self.BLACK] * 11
        self.processing_color = False
        self.color_log = []
        self.current_box_index = 0
        
        # Set up ROS subscriber
        self.color_sub = rospy.Subscriber(
            '/detected_color',  # Replace with your actual topic name
            ColorRGBA,
            self.color_callback,
            queue_size=1
        )
        
        # Initialize DPG
        dpg.create_context()
        self.setup_dpg()
        
    def setup_dpg(self):
        # Create themes for each color
        self.themes = {
            'red': self.create_color_theme(self.RED),
            'green': self.create_color_theme(self.GREEN),
            'blue': self.create_color_theme(self.BLUE),
            'black': self.create_color_theme(self.BLACK)
        }
        
        # Create main window
        with dpg.window(label="Task 5 Light Pattern", width=800, height=400, no_collapse=True):
            dpg.add_text("Scan the Code")
            
            # Create horizontal group for boxes
            with dpg.group(horizontal=True):
                for i in range(11):
                    tag = f"square_{i}"
                    dpg.add_button(tag=tag, width=self.box_size_pixels, height=self.box_size_pixels)
                    dpg.bind_item_theme(tag, self.themes['black'])
    
    def create_color_theme(self, color):
        with dpg.theme() as theme_id:
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvThemeCol_Button, color, category=dpg.mvThemeCat_Core)
        return theme_id
    
    def update_display(self):
        for i, color in enumerate(self.current_colors):
            if color == self.RED:
                dpg.bind_item_theme(f"square_{i}", self.themes['red'])
            elif color == self.GREEN:
                dpg.bind_item_theme(f"square_{i}", self.themes['green'])
            elif color == self.BLUE:
                dpg.bind_item_theme(f"square_{i}", self.themes['blue'])
            else:
                dpg.bind_item_theme(f"square_{i}", self.themes['black'])
    
    def check_consecutive_colors(self):
        # Check for consecutive duplicate colors
        for i in range(len(self.current_colors) - 1):
            if (self.current_colors[i] != self.BLACK and 
                self.current_colors[i] == self.current_colors[i + 1]):
                return True
        # Check for three consecutive same colors
        for i in range(len(self.current_colors) - 2):
            if (self.current_colors[i] != self.BLACK and 
                self.current_colors[i] == self.current_colors[i + 1] == self.current_colors[i + 2]):
                return True
        return False
    
    def reset_colors(self):
        self.current_colors = [self.BLACK] * 11
        self.current_box_index = 0
        self.update_display()
    
    def save_to_csv(self):
        if any(color != self.BLACK for color in self.current_colors):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            with open(f"color_log_{timestamp}.csv", mode="a", newline="") as file:
                writer = csv.writer(file)
                # Convert colors to a more readable format for CSV
                color_names = []
                for color in self.current_colors:
                    if color == self.RED:
                        color_names.append('RED')
                    elif color == self.GREEN:
                        color_names.append('GREEN')
                    elif color == self.BLUE:
                        color_names.append('BLUE')
                    else:
                        color_names.append('BLACK')
                writer.writerow(color_names)
    
    def shift_colors_left(self):
        self.current_colors = self.current_colors[1:] + [self.BLACK]
        self.current_box_index = 10  # Set to last position
        self.update_display()
    
    def color_callback(self, msg):
        if self.processing_color:
            return
            
        self.processing_color = True
        
        # Convert ROS color message to our format
        received_color = (
            int(msg.r * 255),
            int(msg.g * 255),
            int(msg.b * 255),
            int(msg.a * 255)
        )
        
        # Process the received color
        if received_color == self.BLACK:
            self.current_colors[self.current_box_index] = self.BLACK
            self.update_display()
            threading.Timer(2.0, self.set_processing_false).start()
        else:
            self.current_colors[self.current_box_index] = received_color
            
            if self.check_consecutive_colors():
                self.reset_colors()
                threading.Timer(1.0, self.set_processing_false).start()
            else:
                self.update_display()
                if self.current_box_index == 10:  # Last box
                    self.save_to_csv()
                    self.shift_colors_left()
                else:
                    self.current_box_index += 1
                self.processing_color = False
    
    def set_processing_false(self):
        self.processing_color = False
    
    def run(self):
        # Setup viewport
        dpg.create_viewport(title='Task 5 Light Pattern', width=850, height=450)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        
        # Main loop
        while not rospy.is_shutdown():
            dpg.render_dearpygui_frame()
        
        # Cleanup
        dpg.destroy_context()

if __name__ == "__main__":
    try:
        node = ColorDisplayNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
