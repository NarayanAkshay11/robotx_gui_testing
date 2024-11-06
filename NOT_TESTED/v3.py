import dearpygui.dearpygui as dpg
import random
import csv

# Define colors
red = (255, 0, 0, 255)
green = (0, 255, 0, 255)
blue = (0, 0, 255, 255)
black = (0, 0, 0, 255)

colors_list = [red, green, blue]

# Convert 3 cm to pixels (assuming 96 DPI)
box_size_pixels = int(3 * 96 / 2.54)  # Approx. 113 pixels

# Color log for storing color sequences
color_log = []

# Theme creation function
def create_color_theme(color):
    with dpg.theme() as theme_id:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, color, category=dpg.mvThemeCat_Core)
    return theme_id

# Create color themes for each color
black_theme = create_color_theme(black)
red_theme = create_color_theme(red)
green_theme = create_color_theme(green)
blue_theme = create_color_theme(blue)

def create_task5_light_pattern():
    # Create the main window
    with dpg.window(label="Scan the Code", width=800, height=400, no_collapse=True):
        dpg.add_text("Scan the Code")

        # Create a horizontal group for the boxes
        with dpg.group(horizontal=True):
            # Initialize 11 black boxes
            for i in range(11):
                tag = f"square_{i}"
                dpg.add_button(tag=tag, width=box_size_pixels, height=box_size_pixels)
                dpg.bind_item_theme(tag, black_theme)  # Set initial color to black

        # Add button to trigger color update manually
        dpg.add_button(label="Update Colors", callback=update_colors)

def update_colors():
    global color_log
    color_sequence = [random.choice(colors_list) for _ in range(11)]  # Simulated color sequence

    # Check for consecutive duplicate colors and reset to black if found
    if any(color_sequence[i] == color_sequence[i + 1] for i in range(len(color_sequence) - 1)):
        for i in range(11):
            dpg.bind_item_theme(f"square_{i}", black_theme)  # Set all to black if there’s a repeat
    else:
        # Store non-black colors in color_log
        color_log.extend([color for color in color_sequence if color != black])

        # Write to CSV file in rows of 3 colors each
        with open("color_log.csv", mode="a", newline="") as file:
            writer = csv.writer(file)
            writer.writerows([color_log[i:i+3] for i in range(0, len(color_log), 3)])
        color_log = []  # Reset color log after saving

        # Update the colors in GUI
        for i, color in enumerate(color_sequence):
            if color == red:
                dpg.bind_item_theme(f"square_{i}", red_theme)
            elif color == green:
                dpg.bind_item_theme(f"square_{i}", green_theme)
            elif color == blue:
                dpg.bind_item_theme(f"square_{i}", blue_theme)

# Initialize DPG
dpg.create_context()
dpg.create_viewport(title='Task 5 Light Pattern')
dpg.setup_dearpygui()

# Main GUI setup
create_task5_light_pattern()

# Show and start the GUI
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()

