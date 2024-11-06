import dearpygui.dearpygui as dpg

dpg.create_context()

red = (255, 0, 0, 255)
blue = (0, 0, 255, 255)
green = (0, 255, 0, 255)
black = (0, 0, 0, 255)
battery_percentage1 = 80
battery_percentage2 = 50
gps_reference = [(30, -100), (40, -95), (50, -85), (60, -70), (70, -55), (80, -45), (90, -40)]

def adjust_elements():
    viewport_width = dpg.get_viewport_width()
    viewport_height = dpg.get_viewport_height()

    # Navigation
    dpg.set_item_width("navigation", int(viewport_width*0.4))
    dpg.set_item_height("navigation", int(viewport_height*0.4))
    dpg.set_item_pos("navigation", [int(viewport_width*0.005),int(viewport_height*0.008)])
    for tag in ["navforward", "navsidemove", "navheading"]:
        dpg.set_item_width(tag, int(viewport_width * 0.15))
    dpg.set_item_width("navwindow1", int(viewport_width*0.2))
    dpg.set_item_height("navwindow1", int(viewport_height*0.14))
    for tag in ["movementstep", "headingstep"]:
        dpg.set_item_width(tag, int(viewport_width * 0.12))
    dpg.set_item_width("navwindow2", int(viewport_width*0.2-24))
    dpg.set_item_height("navwindow2", int(viewport_height*0.14))
    for tag in ["startgoal", "endgoal", "benchtest"]:
        dpg.set_item_width(tag, int((dpg.get_item_width("navigation")-32)/3))
        dpg.set_item_height(tag, int(dpg.get_item_height("navigation")*0.08))
    dpg.set_item_width("killpid", int(dpg.get_item_width("navigation")-17))
    dpg.set_item_height("killpid", int(dpg.get_item_height("navigation")*0.08))
    for tag in ["-H", "F", "+H", "L", "B", "R"]:
        dpg.set_item_width(tag, int(dpg.get_item_width("navigation")*0.08))
        dpg.set_item_height(tag, int(dpg.get_item_height("navigation")*0.07))
    dpg.set_item_label("col2", "Setpoint")
    dpg.set_item_label("col3", "Error")
    dpg.set_item_label("col4", "Reading")
    dpg.set_value("11", "Forward")
    dpg.set_value("21", "Sidemove")
    dpg.set_value("31", "Heading")
    dpg.set_value("41", "Pitch")
    dpg.set_value("51", "Roll")
    dpg.set_value("14", "-")
    dpg.set_value("24", "-")

    # Readings
    dpg.set_item_width("readings", int(viewport_width*0.2))
    dpg.set_item_height("readings", int(viewport_height*0.4))
    dpg.set_item_pos("readings", [int(viewport_width*0.005),int(viewport_height*0.416)])
    for tag in ["xpos", "ypos", "zpos", "xvel", "yvel", "zvel"]:
        dpg.set_item_width(tag, int(dpg.get_item_width("readings")*0.233))
    
    # Boat Camera
    dpg.set_item_width("boatcam", int(viewport_width*0.25))
    dpg.set_item_height("boatcam", int(viewport_height*0.3))
    dpg.set_item_pos("boatcam", [int(viewport_width*0.41),int(viewport_height*0.008)])

    # Drone Camera
    dpg.set_item_width("dronecam", int(viewport_width*0.25))
    dpg.set_item_height("dronecam", int(viewport_height*0.3))
    dpg.set_item_pos("dronecam", [int(viewport_width*0.665),int(viewport_height*0.008)])

    # Vehicle Status
    dpg.set_item_width("veh", int(viewport_width*0.25))
    dpg.set_item_height("veh", int(viewport_height*0.5))
    dpg.set_item_pos("veh", [int(viewport_width*0.41),int(viewport_height*0.316)])
    for tag in ["b1voltage", "b1current", "b1temp", "b1capacity", "b2voltage", "b2current", "b2temp", "b2capacity"]:
        dpg.set_item_width(tag, int(dpg.get_item_width("veh")*0.192))
    dpg.set_value("b1capacity", battery_percentage1/100)
    dpg.set_value("b2capacity", battery_percentage2/100)

    # Waypoints
    dpg.set_item_width("waypoints", int(viewport_width*0.25))
    dpg.set_item_height("waypoints", int(viewport_height*0.3))
    dpg.set_item_pos("waypoints", [int(viewport_width*0.665),int(viewport_height*0.316)])
    dpg.set_item_width("gpswaypoints", int(dpg.get_item_width("waypoints")*0.6))
    dpg.set_item_height("gpswaypoints", int(dpg.get_item_height("waypoints")*0.88))

    # Ball Shooter
    dpg.set_item_width("ballshooter", int(viewport_width*0.08))
    dpg.set_item_height("ballshooter", int(viewport_height*0.1))
    dpg.set_item_pos("ballshooter", [int(viewport_width*0.21),int(viewport_height*0.416)])
    dpg.set_item_width("bscombo", int(dpg.get_item_width("ballshooter")-16))
    dpg.set_item_width("firebs", int(dpg.get_item_width("ballshooter")-68))

    # Thrusters
    dpg.set_item_width("thrusters", int(viewport_width*0.11))
    dpg.set_item_height("thrusters", int(viewport_height*0.15))
    dpg.set_item_pos("thrusters", [int(viewport_width*0.295),int(viewport_height*0.416)])
    for tag in ["thr1", "thr2", "thr3", "thr4"]:
        dpg.set_item_width(tag, int(dpg.get_item_width("thrusters")-73))

    # Power Control
    dpg.set_item_width("powercontrol", int(viewport_width*0.11))
    dpg.set_item_height("powercontrol", int(viewport_height*0.15))
    dpg.set_item_pos("powercontrol", [int(viewport_width*0.295),int(viewport_height*0.574)])
    for tag in ["nav", "ouster", "velodyne"]:
        dpg.set_item_width(tag, int(dpg.get_item_width("powercontrol")-94))

    # Emergency Stop
    dpg.set_item_width("estop", int(viewport_width*0.08))
    dpg.set_item_height("estop", int(viewport_height*0.1))
    dpg.set_item_pos("estop", [int(viewport_width*0.21),int(viewport_height*0.524)])
    dpg.set_item_width("estopbutton", int(dpg.get_item_width("estop")-16))

    # Task 5 Light Pattern
    dpg.set_item_width("task5", int(viewport_width*0.25))
    dpg.set_item_height("task5", int(viewport_height*0.1))
    dpg.set_item_pos("task5", [int(viewport_width*0.665),int(viewport_height*0.624)])
    dpg.delete_item("task5", children_only=True)
    t5squares()
    dpg.configure_item("s0", fill=black)
    dpg.configure_item("s1", fill=black)
    dpg.configure_item("s2", fill=red)
    dpg.configure_item("s3", fill=green)
    dpg.configure_item("s4", fill=blue)
    dpg.configure_item("s5", fill=black)
    dpg.configure_item("s6", fill=black)
    dpg.configure_item("s7", fill=red)
    dpg.configure_item("s8", fill=green)
    dpg.configure_item("s9", fill=blue)

    # Task 5 Visual Display
    dpg.set_item_width("task5vd", int(viewport_width*0.25))
    dpg.set_item_height("task5vd", int(viewport_height*0.2))
    dpg.set_item_pos("task5vd", [int(viewport_width*0.665),int(viewport_height*0.732)])
    dpg.delete_item("task5vd", children_only=True)
    window_width = dpg.get_item_width("task5vd")
    square_size = int(window_width*0.2)
    spacing = int(window_width*0.05)
    total_width = (square_size*3)+(spacing*2)
    x_pos = (window_width-total_width)/2
    y_pos = 50
    tx_pos = (window_width-91)/2
    with dpg.group(parent="task5vd"):
        dpg.add_text("Scan the Code", pos=[tx_pos,25], tag="vdtext")
        dpg.add_drag_int(tag="vdheader0", format="Red", enabled=False, width=int(dpg.get_item_width("task5vd")*0.2), pos=[x_pos, y_pos])
        dpg.add_drag_int(tag="vdheader1", format="Green", enabled=False, width=int(dpg.get_item_width("task5vd")*0.2), pos=[x_pos+square_size+spacing, y_pos])
        dpg.add_drag_int(tag="vdheader2", format="Blue", enabled=False, width=int(dpg.get_item_width("task5vd")*0.2), pos=[x_pos+2*(square_size+spacing), y_pos])
    t5squaresvd()
    dpg.configure_item("vd0", fill=red)
    dpg.configure_item("vd1", fill=green)
    dpg.configure_item("vd2", fill=blue)

    
def t5squares():
    window_width = dpg.get_item_width("task5")
    window_height = dpg.get_item_height("task5")
    square_size = int(window_width*0.088)
    spacing = int(window_width*0.01)
    total_width = (square_size*10)+(spacing*9)
    x_pos = (window_width-total_width-16)/2
    y_pos = 0
    with dpg.drawlist(parent="task5", width=window_width-16, height=window_height-35):
        for i in range(10):
            dpg.draw_rectangle(
                [x_pos, y_pos],
                [x_pos+square_size, y_pos+square_size],
                tag=f"s{i}",
                thickness=0
            )
            x_pos += square_size+spacing

def t5squaresvd():
    window_width = dpg.get_item_width("task5vd")
    window_height = dpg.get_item_height("task5vd")
    square_size = int(window_width*0.2)
    spacing = int(window_width*0.05)
    total_width = (square_size*3)+(spacing*2)
    x_pos = (window_width-total_width-16)/2
    y_pos = 3
    with dpg.drawlist(parent="task5vd", width=window_width-16, height=window_height-85):
        for i in range(3):
            dpg.draw_rectangle(
                [x_pos, y_pos],
                [x_pos+square_size, y_pos+square_size],
                tag=f"vd{i}",
                thickness=0
            )
            x_pos += square_size+spacing

def bsstate(sender, app_data):
    if app_data == "Uncocked":
        dpg.show_item("uncocked")
        dpg.hide_item("cock")
    elif app_data == "Cock":
        dpg.hide_item("uncocked")
        dpg.show_item("cock")

def armstate(sender, app_data, user_data):
    if app_data == 1:
        dpg.show_item("firebs")
    if app_data == 0:
        dpg.hide_item("firebs")

def toggle_button(sender, app_data, user_data):
    current_label = dpg.get_item_label(sender)
    if current_label == "OFF":
        dpg.set_item_label(sender, "ON")
        dpg.bind_item_theme(sender, green_button_theme)
    else:
        dpg.set_item_label(sender, "OFF")
        dpg.bind_item_theme(sender, red_button_theme)

with dpg.theme() as red_button_theme:
    with dpg.theme_component(dpg.mvButton):
        dpg.add_theme_color(dpg.mvThemeCol_Button, [255,0,0,155])
        dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, [255,100,100,155])
        dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, [200,0,0,155])

with dpg.theme() as green_button_theme:
    with dpg.theme_component(dpg.mvButton):
        dpg.add_theme_color(dpg.mvThemeCol_Button, [0,255,0,155])
        dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, [100,255,100,155])
        dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, [0,255,0,155])

with dpg.theme() as no_header_theme:
    with dpg.theme_component(dpg.mvTable):
        dpg.add_theme_color(dpg.mvThemeCol_TableHeaderBg, [0, 0, 0, 0])


with dpg.window(tag="navigation", label="Navigation"):
    with dpg.group(horizontal=True):
        with dpg.child_window(tag="navwindow1", border=True):
            dpg.add_drag_float(tag="navforward", label="Forward", default_value=0, format="%.2f", enabled=False)
            dpg.add_drag_float(tag="navsidemove", label="Sidemove", default_value=0, format="%.2f", enabled=False)
            dpg.add_drag_int(tag="navheading", label="Heading", default_value=0, enabled=False)
        with dpg.child_window(tag="navwindow2", border=True):
            dpg.add_slider_float(tag="movementstep", label="Movement Step", default_value=0, format="%.2f", max_value=10)
            dpg.add_slider_float(tag="headingstep", label="Heading Step", default_value=0, format="%.2f", max_value=100)
            with dpg.group(horizontal=True):
                dpg.add_checkbox(tag="keycontrol", label="Enable Keyboard Controls")
                dpg.add_text("(?) - Hover Me", tag="tooltipcontrols", color=[255,0,0,155])
                with dpg.tooltip("tooltipcontrols"):
                    dpg.add_text("A tooltip")
            with dpg.group(horizontal=True):
                dpg.add_button(tag="-H", label="-H")
                dpg.add_button(tag="F", label="F")
                dpg.add_button(tag="+H", label="+H")
            with dpg.group(horizontal=True):
                dpg.add_button(tag="L", label="L")
                dpg.add_button(tag="B", label="B")
                dpg.add_button(tag="R", label="R")
    with dpg.group(horizontal=True):
        dpg.add_button(tag="startgoal", label="Start Goal")
        dpg.add_button(tag="endgoal", label="End Goal")
        dpg.add_button(tag="benchtest", label="Bench Test")
    dpg.add_button(tag="killpid", label="Kill PID")
    dpg.add_separator()
    dpg.add_text("Goal ID:")
    with dpg.table(header_row=True, borders_outerH=True, borders_innerH=True, borders_outerV=True, borders_innerV=True):
        for i in range(4):
            dpg.add_table_column(tag=f"col{i+1}")
        for row in range(5):
            with dpg.table_row():
                for col in range(5):
                    celltag = f"{row+1}{col+1}"
                    dpg.add_text(default_value= "0.00", tag=celltag)

with dpg.window(tag="readings", label="Readings"):
    with dpg.collapsing_header(label="IMU, Depth, Pressure", default_open=False):
        dpg.add_text("TBD")
    with dpg.collapsing_header(label="Odometry", default_open=False):
        dpg.add_text("Position")
        with dpg.group(horizontal=True):
            dpg.add_text("x: ")
            dpg.add_drag_float(tag="xpos", default_value=0.00, format="%.2f", enabled=False)
            dpg.add_text("y: ")
            dpg.add_drag_float(tag="ypos", default_value=0.00, format="%.2f", enabled=False)
            dpg.add_text("z: ")
            dpg.add_drag_float(tag="zpos", default_value=0.00, format="%.2f", enabled=False)
        dpg.add_text("Velocity")
        with dpg.group(horizontal=True):
            dpg.add_text("vx:")
            dpg.add_drag_float(tag="xvel", default_value=0.00, format="%.2f", enabled=False)
            dpg.add_text("vy:")
            dpg.add_drag_float(tag="yvel", default_value=0.00, format="%.2f", enabled=False)
            dpg.add_text("vz:")
            dpg.add_drag_float(tag="zvel", default_value=0.00, format="%.2f", enabled=False)
    with dpg.collapsing_header(label="GPS", default_open=False):
        dpg.add_text("TBD")
    with dpg.collapsing_header(label="Raw Sensor Data", default_open=False):
        dpg.add_text("TBD")
    with dpg.collapsing_header(label="ESC", default_open=False):
        dpg.add_text("TBD")
        dpg.add_text("TBD")
        dpg.add_text("TBD")
        dpg.add_text("TBD")
        dpg.add_text("TBD")
        dpg.add_text("TBD")
        dpg.add_text("TBD")
        dpg.add_text("TBD")

with dpg.window(tag="boatcam", label="Boat Camera"):
    pass

with dpg.window(tag="dronecam", label="Drone Camera"):
    pass

with dpg.window(tag="veh", label="Vehicle Status"):
    with dpg.collapsing_header(label="Battery Status", default_open=False):
        with dpg.table(tag="vehtable", header_row=True, resizable=False):
            dpg.bind_item_theme("vehtable", no_header_theme)
            dpg.add_table_column(tag="firstvehcol", init_width_or_weight=0.3)
            dpg.add_table_column(label="Voltage")
            dpg.add_table_column(label="Current")
            dpg.add_table_column(label="Temp")
            dpg.add_table_column(label="Capacity")
            dpg.add_table_column(tag="lastvehcol", init_width_or_weight=0.3)
            with dpg.table_row():
                dpg.add_text("B1:")
                dpg.add_drag_float(tag="b1voltage", default_value=0, format="%.2fV", enabled=False)
                dpg.add_drag_float(tag="b1current", default_value=0, format="%.2fA", enabled=False)
                dpg.add_drag_float(tag="b1temp", default_value=0, format="%.2f°C", enabled=False)
                dpg.add_progress_bar(tag="b1capacity")
                dpg.add_text(f"{battery_percentage1}%")
            with dpg.table_row():
                dpg.add_text("B2:")
                dpg.add_drag_float(tag="b2voltage", default_value=0, format="%.2fV", enabled=False)
                dpg.add_drag_float(tag="b2current", default_value=0, format="%.2fA", enabled=False)
                dpg.add_drag_float(tag="b2temp", default_value=0, format="%.2f°C", enabled=False)
                dpg.add_progress_bar(tag="b2capacity")
                dpg.add_text(f"{battery_percentage2}%")
    with dpg.collapsing_header(label="Heartbeat Status", default_open=False):
        with dpg.group(horizontal=True):
            dpg.add_text("Acoustics:")
            dpg.add_button(tag="hbacoustics", label="OFF", callback=toggle_button)
            dpg.bind_item_theme("hbacoustics", red_button_theme)
            dpg.add_text("DTLS:")
            dpg.add_button(tag="hbdtls", label="OFF", callback=toggle_button)
            dpg.bind_item_theme("hbdtls", red_button_theme)
            dpg.add_text("ESC1:")
            dpg.add_button(tag="hbesc1", label="OFF", callback=toggle_button)
            dpg.bind_item_theme("hbesc1", red_button_theme)
            dpg.add_text("ESC2:")
            dpg.add_button(tag="hbesc2", label="OFF", callback=toggle_button)
            dpg.bind_item_theme("hbesc2", red_button_theme)
            dpg.add_text("Frsky:   ")
            dpg.add_button(tag="hbfrsky", label="OFF", callback=toggle_button)
            dpg.bind_item_theme("hbfrsky", red_button_theme)
        with dpg.group(horizontal=True):
            dpg.add_text("Logic:    ")
            dpg.add_button(tag="hblogic", label="OFF", callback=toggle_button)
            dpg.bind_item_theme("hblogic", red_button_theme)
            dpg.add_text("OCS: ")
            dpg.add_button(tag="hbocs", label="OFF", callback=toggle_button)
            dpg.bind_item_theme("hbocs", red_button_theme)
            dpg.add_text("PMB1:")
            dpg.add_button(tag="hbpmb1", label="OFF", callback=toggle_button)
            dpg.bind_item_theme("hbpmb1", red_button_theme)
            dpg.add_text("PMB2:")
            dpg.add_button(tag="hbpmb2", label="OFF", callback=toggle_button)
            dpg.bind_item_theme("hbpmb2", red_button_theme)
            dpg.add_text("Thruster:")
            dpg.add_button(tag="hbthruster", label="OFF", callback=toggle_button)
            dpg.bind_item_theme("hbthruster", red_button_theme)
        with dpg.group(horizontal=True):
            dpg.add_text("POPB:     ")
            dpg.add_button(tag="hbpopb", label="OFF", callback=toggle_button)
            dpg.bind_item_theme("hbpopb", red_button_theme)
            dpg.add_text("POSB:")
            dpg.add_button(tag="hbposb", label="OFF", callback=toggle_button)
            dpg.bind_item_theme("hbposb", red_button_theme)
            dpg.add_text("POKB:")
            dpg.add_button(tag="hbpokb", label="OFF", callback=toggle_button)
            dpg.bind_item_theme("hbpokb", red_button_theme)
            dpg.add_text("SBC: ")
            dpg.add_button(tag="hbsbc", label="OFF", callback=toggle_button)
            dpg.bind_item_theme("hbsbc", red_button_theme)
            dpg.add_text("Shooter: ")
            dpg.add_button(tag="hbshooter", label="OFF", callback=toggle_button)
            dpg.bind_item_theme("hbshooter", red_button_theme)
    with dpg.collapsing_header(label="Control Link Status", default_open=False):
        dpg.add_text("TBD")
    with dpg.collapsing_header(label="Sensors Status", default_open=False):
        dpg.add_text("TBD")
    with dpg.collapsing_header(label="Emergency Stop Status", default_open=False):
        with dpg.group(horizontal=True):
            dpg.add_text("Frsky:    ")
            dpg.add_button(tag="hbfrsky2", label="OFF", callback=toggle_button)
            dpg.bind_item_theme("hbfrsky2", red_button_theme)
            dpg.add_text("Hard:")
            dpg.add_button(tag="hbhard", label="OFF", callback=toggle_button)
            dpg.bind_item_theme("hbhard", red_button_theme)
            dpg.add_text("OCS: ")
            dpg.add_button(tag="hbocs2", label="OFF", callback=toggle_button)
            dpg.bind_item_theme("hbocs2", red_button_theme)
            dpg.add_text("SBC: ")
            dpg.add_button(tag="hbsbc2", label="OFF", callback=toggle_button)
            dpg.bind_item_theme("hbsbc2", red_button_theme)
            dpg.add_text("Radio:   ")
            dpg.add_button(tag="hbradio", label="OFF", callback=toggle_button)
            dpg.bind_item_theme("hbradio", red_button_theme)
with dpg.window(tag="waypoints", label="Waypoints"):
    with dpg.plot(tag="gpswaypoints", label="GPS Coordinates"):
        x_axis = dpg.add_plot_axis(dpg.mvXAxis, label="Longitude", no_tick_marks=True, no_gridlines=True, no_tick_labels=True)
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="Latitude", no_tick_marks=True, no_gridlines=True, no_tick_labels=True)
        dpg.add_scatter_series(
            x=[coord[0] for coord in gps_reference],
            y=[coord[1] for coord in gps_reference],
            label="Position", parent=y_axis
        )

with dpg.window(tag="ballshooter", label="Ball Shooter"):
    dpg.add_combo(("Uncocked", "Cock"), default_value="Uncocked", tag="bscombo", callback=bsstate)
    with dpg.group(tag="uncocked"):
        pass
    with dpg.group(tag="cock"):
        with dpg.group(horizontal=True):
            dpg.add_checkbox(tag="arm", label="Arm", callback=armstate)
            dpg.add_button(tag="firebs", label="Fire")
            dpg.bind_item_theme("firebs", red_button_theme)
        dpg.hide_item("cock")
    dpg.hide_item("firebs")

with dpg.window(tag="thrusters", label="Thrusters"):
    with dpg.group(horizontal=True):
        dpg.add_text("Thr. 1:")
        dpg.add_drag_int(tag="thr1", default_value=1000, enabled=False)
    with dpg.group(horizontal=True):
        dpg.add_text("Thr. 2:")
        dpg.add_drag_int(tag="thr2", default_value=1000, enabled=False)
    with dpg.group(horizontal=True):
        dpg.add_text("Thr. 3:")
        dpg.add_drag_int(tag="thr3", default_value=1000, enabled=False)
    with dpg.group(horizontal=True):
        dpg.add_text("Thr. 4:")
        dpg.add_drag_int(tag="thr4", default_value=1000, enabled=False)

with dpg.window(tag="powercontrol", label="Power Control"):
    with dpg.group(horizontal=True):
        dpg.add_text("Nav:      ")
        dpg.add_button(tag="nav", label="OFF", callback=toggle_button)
        dpg.bind_item_theme("nav", red_button_theme)
    with dpg.group(horizontal=True):
        dpg.add_text("Ouster:   ")
        dpg.add_button(tag="ouster", label="OFF", callback=toggle_button)
        dpg.bind_item_theme("ouster", red_button_theme)
    with dpg.group(horizontal=True):
        dpg.add_text("Velodyne: ")
        dpg.add_button(tag="velodyne", label="OFF", callback=toggle_button)
        dpg.bind_item_theme("velodyne", red_button_theme)

with dpg.window(tag="estop", label="Emergency Stop"):
    dpg.add_button(tag="estopbutton", label="OFF", callback=toggle_button)
    dpg.bind_item_theme("estopbutton", red_button_theme)

with dpg.window(tag="task5", label="Task 5 Light Pattern"):
    t5squares()

with dpg.window(tag="task5vd", label="Task 5 Visual Display"):
    t5squaresvd()


#dpg.show_style_editor()
dpg.create_viewport(title='NTU Archimedes', width=800, height=600)
dpg.set_viewport_clear_color((173, 216, 230, 255))
dpg.setup_dearpygui()
dpg.maximize_viewport()
dpg.set_viewport_resize_callback(adjust_elements)
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()