# from - https://www.youtube.com/watch?v=mop6g-c5HEY
# import tkinter as tk
from tkinter import ttk
import customtkinter as ctk
from PIL import Image
import matplotlib
# from matplotlib.figure import Figure
matplotlib.use("TkAgg")
from matplotlib import style
# https://www.youtube.com/watch?v=JQ7QP5rPvjU&list=PLQVvvaa0QuDclKx-QpC9wntnURXVJqLyk&t=97s
# from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
import controller as ctr
# from tracking_testing.ArUcoDetector import ArUcoDetector
from tracking_testing.Camera import Camera
import cv2
# import numpy as np
import multiprocessing as mp
# from tracking_testing.RoundDetection_withES import FindRounds
# from tracking_testing.TrajectoryTracker import TrajectoryTracker
from tracking_testing.TuningHoughTranform_copyForIntegration import *
from positions.positions import Coordinates
from _6DOF_final.RunCalculations_Class import RobotArm



class Window(ctk.CTk):
    def __init__(self, geometry, minsize, debug = False):
        super().__init__()

        # configure the window
        self.title("Warehouse Autonomous Robotics")
        self.geometry(f'{geometry[0]}x{geometry[1]}')
        self.minsize(minsize[0], minsize[1])
        try:
            self.iconbitmap("EECS-2016_LOGO_quarter.ico")
        except:
            pass
        ctk.set_appearance_mode("dark")
        self.fullscreen = False
        # print(style.available)
        style.use("dark_background")

        # widgets
        self.menu = Menu(self)
        print(f"screen dimensions: {self.winfo_screenwidth()}, {self.winfo_screenheight()}")

        # events: https://stackoverflow.com/questions/32289175/list-of-all-tkinter-events
        self.bind('<Escape>', lambda e:self.check_exit(e))
        self.bind('<Shift-Escape>', lambda e:ctr.exit(self, e))
        self.bind('<F11>', lambda _:self.toggle_fullscreen())

        # place widgets
        self.menu.pack(fill="x", padx=5, pady=5)
        self.menu.menu_callback("Home")
        
        self.mainloop()

    def check_exit(self, event=None):
        PopupChoice(self, event)
    
    def toggle_fullscreen(self):
        self.fullscreen = not self.fullscreen
        self.attributes('-fullscreen', self.fullscreen)

class Menu(ctk.CTkSegmentedButton):
    def __init__(self, parent):
        super().__init__(parent)
        self.parent = parent
        self.string_var = ctk.StringVar(value="Home")

        # widgets
        home = HomeScreen(self.parent)
        self.frames = {
            "Home": home, 
            "View": ViewTab(self.parent, home), 
            "Settings": Settings(self.parent, home), 
            "Help": Help(self.parent)}
        self.title_font = ctk.CTkFont("Arial", 20, "bold")

        # configure buttons
        self.configure(
            values=list(self.frames.keys()),
            command=self.menu_callback,
            variable=self.string_var,
            height=50,
            corner_radius=50,
            font=self.title_font,
            fg_color=("#DBDBDB", "#2B2B2B"),
            border_width=20)
    
    def menu_callback(self, value):
        for frame in self.frames.values():
            frame.pack_forget()
        if value == "View":
            self.frames["View"].pack(fill='x', padx=5)
            self.frames["Home"].pack(expand=True, fill="both", padx=5, pady=5)
        else:
            self.frames[value].pack(expand=True, fill="both", padx=5, pady=5)

class HomeScreen(ctk.CTkFrame):
    def __init__(self, parent):
        super().__init__(parent)
        self.parent = parent

        # important variables
        self.string_var = ctk.StringVar(value='WAR GUI')
        self.torque_running = ctk.BooleanVar(value=False)
        self.video_running = ctk.BooleanVar(value=False)
        self.depth_overlay_var = ctk.BooleanVar(value=False)
        self.depth_slider_var = ctk.DoubleVar(value=0.5)
        self.aruco_overlay_var = ctk.BooleanVar(value=True)
        self.circle_overlay_var = ctk.BooleanVar(value=True)
        self.gamma_var = ctk.DoubleVar(value=0.0)
        war_image = ctk.CTkImage(Image.open("WAR.png"), size=(700, 500))
        self.destination = (ctk.DoubleVar(value=0.09), ctk.DoubleVar(value=0.45))

        self.camera_offset_y = ctk.DoubleVar(value=0.017) ##### HEEEEERRRE
        self.camera_offset_x = ctk.DoubleVar(value=0.187) ##### HEEEEERRRE
        self.camera_scale = ctk.DoubleVar(value=0.965)
        try:
            self.robot = RobotArm(self)
        except RuntimeError:
            pass

        # self.camera_offset_y_Arm2 = ctk.DoubleVar(value=-0.01)
        # self.camera_offset_x_Arm2 = ctk.DoubleVar(value=0.2)


        self.arm_speed = ctk.DoubleVar(value=0.3)
        self.grab_wait = ctk.IntVar(value=4)
        self.m2_offset = ctk.DoubleVar(value=0.33)
        self.gripper_release = ctk.DoubleVar(value=-0.5)
        self.gripper_hold = ctk.DoubleVar(value=0)
        self.coordinates = Coordinates()

        # set up grid
        self.columnconfigure(0, minsize=150, weight=1, uniform="a")
        self.columnconfigure(1, weight=15, uniform="a")
        self.columnconfigure(2, minsize=300, weight=5, uniform="a")
        self.rowconfigure((0,1), minsize=300, weight=1, uniform="a")

        # widgets
        self.button_container = ctk.CTkFrame(self, width=200)
        self.video_frame = ctk.CTkFrame(self)
        self.live_torque_graph = ctk.CTkFrame(self)
        self.simulation_graph = ctk.CTkFrame(self)
        self.destination_frame = DestinationInput(self.button_container, self.destination)
        self.war_logo = ctk.CTkLabel(self.video_frame, text="", image=war_image, anchor="center")
        self.torque_plot = ctr.TorquePlot(self.live_torque_graph, self.torque_running)
        self.exit_button = ctk.CTkButton(
            master=self.button_container, 
            text = "Exit", 
            command=self.parent.check_exit, 
        )
        self.start_button = ctk.CTkButton(
            master=self.button_container, 
            text = "Start", 
            command=lambda:ctr.start(self), 
        )
        self.start_path_button = ctk.CTkButton(
            master=self.button_container,
            text="Path",
            command=lambda:ctr.phase1(self)
        )
        self.stop_button = ctk.CTkButton(
            master=self.button_container, 
            text = "Stop", 
            command=ctr.stop, 
        )
        self.entry_field = ctk.CTkEntry(
            master=self.button_container, 
            textvariable=self.string_var
        )
        self.torque_sensing_toggle = ctk.CTkSwitch(
            master=self.button_container, 
            text="Torque Sensing", 
            command=lambda:self.torque_plot.toggle(),
            variable=self.torque_running,
            onvalue=True,
            offvalue=False
        )
        self.simulation_button = ctk.CTkButton(
            master=self.button_container, 
            text="Run Simulated Path", 
            command=lambda:ctr.plot(self.simulation_graph, lambda x:ctr.simulate_path(x))
        )
        self.text_output = ctk.CTkTextbox(
            master=self.button_container,
            text_color=('black', 'white')
        )
        self.video_feed = ctr.VideoFeed(
            self.video_frame, 
            self.video_running, 
            self.coordinates, 
            self.text_output, 
            self.depth_overlay_var, 
            self.aruco_overlay_var,
            self.circle_overlay_var,
            self.gamma_var,
            self.depth_slider_var
        )

        # events

        # pack widgets
        self.button_container.grid(row=0, column=0, rowspan=2, sticky='nsew', padx=4, pady=4)
        self.video_frame.grid(row=0, column=1, rowspan=2, sticky='nsew', padx=4, pady=4)
        self.live_torque_graph.grid(row=0, column=2, sticky='nsew', padx=4, pady=4)
        self.simulation_graph.grid(row=1, column=2, sticky='nsew', padx=4, pady=4)
        self.war_logo.place(relx=0.5, rely=0.5, anchor="center")
        self.start_button.pack(padx=5, pady=10)
        # self.start_path_button.pack(padx=5, pady=10)
        self.stop_button.pack(padx=5, pady=10)
        self.torque_sensing_toggle.pack(padx=5, pady=10)
        self.simulation_button.pack(padx=5, pady=10)
        self.destination_frame.pack(padx=5, pady=10)
        self.text_output.pack(pady=10, padx=5, fill='both')
        self.text_output.insert('0.0', 'Circle Coordinate Output\n')
        self.text_output.configure(state='disabled')

class ViewTab(ctk.CTkFrame):
    def __init__(self, parent, homescreen):
        super().__init__(parent)
        self.home = homescreen
        self.video_running = self.home.video_running
        self.depth_overlay_var = self.home.depth_overlay_var
        self.depth_slider_var = self.home.depth_slider_var
        self.aruco_overlay_var = self.home.aruco_overlay_var
        self.circle_overlay_var = self.home.circle_overlay_var
        self.gamma_var = self.home.gamma_var

        # set widget height
        self.configure(height=100)
        
        # set up grid
        self.columnconfigure(0, minsize=150, weight=1, uniform="a")
        self.columnconfigure((1,2,3,4), weight=1, uniform="a")
        self.rowconfigure((0,1), weight=1, uniform="a")

        # widgets
        self.toggle_camera = ctk.CTkSwitch(
            master=self,
            text="Camera",
            command=lambda: self.home.video_feed.toggle(self.home.war_logo),
            variable=self.video_running,
            onvalue=True,
            offvalue=False
        )
        self.toggle_depth_overlay = FormatCheckBox(
            master=self,
            variable=self.depth_overlay_var, 
            onvalue=True, 
            offvalue=False,
            textvariable=self.depth_slider_var,
            format="Depth Overlay: {0:.0%}"
        )
        self.depth_overlay_slider = ctk.CTkSlider(
            master=self,
            variable=self.depth_slider_var
        )
        self.toggle_aruco_overlay = ctk.CTkCheckBox(
            master=self,
            text="Aruco Overlay",
            variable=self.aruco_overlay_var,
            onvalue=True,
            offvalue=False
        )
        self.gamma_slider = ctk.CTkSlider(
            master=self,
            variable=self.gamma_var,
            from_=-100,
            to=100
        )
        self.gamma_label = FormatLabel(
            master=self,
            textvariable=self.gamma_var,
            format="Gamma: {:.0f}"
        )
        self.toggle_circle_overlay = ctk.CTkCheckBox(
            master=self,
            text="Circle Overlay",
            variable=self.circle_overlay_var,
            onvalue=True,
            offvalue=False
        )
        self.text_output = ctk.CTkTextbox(
            master=self,
            text_color=('black', 'white'),
            height=self.winfo_height()
        )

        # check if camera is connected
        try:
            Camera()
        except:
            print("Camera not connected")
            self.toggle_camera.configure(state="disabled")
            self.aruco_overlay_var.set(value=False)
            self.toggle_aruco_overlay.configure(state="disabled")
            self.toggle_depth_overlay.configure(state="disabled")
            self.depth_overlay_slider.configure(state="disabled")
            self.gamma_slider.configure(state="disabled")
            self.circle_overlay_var.set(value=False)
            self.toggle_circle_overlay.configure(state="disabled")

        # events
        if self.depth_overlay_slider.cget("state") != "disabled":
            self.depth_overlay_slider.bind("<MouseWheel>", lambda e: ctr.slider_scroll(e, self.depth_overlay_slider, 0.05))
        if self.gamma_slider.cget("state") != "disabled":
            self.gamma_slider.bind("<MouseWheel>", lambda e: ctr.slider_scroll(e, self.gamma_slider, 10))

        # pack widgets
        self.toggle_camera.grid(column=0, row=0, rowspan=2, padx=5, pady=5)
        self.toggle_depth_overlay.grid(column=1, row=0, padx=5, pady=5)
        self.depth_overlay_slider.grid(column=1, row=1, padx=5, pady=5)
        self.toggle_aruco_overlay.grid(column=2, row=0, padx=5, pady=5)
        self.toggle_circle_overlay.grid(column=2, row=1, padx=5, pady=5)
        self.gamma_slider.grid(column=3, row=1, padx=5, pady=5)
        self.gamma_label.grid(column=3, row=0, padx=5, pady=5)
        self.text_output.grid(column=4, row=0, rowspan=2, sticky='nsew', padx=5, pady=5)
        self.text_output.insert('0.0', 'Circle Coordinate Output\n')

class PopupChoice(ctk.CTkToplevel):
    def __init__(self, parent, event):
        super().__init__(parent)
        x_off = int(self.winfo_screenwidth()/2-self.winfo_width()/2)
        y_off = int(self.winfo_screenheight()/2-self.winfo_height()/2)
        self.geometry(f"200x100+{x_off}+{y_off}")
        self.resizable(False,False)
        self.title("")
        self.tkraise()

        # events
        self.bind('<Escape>', lambda _:self.destroy())
        
        # set up grid
        self.columnconfigure((0,1), weight=1, uniform="a")
        self.rowconfigure(0, weight=3, uniform="a")
        self.rowconfigure(1, weight=2, uniform="a")

        # widgets
        self.label = ctk.CTkLabel(self, text="Are you sure you want to exit?")
        self.exit_button = ctk.CTkButton(self, text="Exit", command=lambda:ctr.exit(parent, event))
        self.cancel_button = ctk.CTkButton(self, text="Cancel", command=self.destroy)

        # pack widgets
        self.label.grid(column=0, row=0, columnspan=2, sticky="nsew", padx=5, pady=5)
        self.exit_button.grid(column=0, row=1, sticky="nsew", padx=5, pady=5)
        self.cancel_button.grid(column=1, row=1, sticky="nsew", padx=5, pady=5)
        
        self.wait_visibility()
        self.grab_set()
        self.wait_window()

class FormatLabel(ctk.CTkLabel):
    # https://stackoverflow.com/questions/59408126/tkinter-formatting-doublevars-for-display
    def __init__(self, master=None, **kw):

        # default values
        self._format = '{}'  
        self._textvariable = None

        # get new format and remove it from `kw` so later `super().__init__` doesn't use them (it would get error message)
        if 'format' in kw:
            self._format = kw['format']
            del kw['format']

        # get `textvariable` to assign own function which set formatted text in Label when variable change value
        if 'textvariable' in kw:
            self._textvariable = kw['textvariable']
            self._textvariable.trace('w', self._update_text)
            del kw['textvariable']

        # run `Label.__init__` without `format` and `textvariable`
        super().__init__(master, **kw)

        # update text after running `Label.__init__`
        if self._textvariable:
            #self._update_text(None, None, None)
            self._update_text(self._textvariable, '', 'w')

    def _update_text(self, a, b, c):
        """update text in label when variable change value"""
        self.configure(text=self._format.format(self._textvariable.get()))

class FormatCheckBox(ctk.CTkCheckBox):
    # https://stackoverflow.com/questions/59408126/tkinter-formatting-doublevars-for-display
    def __init__(self, master=None, **kw):

        # default values
        self._format = '{}'  
        self._textvariable = None

        # get new format and remove it from `kw` so later `super().__init__` doesn't use them (it would get error message)
        if 'format' in kw:
            self._format = kw['format']
            del kw['format']

        # get `textvariable` to assign own function which set formatted text in Label when variable change value
        if 'textvariable' in kw:
            self.__textvariable = kw['textvariable']
            self.__textvariable.trace('w', self._update_text)
            del kw['textvariable']

        # run `Label.__init__` without `format` and `textvariable`
        super().__init__(master, **kw)
        # print(self._textvariable)

        # update text after running `Label.__init__`
        if self.__textvariable:
            #self._update_text(None, None, None)
            self._update_text(self.__textvariable, '', 'w')

    def _update_text(self, a, b, c):
        """update text in label when variable change value"""
        self.configure(text=self._format.format(self.__textvariable.get()))

class DestinationInput(ctk.CTkFrame):
    def __init__(self, parent, variable):
        super().__init__(parent)

        # set up grid
        self.columnconfigure(0, weight=1, uniform='a')
        self.columnconfigure(1, weight=3, uniform='a')
        self.rowconfigure((0,1,2), weight=1, uniform='a')

        # widgets
        self.title = ctk.CTkLabel(master=self, text="Destination")
        self.x = ctk.CTkLabel(master=self, text="X:")
        self.y = ctk.CTkLabel(master=self, text="Y:")
        self.x_entry = ctk.CTkEntry(master=self, textvariable=variable[0])
        self.y_entry = ctk.CTkEntry(master=self, textvariable=variable[1])

        # pack widgets
        self.title.grid(row=0, column=0, columnspan=2, padx=5, pady=5)
        self.x.grid(row=1, column=0, padx=5, pady=5)
        self.y.grid(row=2, column=0, padx=5, pady=5)
        self.x_entry.grid(row=1, column=1, padx=5, pady=5)
        self.y_entry.grid(row=2, column=1, padx=5, pady=5)
        
class Settings(ctk.CTkFrame):
    def __init__(self, parent, homescreen):
        super().__init__(parent)

        # widgets
        camera_offset_x = LabelEntry(self, "Camera X Offset:", homescreen.camera_offset_x)
        camera_offset_y = LabelEntry(self, "Camera Y Offset:", homescreen.camera_offset_y)
        arm_speed = LabelEntry(self, "Robot Arm Speed (smaller is faster)", homescreen.arm_speed)
        grab_wait = LabelEntry(self, "Wait Time (seconds) to Grab/Release Round", homescreen.grab_wait)
        m2_offset = LabelEntry(self, "M2 Motor Offset", homescreen.m2_offset)
        gripper_release = LabelEntry(self, "Gripper Release Value", homescreen.gripper_release)
        gripper_hold = LabelEntry(self, "Gripper Hold Value", homescreen.gripper_hold)
        camera_scale = LabelEntry(self, "Camera Scale", homescreen.camera_scale)

        # pack widgets
        camera_offset_x.pack(padx=5, pady=5)
        camera_offset_y.pack(padx=5, pady=5)
        camera_scale.pack(padx=5, pady=5)
        arm_speed.pack(padx=5, pady=5)
        grab_wait.pack(padx=5, pady=5)
        m2_offset.pack(padx=5, pady=5)
        gripper_release.pack(padx=5, pady=5)
        gripper_hold.pack(padx=5, pady=5)

class LabelEntry(ctk.CTkFrame):
    def __init__(self, parent, text="", variable=None):
        super().__init__(parent)

        # widgets
        label = ctk.CTkLabel(self, text=text)
        entry = ctk.CTkEntry(self, textvariable=variable)

        # pack widgets
        label.pack(side="left", padx=5, pady=5)
        entry.pack(side="left", padx=5, pady=5)

class Help(ctk.CTkScrollableFrame):
    def __init__(self, parent):
        super().__init__(parent)

