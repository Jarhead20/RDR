#!/usr/bin/env python

import rclpy
import tkinter as tk
from tkinter import messagebox

from rclpy.node import Node

from unity_robotics_demo_msgs.msg import PosRot


class MovePublisher(Node):

    def __init__(self):
        super().__init__('move_publisher')
        self.publisher_ = self.create_publisher(PosRot, 'move', 10)

        self.root = tk.Tk()
        self.root.title("ROS 2 Button Status")

        buttons = [
            {"label": "+X", "row": 0, "column": 0},
            {"label": "-X", "row": 0, "column": 1},
            {"label": "+Y", "row": 1, "column": 0},
            {"label": "-Y", "row": 1, "column": 1},
            {"label": "+Z", "row": 2, "column": 0},
            {"label": "-Z", "row": 2, "column": 1},
            {"label": "+Xrot", "row": 3, "column": 0},
            {"label": "-Xrot", "row": 3, "column": 1},
            {"label": "+Yrot", "row": 4, "column": 0},
            {"label": "-Yrot", "row": 4, "column": 1},
            {"label": "+Zrot", "row": 5, "column": 0},
            {"label": "-Zrot", "row": 5, "column": 1},
        ]

        for button_info in buttons:
            label = button_info["label"]
            row = button_info["row"]
            column = button_info["column"]
            button = tk.Button(self.root, text=label, command=lambda l=label: self.button_callback(l))
            button.grid(row=row, column=column, padx=10, pady=5)

        # self.button_x = tk.Button(self.root, text="+X", command=lambda: self.button_callback("+X"))
        # self.button_x.pack()

        

        self.create_timer(0.1, self.do_publish)

    def button_callback(self, button_name):
        # messagebox.showinfo("Button Pressed", f"Button {button_name} pressed.")
        self.do_publish(button_name)

    def do_publish(self, button):
        position = PosRot()
        # set posx to 1 if button is pressed

        position.pos_x = 0.0
        position.pos_y = 0.0
        position.pos_z = 0.0
        rotX = 0.0
        rotY = 0.0
        rotZ = 0.0
    

        if button == "+X":
            position.pos_x = 1.0
        if button == "+Y":
            position.pos_y = 1.0
        if button == "+Z":
            position.pos_z = 1.0

        if button == "-X":
            position.pos_x = -1.0
        if button == "-Y":
            position.pos_y = -1.0
        if button == "-Z":
            position.pos_z = -1.0

        if button == "+Xrot":
            rotX = 1.0
        if button == "+Yrot":
            rotY = 1.0
        if button == "+Zrot":
            rotZ = 1.0
        if button == "-Xrot":
            rotX = -1.0
        if button == "-Yrot":
            rotY = -1.0
        if button == "-Zrot":
            rotZ = -1.0

        # convert euler angle to quaternion
        quat = self.euler_to_quaternion(rotX, rotY, rotZ)
        position.rot_x = quat[0]
        position.rot_y = quat[1]
        position.rot_z = quat[2]
        position.rot_w = quat[3]

        self.get_logger().info(f'Publishing: {position}')
        self.publisher_.publish(position)


    def run(self):
        self.root.mainloop()

    def euler_to_quaternion(self, roll, pitch, yaw):
        import math

        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)

        return [qx, qy, qz, qw]
            
         


def main(args=None):
    rclpy.init(args=args)

    move_pub = MovePublisher()

    move_pub.run()

    #color_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
