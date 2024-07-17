#!/usr/bin/env python

import rclpy
import tkinter as tk
from tkinter import messagebox

from rclpy.node import Node

from geometry_msgs.msg import Twist


class DrivePublisher(Node):

    def __init__(self):
        super().__init__('move_publisher')
        self.publisher_ = self.create_publisher(Twist, 'drive', 10)

        self.root = tk.Tk()
        self.root.title("ROS 2 Button Status")

        buttons = [
            {"label": "Forward", "row": 0, "column": 1},
            {"label": "Backward", "row": 1, "column": 1},
            {"label": "Left", "row": 1, "column": 0},
            {"label": "Right", "row": 1, "column": 2},
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
        position = Twist()
        # set posx to 1 if button is pressed

        if button == 'Forward':
            position.linear.x = 0.5
        elif button == 'Backward':
            position.linear.x = -0.5
        elif button == 'Left':
            position.angular.z = 0.5
        elif button == 'Right':
            position.angular.z = -0.5


        self.get_logger().info(f'Publishing: {position}')
        self.publisher_.publish(position)


    def run(self):
        self.root.mainloop()
            
         


def main(args=None):
    rclpy.init(args=args)

    drive_pub = DrivePublisher()

    drive_pub.run()

    #color_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
