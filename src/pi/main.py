from Processor import Processor
import cv2
import numpy as np
import serial
import os


class Main:
    def __init__(self):

        # hsv filters
        self.blue_lower = (100, 100, 100)
        self.blue_upper = (140, 255, 255)
        self.yellow_lower = (20, 100, 100)
        self.yellow_upper = (30, 255, 255)
        # print current working directory
        self.M = np.load('calibration/BirdsEyeMatrix_college_cpt.npz')['M']

        self.proc = Processor(self.blue_lower, self.blue_upper, self.yellow_lower, self.yellow_upper, self.M)
        # create serial object
        self.serial = serial.Serial('/dev/serial/by-path/pci-0000:02:00.0-usb-0:6:2.0', 115200, timeout=1)

        # if master file exists, set true 
        self.controller = os.path.exists('/home/jared/ros2_ws/src/pi/CONTROLLER')

    def run(self):
        # get image from video
        cap = cv2.VideoCapture("/home/jared/ros2_ws/src/pi/2024-07-01 16-36-50.mp4")
        print(1)
        self.serial.write(b'hello\n')
        print(2)
        self.serial.close()

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            print(self.controller)

            # resize to 640x480
            frame = cv2.resize(frame, (640, 480))

            # filter image
            mask = self.proc.filter_image(frame, self.blue_lower, self.blue_upper)



            # get points from mask
            points = self.proc.maskToPoints(mask)

            # apply perspective transform to the points
            points = self.proc.warp(points)

            # img = np.zeros((480, 640, 3), np.uint8)
            
            # # draw points
            # for point in points:
            #     cv2.circle(img, (int(point[0]), int(point[1])), 3, (0, 255, 0), -1)

            # cv2.imshow('points', img)
            # cv2.waitKey(1)
            
            ranges = self.proc.pointsToRanges(points)

            # convert ranges to ints
            # ranges_int = [int(x) for x in ranges]  # Convert floats to integers
            # ranges_str = ','.join(map(str, ranges_int))  # Convert list of integers to a comma-separated string
            # ranges_bytes = ranges_str.encode()  # Encode the string to bytes
            # self.serial.write(b'hello\n')  # Write the bytes to the serial port
            


            
        

if __name__ == "__main__":
    main = Main()
    main.run()