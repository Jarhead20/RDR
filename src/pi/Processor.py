import cv2
import math
import numpy as np

class Processor():
    def __init__(self, blue_lower, blue_upper, yellow_lower, yellow_upper, M):
        self.blue_lower = blue_lower
        self.blue_upper = blue_upper
        self.yellow_lower = yellow_lower
        self.yellow_upper = yellow_upper
        self.scan_degrees = 180
        self.angle_increment = math.pi / self.scan_degrees
        self.range_min = 1.0
        self.range_max = 50.0
        self.camera_height = 3.4
        self.M = M
        # load calibration file from BirdsEyeMatrix_college_cpt.npz
        

    def filter_image(self, image, lower_bound, upper_bound):
        # Convert image to HSV color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define lower and upper bounds for the HSV filter
        lower_bound = lower_bound
        upper_bound = upper_bound

        # Apply the HSV filter
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        return mask
    
    def maskToPoints(self, mask):
        points = []

        middle = int(mask.shape[1] / 2)
        # add the middle point
        points.append((middle, mask.shape[0]-1))

        angle = 0

        increament = 180/self.scan_degrees

        j = 0

        while j < 180:
            j += increament

            # concentrate more scans in the middle of the image sinusoidally keeping the angle between 0 and 180
            add = ((math.cos(math.radians(2*j)) + 1) * increament)
            
            tx = math.cos(math.radians(angle))
            ty = math.sin(math.radians(angle))

            angle += add
            for distance in range(0, mask.shape[1]):
                y = mask.shape[0] - int(ty*distance) -1
                x = middle + int(distance * tx)
                
                if x < 0 or x >= mask.shape[1] or y < 0 or y >= mask.shape[0]:
                    # if no point is found, set it to max range
                    x = middle
                    y = mask.shape[0] - 1
                    points.append((x, y))
                    break

                if mask[y, x] == 255:                 
                    points.append((x, y))
                    break

        return np.array(points, dtype=np.float32)
        
    def warp(self, data):
        return cv2.perspectiveTransform(np.array([data], dtype=np.float32), self.M)[0]
    
    def pointsToRanges(self, points):
        ranges = []
        
        laser_angle = 0

        midpoint = points[0]
        points = points[1:]

        # determine the scale of the distance to be in meters from the calibration file
        corners = np.array([[0, 0], [0, 640], [480, 640], [480, 0]], dtype=np.float32)
        corners = self.warp(corners)
        # print(corners)
        dist1 = corners[2][1] - corners[1][1]

        # calculate the width that the camera would see when it is 2m off the ground and with a 90 degree fov
        ground_width = 2 * math.tan(math.pi / 4) * self.camera_height

        # print(ground_width)
        scale = 480/dist1
        # print("scale1 ", scale)
        scale = ground_width * scale
        # print("scale2 ", scale)

        counter = 0

        while(laser_angle < math.pi and counter < len(points)):
            x = points[counter][1]
            y = points[counter][0]

            # calculate the distance from the midpoint
            distance = math.sqrt((x - midpoint[1])**2 + (y - midpoint[0])**2)

            # scale distance
            distance = distance / scale

            # calculate the angle
            image_angle = math.atan2(y - midpoint[0], x - midpoint[1])

            counter += 1

            if laser_angle < image_angle:
                laser_angle -= self.angle_increment
                ranges.append(distance)
                continue
            
            if distance < self.range_min or distance > self.range_max:
                ranges.append(self.range_max)
                laser_angle += self.angle_increment
                continue

            if image_angle > laser_angle:
                laser_angle += self.angle_increment
                ranges.append(self.range_max)
                continue

            ranges.append(distance)
        return ranges

