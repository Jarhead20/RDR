import cv2
import math
import numpy as np

class Processor():
    def __init__(self, blue_lower, blue_upper, yellow_lower, yellow_upper):
        self.blue_lower = blue_lower
        self.blue_upper = blue_upper
        self.yellow_lower = yellow_lower
        self.yellow_upper = yellow_upper
        self.scan_degrees = 180
        self.angle_increment = math.pi / self.scan_degrees
        self.range_min = 1.0
        self.range_max = 50.0

        # load calibration file from BirdsEyeMatrix_college_cpt.npz
        self.M = np.load('calibration/BirdsEyeMatrix_college_cpt.npz')['M']

    def filter_image(self, image, lower_bound, upper_bound):
        # Convert image to HSV color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define lower and upper bounds for the HSV filter
        lower_bound = lower_bound
        upper_bound = upper_bound

        # Apply the HSV filter
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        filtered_image = cv2.bitwise_and(image, image, mask=mask)

        return filtered_image
    
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
            for distance in range(0, self.mask.shape[1]):
                y = self.mask.shape[0] - int(ty*distance) -1
                x = middle + int(distance * tx)
                if x < 0 or x >= self.mask.shape[1] or y < 0 or y >= self.mask.shape[0]:
                    # if no point is found, set it to max range
                    x = middle
                    y = self.mask.shape[0] - 1
                    points.append((x, y))
                    break

                if self.mask[y, x] == 255:                   
                    points.append((x, y))
                    break
        
    def warp(self, data):
        return cv2.warpPerspective(data, self.M, (data.shape[1], data.shape[0]))[0]
    
    def pointsToAngles(self, points):
        ranges = []
        
        laser_angle = 0

        midpoint = points[0]
        points = points[1:]

        # determine the scale of the distance to be in meters from the calibration file
        corners = np.array([[0, 0], [0, 640], [480, 640], [480, 0]], dtype=np.float32)
        corners = self.warp(corners)

        dist1 = corners[2][1] - corners[1][1]

        # calculate the width that the camera would see when it is 2m off the ground and with a 90 degree fov
        ground_width = 2 * math.tan(math.pi / 4) * self.camera_height
        
        print(ground_width)
        scale = 480/dist1
        print("scale1 ", scale)
        scale = ground_width * scale
        print("scale2 ", scale)

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
                c1 += 1
                laser_angle -= self.angle_increment
                ranges.append(distance)
                continue
            
            if distance < self.range_min or distance > self.range_max:
                c2 += 1
                ranges.append(self.range_max)
                laser_angle += self.angle_increment
                continue

            if image_angle > laser_angle:
                c3 += 1
                laser_angle += self.angle_increment
                ranges.append(self.range_max)
                continue

            c4 += 1
            ranges.append(distance)

