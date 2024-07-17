import cv2
import math

class Processor():
    def __init__(self, blue_lower, blue_upper, yellow_lower, yellow_upper):
        self.blue_lower = blue_lower
        self.blue_upper = blue_upper
        self.yellow_lower = yellow_lower
        self.yellow_upper = yellow_upper
        self.scan_degrees = 180


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
        
        
