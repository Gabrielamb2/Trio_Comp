import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', 
                                      Image, self.encontra_centro)
    self.twist = Twist()
  def encontra_centro(self, image):
    cx = None
    cy = None
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([ 20,  50,  50])
    upper_yellow = numpy.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
    # BEGIN CROP
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = search_top + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    # END CROP
    # BEGIN FINDER
    M = cv2.moments(mask)
    if M['m00'] > 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
    # END FINDER
    # BEGIN CIRCLE
        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
    # END CIRCLE
    return [cx,cy]
