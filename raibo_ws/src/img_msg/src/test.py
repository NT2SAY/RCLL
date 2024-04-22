import cv2
import numpy as np
image = cv2.imread(r'/home/rai/raibocup/raibo_ws/src/img_msg/src/temp.png')
cv2.imshow("", image)
cv2.waitKey(0)
cv2.destroyAllWindows()