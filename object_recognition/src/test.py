#!/usr/bin/env python3

import sys
import cv2
from std_msgs.msg import String
from std_msgs.msg import Float32
from keras.applications.resnet50 import ResNet50
from keras.preprocessing import image
from object_recognition.msg import Predictor
from keras.preprocessing.image import img_to_array
from keras.models import Sequential
from keras.applications.resnet50 import preprocess_input, decode_predictions
from sensor_msgs.msg import Image
from object_recognition.msg import Predictor
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import cv2
import numpy as np
import cv2
import rospy
import tensorflow as tf


GPU_OPTIONS = tf.compat.v1.GPUOptions(allow_growth=True)
CONFIG = tf.compat.v1.ConfigProto(gpu_options=GPU_OPTIONS)
CONFIG.gpu_options.per_process_gpu_memory_fraction = 0.5

sess = tf.compat.v1.Session(config=CONFIG)
tf.compat.v1.keras.backend.set_session(
sess
)

model = ResNet50(weights='imagenet')
graph = tf.compat.v1.get_default_graph 
target_size = (224, 224)

bridge = CvBridge()

   
def callback(image_msg):
            
    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    cv_image = cv2.resize(cv_image, target_size)  # resize image
    np_image = np.asarray(cv_image)               # read as np array
    np_image = np.expand_dims(np_image, axis=0)   # Add another dimension for tensorflow
    np_image = np_image.astype(float)             # preprocess needs float64 and img is uint8
    np_image = preprocess_input(np_image)         # Regularize the data

   
    global sess
    global graph                       # This is a workaround for asynchronous execution
    global model
    
    
    yhat = model.predict(np_image)            # Classify the image
   
    preds = list()
    for i in range(len(yhat)):
      # decode the output of the network
      preds = model.predict(np_image)           # Classify the image
    
    # get the details of the detected objects
    pred_string = decode_predictions(preds, top=1)   # Decode top 1 predictions 

    # summarize what we found
    for i in range(len(pred_string)):
      print(pred_string)    
    
    # Publish only the max values
    try:
      max_pred_score = max(pred_string)

    except ValueError:
      print("Prediction below threshold")
      pass
    
    rospy.loginfo('receiving image')
    cv2.waitKey(1)
  
if __name__ == '__main__':

    rospy.init_node('classify', anonymous=True)
    rospy.Subscriber("usb_cam/image_raw", Image, callback, queue_size = 1, buff_size = 16777216) 
while not rospy.is_shutdown():   
    rospy.spin()
    cv2.destroyAllWindows()
