#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# based on https://github.com/experiencor/keras-yolo3

import sys
import cv2

import rospy
import roslib
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from object_recognition.msg import Predictor
import tensorflow as tf
from keras.preprocessing import image
from keras.preprocessing.image import load_img
from keras.preprocessing.image import img_to_array
from keras.models import load_model

GPU_OPTIONS = tf.compat.v1.GPUOptions(allow_growth=True)
CONFIG = tf.compat.v1.ConfigProto(gpu_options=GPU_OPTIONS)
CONFIG.gpu_options.per_process_gpu_memory_fraction = 0.5

sess = tf.compat.v1.Session(config=CONFIG)
tf.compat.v1.keras.backend.set_session(
sess
)

graph = tf.compat.v1.get_default_graph()
target_size = (416, 416)

model = load_model('/home/vivek/sae_ws/git_ws/bootcamp-assignments/object_recognition/models/tinyyolo.h5', compile=False)


labels = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck",
  "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
  "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
  "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
  "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
  "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana",
  "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
  "chair", "sofa", "pottedplant", "bed", "diningtable", "toilet", "tvmonitor", "laptop", "mouse",
  "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
  "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"]
        

bridge = CvBridge()

pub_msg = Predictor()

class Predictions:
  def __init__(self, objness = None, classes = None):
    self.objness = objness
    self.classes = classes
    self.label = -1
    self.score = -1
 
  def get_label(self):
    if self.label == -1:
      self.label = np.argmax(self.classes)

    return self.label
 
  def get_score(self):
    if self.score == -1:
      self.score = self.classes[self.get_label()]

    return self.score
 
def _sigmoid(x):
  return 1. / (1. + np.exp(-x))

def decode_netout(netout, anchors, obj_thresh, net_size):
  net_h = net_size[0]
  net_w = net_size[1]
  grid_h, grid_w = netout.shape[:2]
  nb_box = 3
  netout = netout.reshape((grid_h, grid_w, nb_box, -1))
  nb_class = netout.shape[-1] - 5
  
  pred_list = []
  netout[..., :2]  = _sigmoid(netout[..., :2])
  netout[..., 4:]  = _sigmoid(netout[..., 4:])
  netout[..., 5:]  = netout[..., 4][..., np.newaxis] * netout[..., 5:]
  netout[..., 5:] *= netout[..., 5:] > obj_thresh

  for i in range(grid_h*grid_w):
    row = i / grid_w
    col = i % grid_w
    for b in range(nb_box):
      # 4th element is objectness score
      objectness = netout[int(row)][int(col)][b][4]
      if(objectness.all() <= obj_thresh): continue
      # last elements are class probabilities
      classes = netout[int(row)][col][b][5:]
      pred = Predictions(objectness, classes)
      pred_list.append(pred)
  return pred_list
 
# get all of the results above a threshold
def get_predictions(preds, labels, thresh):
  
  v_labels, v_scores = list(), list()
  
  # enumerate all the predictions
  for pred in preds:
    # enumerate all possible labels
    for i in range(len(labels)):
      # check if the threshold for this label is high enough
      if pred.classes[i] > thresh:
        v_labels.append(labels[i])
        v_scores.append(pred.classes[i]*100)
  return v_labels, v_scores

def callback(image_msg):
    #First convert the image to OpenCV image 
    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    height, width, channels = cv_image.shape
    #print(cv_image.shape)
    cv_image_target = cv2.resize(cv_image, target_size) 
    np_image = img_to_array(cv_image_target) #convert to numpy
    np_image = np_image.astype('float32')
    np_image /= 255.0
    np_image = np.expand_dims(np_image, 0)     # add a dimension so that we have one sample

   
    global sess
    global graph                                  # This is a workaround for asynchronous execution
    global model

    yhat = model.predict(np_image)            # Classify the image

    anchors = [[ 81,82,  135,169,  344,319],[10,14,  23,27,  37,58,]]

    # define the probability threshold for detected objects
    class_threshold = 0.1
    
    preds = list()
    for i in range(len(yhat)):
      # decode the output of the network
      preds += decode_netout(yhat[i][0], anchors[i], class_threshold, target_size)
        
    
    # get the details of the detected objects
    v_labels, v_scores = get_predictions(preds, labels, class_threshold)

    # summarize what we found
    for i in range(len(v_labels)):
      print(v_labels[i], v_scores[i])
    
    # Publish only the max values
    try:
      max_pred_score = max(v_scores)
      max_pred_index = v_scores.index(max_pred_score)
      max_pred_label = v_labels[max_pred_index]

      pub_msg.header.stamp = rospy.Time.now()
      pub_msg.label = max_pred_label
      pub_msg.score = float(max_pred_score)
      pub_msg.box_coords = []
      pub.publish(pub_msg)

    except ValueError:
      print("Prediction below threshold")
      pass

rospy.init_node('classify', anonymous=True)

rospy.Subscriber("usb_cam/image_raw", Image, callback, queue_size = 1, buff_size = 16777216)

pub = rospy.Publisher('object_detector', Predictor, queue_size = 10)

while not rospy.is_shutdown():
  rospy.spin()
