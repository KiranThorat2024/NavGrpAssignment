#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# based on https://github.com/experiencor/keras-yolo3

import sys

import rospy
import roslib
import numpy as np
import cv2
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
tf.compat.v1.keras.backend.set_session(sess)

graph = tf.compat.v1.get_default_graph()
target_size = (416, 416)
model = load_model('/home/vivek/sae_ws/git_ws/bootcamp-assignments/object_recognition/models/yolo.h5', compile=False)

# define the labels for yolov3
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

class BoundBox:
  def __init__(self, xmin, ymin, xmax, ymax, objness = None, classes = None):
    self.xmin = xmin
    self.ymin = ymin
    self.xmax = xmax
    self.ymax = ymax
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
  boxes = []
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
      # first 4 elements are x, y, w, and h
      x, y, w, h = netout[int(row)][int(col)][b][:4]
      x = (col + x) / grid_w # center position, unit: image width
      y = (row + y) / grid_h # center position, unit: image height
      w = anchors[2 * b + 0] * np.exp(w) / net_w # unit: image width
      h = anchors[2 * b + 1] * np.exp(h) / net_h # unit: image height
      # last elements are class probabilities
      classes = netout[int(row)][col][b][5:]
      box = BoundBox(x-w/2, y-h/2, x+w/2, y+h/2, objectness, classes)
      boxes.append(box)
  return boxes

def correct_yolo_boxes(boxes, image_h, image_w, net_size):
  net_h = net_size[0]
  net_w = net_size[1]
  new_w, new_h = net_w, net_h
  for i in range(len(boxes)):
    x_offset, x_scale = (net_w - new_w)/2./net_w, float(new_w)/net_w
    y_offset, y_scale = (net_h - new_h)/2./net_h, float(new_h)/net_h
    boxes[i].xmin = int((boxes[i].xmin - x_offset) / x_scale * image_w)
    boxes[i].xmax = int((boxes[i].xmax - x_offset) / x_scale * image_w)
    boxes[i].ymin = int((boxes[i].ymin - y_offset) / y_scale * image_h)
    boxes[i].ymax = int((boxes[i].ymax - y_offset) / y_scale * image_h)

def _interval_overlap(interval_a, interval_b):
	x1, x2 = interval_a
	x3, x4 = interval_b
	if x3 < x1:
		if x4 < x1:
			return 0
		else:
			return min(x2,x4) - x1
	else:
		if x2 < x3:
			 return 0
		else:
			return min(x2,x4) - x3
 
def bbox_iou(box1, box2):
	intersect_w = _interval_overlap([box1.xmin, box1.xmax], [box2.xmin, box2.xmax])
	intersect_h = _interval_overlap([box1.ymin, box1.ymax], [box2.ymin, box2.ymax])
	intersect = intersect_w * intersect_h
	w1, h1 = box1.xmax-box1.xmin, box1.ymax-box1.ymin
	w2, h2 = box2.xmax-box2.xmin, box2.ymax-box2.ymin
	union = w1*h1 + w2*h2 - intersect
	return float(intersect) / union
 
def do_nms(boxes, nms_thresh):
	if len(boxes) > 0:
		nb_class = len(boxes[0].classes)
	else:
		return
	for c in range(nb_class):
		sorted_indices = np.argsort([-box.classes[c] for box in boxes])
		for i in range(len(sorted_indices)):
			index_i = sorted_indices[i]
			if boxes[index_i].classes[c] == 0: continue
			for j in range(i+1, len(sorted_indices)):
				index_j = sorted_indices[j]
				if bbox_iou(boxes[index_i], boxes[index_j]) >= nms_thresh:
					boxes[index_j].classes[c] = 0

# get all of the results above a threshold
def get_boxes(boxes, labels, thresh):
	v_boxes, v_labels, v_scores = list(), list(), list()
	# enumerate all boxes
	for box in boxes:
		# enumerate all possible labels
		for i in range(len(labels)):
			# check if the threshold for this label is high enough
			if box.classes[i] > thresh:
				v_boxes.append(box)
				v_labels.append(labels[i])
				v_scores.append(box.classes[i]*100)
				# don't break, many labels may trigger for one box
	return v_boxes, v_labels, v_scores

def draw_boxes(np_image, v_boxes, v_labels, v_scores):
  
  for i in range(len(v_boxes)):
    box = v_boxes[i]
    y1, x1, y2, x2 = box.ymin, box.xmin, box.ymax, box.xmax
    width, height = x2 - x1, y2 - y1
    label = "%s (%.3f)" % (v_labels[i], v_scores[i])
    cv_image = cv2.rectangle(np_image, (x1, y1), (x1 + width, y1 + height), (36,255,12), 1)
    cv2.putText(cv_image, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
  
  return cv_image

def callback(image_msg):
    #First convert the image to OpenCV image 
    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="rgb8")
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
    preds = model.predict(np_image)            # Classify the image
    #print([a.shape for a in preds])
    # define the anchors
    anchors = [[116,90, 156,198, 373,326], [30,61, 62,45, 59,119], [10,13, 16,30, 33,23]]
    # define the probability threshold for detected objects
    class_threshold = 0.6
    boxes = list()
    for i in range(len(preds)):
      # decode the output of the network
      boxes += decode_netout(preds[i][0], anchors[i], class_threshold, target_size)
    
    # correct the sizes of the bounding boxes for the shape of the image
    correct_yolo_boxes(boxes, height, width, target_size)
    do_nms(boxes, 0.5)
    
    
    # get the details of the detected objects
    v_boxes, v_labels, v_scores = get_boxes(boxes, labels, class_threshold)

    # summarize what we found
    for i in range(len(v_boxes)):
      print(v_labels[i], v_scores[i])
    
    try:
      max_pred_score = max(v_scores)
      max_pred_index = v_scores.index(max_pred_score)
      max_pred_label = v_labels[max_pred_index]
      box = v_boxes[max_pred_index]
      
      max_pred_box_coords = [box.ymin, box.xmin, box.ymax, box.xmax]

      pub_msg.header.stamp = rospy.Time.now()
      pub_msg.label = max_pred_label
      pub_msg.score = float(max_pred_score)
      pub_msg.box_coords = max_pred_box_coords
      pub.publish(pub_msg)

      pred_image = draw_boxes(cv_image,v_boxes,v_labels,v_scores)

      image_pub.publish(bridge.cv2_to_imgmsg(pred_image,"rgb8"))

    except ValueError:
      print("Prediction below threshold")
      pass  

rospy.init_node('classify', anonymous=True)
rospy.Subscriber("usb_cam/image_raw", Image, callback, queue_size = 1, buff_size = 16777216)

#These should be combined into a single message
pub = rospy.Publisher('object_detector', Predictor, queue_size = 1)
image_pub = rospy.Publisher('object_detector/image_raw',Image,queue_size=10)

while not rospy.is_shutdown():
  rospy.spin()
