#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from object_detection.msg import ObjectDetection, BoundingBox
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile

import sys,os   
import time

sys.path.append("./utils")
import label_map_util
import visualization_utils as vis_util

# Create needed subdirs
if not os.path.exists('../models'):
  os.mkdir('../models')

if not os.path.exists('../data'):
  os.mkdir('../data')

# What model to download.
MODEL_NAME = 'ssd_mobilenet_v1_coco_11_06_2017'
MODEL_FILE = MODEL_NAME + '.tar.gz'
DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/'
PATH_TO_CKPT = '../models/' + MODEL_NAME + '/frozen_inference_graph.pb'
PATH_TO_LABELS = os.path.join('../data', 'mscoco_label_map.pbtxt')
NUM_CLASSES = 90


def download_model():
  opener = urllib.request.URLopener()
  print('Downloading model.')
  opener.retrieve(DOWNLOAD_BASE + MODEL_FILE, MODEL_FILE)
  tar_file = tarfile.open(MODEL_FILE)
  for file in tar_file.getmembers():
    file_name = os.path.basename(file.name)
    if 'frozen_inference_graph.pb' in file_name:
      tar_file.extract(file, '../models/')


# Download model checkpoint if it doesn't exist already
if not os.path.isfile(PATH_TO_CKPT):
  download_model()

if not os.path.isfile(PATH_TO_LABELS):
  raise Exception('Error: did not find a label mapping. Make sure', PATH_TO_LABELS, 'exists.')

label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)
print('Loaded label map.')


def build_detection_graph():
  """ Loads the detection graph from frozen model. """
  detection_graph = tf.Graph()
  with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
      serialized_graph = fid.read()
      od_graph_def.ParseFromString(serialized_graph)
      tf.import_graph_def(od_graph_def, name='')
  return detection_graph


class MobileNetSSD(object):
  def __init__(self):
    self.setup_complete = False

    # ROS Setup
    self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_cb, queue_size =1)
    self.image_pub = rospy.Publisher('/mnssd/debug_detections', Image, queue_size=1)
    self.detections_pub = rospy.Publisher('mnssd/detections', ObjectDetection, queue_size=1)
    self.bridge = CvBridge()

    # MobileNetSSD setup
    self.detection_graph = build_detection_graph()
    with self.detection_graph.as_default():
      self.sess = tf.Session(graph=self.detection_graph)
      self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
      self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
      self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
      self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
      self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

    print "\nMobileNetSSD setup complete!"
    self.setup_complete = True

  def image_cb(self, msg):
    # Make sure tensorflow has time to get set up
    if not self.setup_complete:
      return

    try:
      cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    except CvBridgeError as e:
      print e

    # Do detection
    boxes, scores, classes, num = self.detect(cv_image)
    
    # Visualize the results for debugging and publish.
    vis_util.visualize_boxes_and_labels_on_image_array(
      cv_image,
      np.squeeze(boxes),
      np.squeeze(classes).astype(np.int32),
      np.squeeze(scores),
      category_index,
      use_normalized_coordinates=True,
      line_thickness=8
    )
    try:
      ros_img = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
      ros_img.header.stamp = rospy.get_rostime()
      self.image_pub.publish(ros_img)
    except:
      pass

    # Publish an ObjectDetection msg
    try:
      detection_msg = ObjectDetection()
      detection_msg.header.stamp = rospy.get_rostime()
      thresh = 0.3 # only detections above this threshold are published
      limit = 5 # limits the number of detections in a msg
      for i in range(limit):
        if scores[0][i] > thresh:
          det_class = category_index[classes[0][i]]['name'] if classes[0][i] in category_index else 'unknown'
          detection_msg.classes.append(det_class)

          detection_msg.scores.append(scores[0][i])

          box = boxes[0][i]
          detection_msg.boxes.append(BoundingBox(box[0], box[1], box[2], box[3]))

      if len(detection_msg.classes) > 0:
        self.detections_pub.publish(detection_msg)
    except:
      pass

  def detect(self, frame):
    image_np = np.array(frame).astype(np.uint8)
    image_np_expanded = np.expand_dims(image_np, axis=0)

    boxes, scores, classes, num = self.sess.run(
      [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
      feed_dict={self.image_tensor: image_np_expanded}
    )
    return (boxes, scores, classes, num)

if __name__== "__main__":
  rospy.init_node('mnssd_node')
  MNSSD = MobileNetSSD()
  rospy.spin()