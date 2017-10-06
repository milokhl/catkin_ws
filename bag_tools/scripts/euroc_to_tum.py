#!/usr/bin/env python
import os
import rosbag
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import Image, CameraInfo

def handleImage(msg, out_file, out_dir):
  """
  msg: sensor_msgs/Image
  out_file: opened text file with TUM image lines
  out_dir: directory for image files

  Writes TUM image lines to the out_file
  Adds images to out_dir, named by timestamp
  """
  pass

def handleImu(msg, out_file):
  """
  msg: sensor_msgs/Imu
  out_file: opened text file for TUM imu lines
  """
  pass

def handleGroundTruth(msg, out_file):
  """
  msg:
  - geometry_msgs/PoseStamped
  - geometry_msgs/TransformStamped

  out_file: opened text file for TUM transform line (translation and quaternion)
  """
  pass

def handleComparePose(msg, out_file):
  """
  msg:
  - geometry_msgs/PoseStamped
  - geometry_msgs/TransformStamped

  out_file: opened text file for TUM transform line (translation and quaternion)
  """
  pass

def handleCameraInfo(msg, out_file):
  """
  msg: sensor_msgs/CameraInfo
  out_file: 
  """
  pass

def poseStampedToTUM(msg):
  pass

def transformStampedToTUM(msg):
  pass

def safeOpen(file_path, options='w'):
  directory = os.path.dirname(file_path)
  try:
    os.stat(directory)
  except:
    os.makedirs(directory)
  f = open(file_path, 'w')
  return f

# TODO:
# - support for PoseStamped
# - support for Transform Stamped
def main():

  ### INPUT ARGS ###
  # TODO: set this stuff as command line arg
  rosbag_path = '/home/mknowles/Datasets/bagfiles/euroc/euroc_vicon_room/V1_01_easy.bag'
  image_topic = '/cam0/image_raw'
  cinfo_topic = 'cam0/camera_info'
  imu_topic = '/fcu/imu'
  groundtruth_topic = "/vicon/firefly_sbx/firefly_sbx"
  compare_pose_topic = "/vicon" # N/A

  ### OUTPUT ARGS ###
  output_dir = '/home/mknowles/Datasets/converted/tum_example/'

  image_output_file = 'rgb.txt'
  image_output_dir = 'rgb/'
  image_output_dir = output_dir + image_output_dir
  img_txt = safeOpen(output_dir + image_output_file)

  cinfo_output_file = 'camera_info.txt'
  cinfo_txt = safeOpen(output_dir + cinfo_output_file)

  imu_output_file = 'imu.txt'
  imu_txt = safeOpen(output_dir + imu_output_file)

  groundtruth_output_file = 'groundtruth.txt'
  gt_txt = safeOpen(output_dir + groundtruth_output_file)

  cpose_output_file = 'benchmark.txt'
  cpose_txt = safeOpen(output_dir + cpose_output_file)

  print "Opened all files."

  # TODO: add or remove topics optionally
  query_topics = [image_topic, imu_topic, groundtruth_topic, compare_pose_topic]

  bag = rosbag.Bag(rosbag_path, 'r') # open in read mode

  # step through the bag and handle each message
  prev_time = 0
  for topic, msg, t in bag.read_messages(topics=query_topics):

    # print time every 10 sec
    if t.secs-prev_time > 10:
      prev_time = t.secs
      print t.secs

    if topic == image_topic:
      handleImage(msg, img_txt, image_output_dir)

    elif topic == cinfo_topic:
      handleCameraInfo(msg, cinfo_txt)

    elif topic == imu_topic:
      handleImu(msg, imu_txt)

    elif topic == groundtruth_topic:
      handleGroundTruth(msg, gt_txt)

    elif topic == compare_pose_topic:
      handleComparePose(msg, cpose_txt)

  # close things
  img_txt.close()
  cinfo_txt.close()
  imu_txt.close()
  gt_txt.close()
  cpose_txt.close()

  bag.close()

if __name__ == "__main__":
  main()