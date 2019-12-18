# take the blob and iterate through the entire array copy only
# cells that has at least one neighbor that is 0.

import numpy as np  # You may have to "pip install numpy" to install this
import cv2  # You may have to "pip install opencv-contrib-python" to install this
import copy
import random
import pickle
import pdb
import PIL
import math
from scipy.misc import toimage
from scipy.misc import imsave
from matplotlib import pyplot as plt
from operator import itemgetter
import intera_interface
import argparse
import rospy
import copy
import math 

from geometry_msgs.msg import Pose, Point, Quaternion

LAST_NAME = "Lenard"

g_limb = None
g_limb_end = None
g_orientation_hand_down = None
g_position_neutral = None

# robertros image
g_image_path = "/home/ub/catkin_ws/bobros.jpg"
# from homework 3
color_ranges = []

def init():
    global g_limb, g_orientation_hand_down, g_position_neutral, pos, posp, gripper
    global g_limb_end
    global marker_p, marker_q
    global square_p, square_q

    rospy.init_node('cairo_sawyer_ik_example')
    g_limb = intera_interface.Limb('right')
    gripper = intera_interface.Gripper()

    # Straight down and 'neutral' position
    g_orientation_hand_down = Quaternion()
    g_orientation_hand_down.x = 0.704238785359
    g_orientation_hand_down.y = 0.709956638597
    g_orientation_hand_down.z = -0.00229009932359
    g_orientation_hand_down.w = 0.00201493272073
    g_position_neutral = Point()
    g_position_neutral.x = 0.45371551183
    g_position_neutral.y = 0.0663097073071
    g_position_neutral.z = 0.0271459370863

    #Marker position
    marker_q = Quaternion()
    marker_q.x = 0.704238785359
    marker_q.y = 0.709956638597
    marker_q.z = -0.00229009932359
    marker_q.w = 0.00201493272073
    marker_p = Point()
    marker_p.x = 0.525423244892
    marker_p.y = 0.254786824385
    marker_p.z = 0.0125670410943

    #Square position
    square_q = Quaternion()
    square_q.x = 0.704238785359
    square_q.y = 0.709956638597
    square_q.z = -0.00229009932359
    square_q.w = 0.00201493272073
    square_p = Point()
    square_p.x = 0.53430244888
    square_p.y = -0.152176453277
    square_p.z = 0.1125670410943


# List of color ranges to look for... but stored in BGR order because that's how OpenCV does it
# Example: color_ranges = [((0,0,100), (0,0,255)), ((0,100,0), (0,255,0))]
#                         Dark Red to Bright Red    Dark Green to Bright Green

def display_image():
    global g_image_path
    head_display = intera_interface.HeadDisplay()
    head_display.display_image(g_image_path)

def add_color_range_to_detect(lower_bound, upper_bound):
  global color_ranges
  # Add color range to global list of color ranges to detect
  color_ranges.append([lower_bound, upper_bound])

def check_if_color_in_range(bgr_tuple):
  for entry in color_ranges:
    lower, upper = entry[0], entry[1]
    in_range = True
    for i in range(len(bgr_tuple)):
      if bgr_tuple[i] < lower[i] or bgr_tuple[i] > upper[i]:
        in_range = False
        break
    if in_range:
      return True
  return False

def do_color_filtering(img):
  img_height = img.shape[0]
  img_width = img.shape[1]
  # Create a matrix of dimensions [height, width] using numpy
  # Index mask as [height, width] (e.g.,: mask[y,x])
  mask = np.zeros([img_height, img_width])
  for y in range(0, img_height):
    for x in range(0, img_width):
        if check_if_color_in_range(img[y,x]):
            mask[y, x] = 1
        else:
            mask[y, x] = 0
  return mask

def get_outline(mask):
    outline = []
    for y in range(0, mask.shape[0]):
        for x in range(0, mask.shape[1]):
            if mask[y,x] == 1:
                if (mask[y-1,x] == 0 or mask[y+1,x] == 0 or mask[y,x-1] == 0 or mask[y,x+1] == 0):
                    outline.append([y,x])
    return np.array(outline)

class KMeansClassifier(object):

  def __init__(self):
    self._cluster_centers = [] # List of cluster centers, each of which is a point. ex: [ [10,10], [2,1], [0,-3] ]
    self._data = [] # List of datapoints (list of immutable lists, ex:  [ (0,0), (1.,5.), (2., 3.) ] )

  def add_datapoint(self, datapoint):
    self._data.append(datapoint)

  def fit(self, k):
    # Fit k clusters to the data, by starting with k randomly selected cluster centers.
    self._cluster_centers = [] # Reset cluster centers array

    # TODO: Initialize k cluster centers at random points
    for x in range(k):
      self._cluster_centers.append(random.choice(self._data))
    # HINT: To choose reasonable initial cluster centers, you can set them to be in the same spot as random (different) points from the dataset

    # TODO Follow convergence procedure to find final locations for each center
    while True:
      # TODO: Iterate through each datapoint in self._data and figure out which cluster it belongs to
      # HINT: Use self.classify(p) for each datapoint p
        new_cluster = []
        new_cluster_center = []
        quit = False
        for x in range(k):
            new_cluster.append([])
            
        for q in self._data:
            index = self.classify(q)
            new_cluster[index].append(q)
    
      # TODO: Figure out new positions for each cluster center (should be the average position of all its points)
        for i in range(k):
            x = 1.0
            y = 1.0
            cluster = new_cluster[i]
            for j in range(len(new_cluster[i])):
                x += cluster[j][0]
                y += cluster[j][1]
            length = len(new_cluster[i])
            if length == 0:
                length = 1
            x /= length
            y /= length
            new_cluster_center.append([x,y])
        
        dist = 0.0
        for i in range(0,k):
            dist += math.sqrt((self._cluster_centers[i][0] - new_cluster_center[i][0])**2 + (self._cluster_centers[i][1] - new_cluster_center[i][1])**2)
            if dist < 0.001:
                quit = True
                
        
        if quit == True: 
            break
        else:
            self._cluster_centers = new_cluster_center

    # TODO: Check to see how much the cluster centers have moved (for the stopping condition)
    # TODO: If the centers have moved less than some predefined threshold (you choose!) then exit the loop   

    # TODO Add each of the 'k' final cluster_centers to the model (self._cluster_centers)

  def classify(self,p):
    # Given a data point p, figure out which cluster it belongs to and return that cluster's ID (its index in self._cluster_centers)
    closest_cluster_index = 0
    smallest_distance = float('inf')

    # TODO Find nearest cluster center, then return its index in self._cluster_centers
    for r in range(len(self._cluster_centers)):
        
        dist = math.sqrt((self._cluster_centers[r][0] - p[0])**2 + (self._cluster_centers[r][1] - p[1])**2)
        if dist < smallest_distance:
            smallest_distance = dist            
            closest_cluster_index = r

    return closest_cluster_index

class KNNClassifier(object):

  def __init__(self):
    self._data = [] # list of (datapoint, label) tuples
  
  def clear_data(self):
    # Removes all data stored within the model
    self._data = []

  def add_labeled_datapoint(self, data_point, label):
    # Adds a labeled datapoint tuple onto the object's _data member
    self._data.append((data_point, label))
  
  def classify_datapoint(self, data_point, k):
    label_counts = {} # Dictionary mapping "label" => vote count
    best_label = None
    distances = []
    unique_keys = []

    # Perform k_nearest_neighbor classification, setting best_label to the majority-vote label for k-nearest points
    #TODO: Find the k nearest points in self._data to data_point
    for x in range(0,len(self._data)):
        dist = math.sqrt((data_point[0] - self._data[x][0][0])**2 + (data_point[1] - self._data[x][0][1])**2)
        distances.append(dist)
    
    for z in range(0,len(self._data)):
        key = self._data[z][1]
        if key not in unique_keys:
            unique_keys.append(key)
    
    for x in range(0,len(unique_keys)):
        key = unique_keys[x]
        label_counts[key] = 0.0
    
    for y in range(0,k):
        min_dist = min(distances)
        index = distances.index(min_dist)
        distances.pop(index)
        label_counts[self._data[index][1]] += 1/k
    #TODO: Populate label_counts with the number of votes each label got from the k nearest points
    #TODO: Make sure to scale the weight of the vote each point gets by how far away it is from data_point
    #      Since you're just taking the max at the end of the algorithm, these do not need to be normalized in any way
    best_label = max(label_counts.keys(), key=(lambda k: label_counts[k]))

    return best_label

def calculatePath(l_coords, x_scale, y_scale):
  l_coords = l_coords.tolist()
  min_dist = 10000
  min_coords = None
  dirDist = []
  inOrder = []
  index = -1
  # set first point make value
  point = l_coords[0]
  l_coords[0] = [0,0]
  for k in range(1, len(l_coords)-1):
    inOrder.append(point)
    min_dist = 10000
    for i in range(1,len(l_coords)-1):
      if l_coords[i] != point:
        dist = math.sqrt((point[1]- l_coords[i][1])**2 + (point[0]-l_coords[i][0])**2)
        if dist < min_dist:
          min_dist = dist
          min_coords = l_coords[i]
          index = i
    scaled_x = (min_coords[1]-point[1])*x_scale 
    scaled_y = (min_coords[0]-point[0])*y_scale
    s_dist = math.sqrt(scaled_x**2 + scaled_y**2)
    direction = math.atan2(scaled_y,scaled_x)
    dirDist.append([direction,s_dist])
    print(point)
    point = min_coords
    l_coords[index] = [0,0]
  return dirDist, inOrder

def draw_blob(l_vectors, fromZeroZero):
  # start_p = [inorder[0], [from 00 to minY,minX]]
    global g_limb, g_position_neutral, g_orientation_hand_down, pos, posp, gripper
    global marker_p, marker_q
    global square_p, square_q
    
    rospy.sleep(2)
    gripper.open()

    marker_pose = Pose()
    marker_pose.position = marker_p
    marker_pose.orientation = marker_q

    fancy_pose = Pose()
    fancy_pose.position = square_p
    fancy_pose.orientation = square_q

    move_to(marker_pose, 0.3, 5)
    rospy.sleep(2.0)
    gripper.close()
    rospy.sleep(2)

    marker_pose.position.z += 0.1
    move_to(marker_pose, 0.2, 5)
    rospy.sleep(1)
    fancy_pose.position.z +=0.1
    move_to(fancy_pose, 0.3, 5)

    cur_pose = Pose()
    cur_pose.orientation = fancy_pose.orientation
    cur_pose.position = fancy_pose.position

    while (True):
        # calculate new position
        cur_pose.position.x = fancy_pose.position.x + math.sin(fromZeroZero[0])*fromZeroZero[1][1]
        cur_pose.position.y = fancy_pose.position.y + math.cos(fromZeroZero[0])*fromZeroZero[1][0]
        cur_pose.position.z = cur_pose.position.z
        # move to first pose
        move_to(cur_pose, 0.15, 5)
        cur_pose.position.z -= 0.21
        move_to(cur_pose, 0.15, 5)
        for i in range (0, len(l_vectors)-1):
          cur_pose.position.x = fancy_pose.position.x + math.sin(l_vectors[i][0])*l_vectors[i][1]
          cur_pose.position.y = fancy_pose.position.y + math.cos(l_vectors[i][0])*l_vectors[i][1]
          cur_pose.position.z = cur_pose.position.z
          move_to(cur_pose, 0.15, 5)

        

def move_to(pose, speed, to):
    global g_limb
    angle = g_limb.ik_request(pose, "right_hand")
    g_limb.set_joint_position_speed(speed)
    g_limb.move_to_joint_positions(angle, timeout=to)


def main():
  global img_height, img_width
  img = cv2.imread('./color-blobs.png')
  add_color_range_to_detect([200, 0, 0], [255, 0, 0])  # Detect blue  # Detect blue
  outline = []
  img_mask = do_color_filtering(img)
  outline = get_outline(img_mask)

  # Create and test K-nearest neighbor classifier
  kMeans_classifier = KMeansClassifier()
  for datapoint in outline:
    kMeans_classifier.add_datapoint(datapoint)
  kMeans_classifier.fit(100)
  temp = kMeans_classifier._cluster_centers
  #for some reason there are a bunch of 1.0,1.0s
  temp = [item for item in temp if item[0] != 1.0]
  temp = np.array(temp)
  maxX = max(temp, key=itemgetter(1))[0]
  maxY = max(temp, key=itemgetter(1))[0]
  minX = min(temp, key=itemgetter(1))[1]
  minY = min(temp, key=itemgetter(1))[0]
  y,x = temp.T
  plt.scatter(y,x)
  plt.savefig('scatter plot')

  # base picture size will be 15 cm on the largest side
  # determine scale
  p_width = maxX - minX
  p_height = maxY - minY
  p_ratio = p_width/p_height
  d_height = 0.15 
  d_width = 0.15
  if p_width > p_height:
    p_ratio = p_height/p_width
    d_height = d_height * p_ratio
  else:
    d_width = d_width * p_ratio
  
  x_dist_scale = d_width / p_width
  y_dist_scale = d_height / p_height
  
  # pass list pick a random point
  # calculate distnace to point / calculate angle
  # add to new list of vectors
  dirDist, inOrder = calculatePath(temp, x_dist_scale, y_dist_scale)
  for i in range(0, len(inOrder)):
    print(inOrder[i])
  print("DIR DIST --------------------")
  for i in range(0, len(inOrder)):
    print(dirDist[i])
  
  direction = math.atan2((inOrder[0][1]-minY)/(inOrder[0][0]-minX))
  distance = math.sqrt(((inOrder[0][1]-minY)*y_dist_scale)**2 + ((inOrder[0][0]-minX)*x_dist_scale)**2)
  start_p = [direction, distance]
  draw_blob(l_vectors, start_p)




if __name__ == '__main__':
  main()
