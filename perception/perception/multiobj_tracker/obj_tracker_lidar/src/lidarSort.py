#!/usr/bin/env python

"""
    SORT: A Simple, Online and Realtime Tracker
    Copyright (C) 2016-2020 Alex Bewley alex@bewley.ai
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""
from __future__ import print_function

import numpy as np
import cv2

import time
import argparse
from filterpy.kalman import KalmanFilter

import rospy
from obj_msgs.msg import ObjTrackBoxes
from obj_msgs.msg import ObjTrackBox
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2

import sys
import signal
def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


try:
  from numba import jit
except:
  def jit(func):
    return func
np.random.seed(0)


def linear_assignment(cost_matrix):
  try:
    import lap
    _, x, y = lap.lapjv(cost_matrix, extend_cost=True)
    return np.array([[y[i],i] for i in x if i >= 0]) #
  except ImportError:
    from scipy.optimize import linear_sum_assignment
    x, y = linear_sum_assignment(cost_matrix)
    return np.array(list(zip(x, y)))


@jit
def iou(bb_test, bb_gt):
  """
  Computes IUO between two bboxes in the form [x1,y1,x2,y2]
  """
  xx1 = np.maximum(bb_test[0], bb_gt[0])
  yy1 = np.maximum(bb_test[1], bb_gt[1])
  xx2 = np.minimum(bb_test[2], bb_gt[2])
  yy2 = np.minimum(bb_test[3], bb_gt[3])
  w = np.maximum(0., xx2 - xx1)
  h = np.maximum(0., yy2 - yy1)
  wh = w * h
  o = wh / ((bb_test[2] - bb_test[0]) * (bb_test[3] - bb_test[1])
    + (bb_gt[2] - bb_gt[0]) * (bb_gt[3] - bb_gt[1]) - wh)
  return(o)


def convert_bbox_to_z(bbox):
  """
  Takes a bounding box in the form [x1,y1,x2,y2] and returns z in the form
    [x,y,s,r] where x,y is the centre of the box and s is the scale/area and r is
    the aspect ratio
  """
  w = bbox[2] - bbox[0]
  h = bbox[3] - bbox[1]
  x = bbox[0] + w/2.
  y = bbox[1] + h/2.
  s = w * h    #scale is just area
  r = w / float(h)
  return np.array([x, y, s, r]).reshape((4, 1))


def convert_x_to_bbox(x,score=None):
  """
  Takes a bounding box in the centre form [x,y,s,r] and returns it in the form
    [x1,y1,x2,y2] where x1,y1 is the top left and x2,y2 is the bottom right
  """
  w = np.sqrt(x[2] * x[3])
  h = x[2] / w
  if(score==None):
    return np.array([x[0]-w/2.,x[1]-h/2.,x[0]+w/2.,x[1]+h/2.]).reshape((1,4))
  else:
    return np.array([x[0]-w/2.,x[1]-h/2.,x[0]+w/2.,x[1]+h/2.,score]).reshape((1,5))


class KalmanBoxTracker(object):
  """
  This class represents the internal state of individual tracked objects observed as bbox.
  """
  count = 11 #start with ID 1
  def __init__(self,bbox):
    """
    Initialises a tracker using initial bounding box.
    """
    #define constant velocity model
    self.kf = KalmanFilter(dim_x=7, dim_z=4)
    self.kf.F = np.array([[1,0,0,0,1,0,0],[0,1,0,0,0,1,0],[0,0,1,0,0,0,1],[0,0,0,1,0,0,0],  [0,0,0,0,1,0,0],[0,0,0,0,0,1,0],[0,0,0,0,0,0,1]])
    self.kf.H = np.array([[1,0,0,0,0,0,0],[0,1,0,0,0,0,0],[0,0,1,0,0,0,0],[0,0,0,1,0,0,0]])

    self.kf.Q[:4,:4] *= 10    #State Error
    self.kf.Q[4:,4:] *= 100   #State Error
    self.kf.R[:2,:2] *= 1    #Measurement
    self.kf.R[2:,2:] *= 100     #Measurement
    # self.kf.R[2:,2:] *= 10    #Measurement
    # self.kf.Q[4:,4:] *= 0.01  #State Error
    # self.kf.Q[-1,-1] *= 0.01
    # self.kf.P[:4, :4] *= 100
    self.kf.P[2:4, 2:4] *= 100
    self.kf.P[4:,4:] *= 10000 #give high uncertainty to the unobservable initial velocities
    # self.kf.P[5:,5:] *= 100. #give high uncertainty to the unobservable initial velocities
    # self.kf.P *= 10.

    self.kf.x[:4] = convert_bbox_to_z(bbox)

    self.id = KalmanBoxTracker.count
    KalmanBoxTracker.count += 1
    self.class_ = 'NEWDET'
    self.time_since_update = 0
    self.history = []
    self.hits = 0
    self.hit_streak = 0
    self.age = 0
    ##########
    self.dx_vector = []
    self.dy_vector = []

  def update(self,bbox):
    """
    Updates the state vector with observed bbox.
    """
    self.time_since_update = 0
    self.history = []
    self.hits += 1
    self.hit_streak += 1
    self.kf.update(convert_bbox_to_z(bbox))


  def predict(self):
    """
    Advances the state vector and returns the predicted bounding box estimate.
    """
    if((self.kf.x[6]+self.kf.x[2])<=0):
      self.kf.x[6] *= 0.0
    self.kf.predict()
    self.age += 1
    if(self.time_since_update>0):
      self.hit_streak = 0
    self.time_since_update += 1
    self.history.append(convert_x_to_bbox(self.kf.x))
    return self.history[-1]


  def get_state(self):
    """
    Returns the current bounding box estimate.
    """
    return convert_x_to_bbox(self.kf.x)


def associate_detections_to_trackers(detections,trackers,iou_threshold = 0.1):
  """
  Assigns detections to tracked object (both represented as bounding boxes)
  Returns 3 lists of matches, unmatched_detections and unmatched_trackers
  """
  if(len(trackers)==0):
    return np.empty((0,2),dtype=int), np.arange(len(detections)), np.empty((0,5),dtype=int)
  iou_matrix = np.zeros((len(detections),len(trackers)),dtype=np.float32)

  for d,det in enumerate(detections):
    for t,trk in enumerate(trackers):
      iou_matrix[d,t] = iou(det,trk)
  # print("<< IoU Matrix >>")
  # print(iou_matrix)
  # print("<< ========== >>")

  if min(iou_matrix.shape) > 0:
    a = (iou_matrix > iou_threshold).astype(np.int32)
    if a.sum(1).max() == 1 and a.sum(0).max() == 1:
        matched_indices = np.stack(np.where(a), axis=1)
    else:
      matched_indices = linear_assignment(-iou_matrix)
  else:
    matched_indices = np.empty(shape=(0,2))

  unmatched_detections = []
  for d, det in enumerate(detections):
    if(d not in matched_indices[:,0]):
      unmatched_detections.append(d)
  unmatched_trackers = []
  for t, trk in enumerate(trackers):
    if(t not in matched_indices[:,1]):
      unmatched_trackers.append(t)
  # print(matched_indices)
  # print("<< ========== >>")
  #filter out matched with low IOU
  matches = []
  for m in matched_indices:
    if(iou_matrix[m[0], m[1]]<iou_threshold):
      unmatched_detections.append(m[0])
      unmatched_trackers.append(m[1])
    else:
      matches.append(m.reshape(1,2))
  if(len(matches)==0):
    matches = np.empty((0,2),dtype=int)
  else:
    matches = np.concatenate(matches,axis=0)

  return matches, np.array(unmatched_detections), np.array(unmatched_trackers)


class Sort(object):
  def __init__(self, max_age=20, min_hits=1):
    print("LidarSort Initialize")
    rospy.init_node('sort', anonymous=True)
    self.subb = rospy.Subscriber('detector/detected_bboxes', ObjTrackBoxes, self.boxcallback)
    self.pubb = rospy.Publisher('tracker/tracked_bboxes', ObjTrackBoxes, queue_size=1)
    self.rate = rospy.Rate(30)
    display = rospy.get_param("/display", False)
    max_age = rospy.get_param("/max_age", max_age)
    min_hits = rospy.get_param("/min_hits", min_hits)

    """
    Sets key parameters for SORT
    """
    self.max_age = max_age
    self.min_hits = min_hits
    self.trackers = []
    self.frame_count = 0
    self.prev_time = 0.0
    self.first = True

  def boxcallback(self, msg):

    """
    GET DATA from MSG(DETECTED BOXLIST)
    """
    boxCB_time_s = time.time()
    # print("boxcallback")
    ## Get time stamp and current yaw of current frame
    curr_time = (msg.header.stamp).to_sec()
    ego_speed = msg.vehicle_speed
    ego_yaw_rate = msg.vehicle_yaw_rate
    ego_link_point = msg.ego_link_point
    global_yaw_ref_map = msg.global_yaw_ref_map
    ego_link_type = msg.ego_link_type

    if self.first:
      self.prev_time = curr_time
      self.first = False
      return

    ## Get delta time between two current frame.
    dTime_frame = curr_time - self.prev_time
    if (dTime_frame < 0) :
        dTime_frame = 0.1
    # print("delta time per frame: %d" %dTime_frame )

    ## Get data of each box( xmin, ymin, xmax, ymax, probability)
    dets = []
    for i in range(len(msg.bounding_boxes)):
        bbox = msg.bounding_boxes[i]
        dets.append(np.array([bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax, bbox.probability]))

    # print("raw_bbox in msg size: %d" %len(msg.raw_bounding_boxes) )
    rawdets  = []
    for i in range(len(msg.raw_bounding_boxes)):
        rawbox = msg.raw_bounding_boxes[i]
        rawdets.append(np.array([rawbox.xmin, rawbox.ymin, rawbox.xmax, rawbox.ymax]))

    rawdets = np.array(rawdets)
    dets = np.array(dets)

    """
    UPDATE DATA  BY APPLYING SORT ALGORITHM
    """

    trackers = self.update(dTime_frame, dets)

    """
    PUBLISH THE UPDATED BOXES
    """

    r = ObjTrackBoxes()
    r.header.stamp = msg.header.stamp
    r.header.frame_id = msg.header.frame_id
    r.vehicle_speed = ego_speed
    r.vehicle_yaw_rate = ego_yaw_rate
    r.ego_link_point = ego_link_point
    r.ego_link_type = ego_link_type
    r.global_yaw_ref_map = global_yaw_ref_map
    r.local_link_points = msg.local_link_points

    # updated bounding boxes
    if len(trackers) == 0 :
        r.bounding_boxes = []
    else :
        for d in range(len(trackers)):
            rb = ObjTrackBox()
            rb.probability=1
            rb.xmin = trackers[d,0]
            rb.ymin = trackers[d,1]
            rb.xmax = trackers[d,2]
            rb.ymax = trackers[d,3]
            rb.id = trackers[d,4]
            rb.xdelta = trackers[d,5]
            rb.ydelta = trackers[d,6]
            rb.Class = trackers[d,7]
            r.bounding_boxes.append(rb)

    # raw bounding boxes
    if len(rawdets) ==0 :
        r.raw_bounding_boxes = []
    else:
        for rd in range(len(rawdets)):
            rawrb = ObjTrackBox()
            rawrb.xmin = rawdets[rd,0]
            rawrb.ymin = rawdets[rd,1]
            rawrb.xmax = rawdets[rd,2]
            rawrb.ymax = rawdets[rd,3]
            r.raw_bounding_boxes.append(rawrb)

    # print("raw_bbox size: %d" %len(r.raw_bounding_boxes) )

    self.pubb.publish(r)
    self.prev_time = curr_time

    boxCB_time_e = time.time()
    cycle_time = boxCB_time_e - boxCB_time_s
    # print("Sort Running Time: " +str(cycle_time))

    return

  def update(self, dt, dets=np.empty((0, 5))):
    """
    Params:
      dets - a numpy array of detections in the format [[x1,y1,x2,y2,score],[x1,y1,x2,y2,score],...]
    Requires: this method must be called once for each frame even with empty detections (use np.empty((0, 5)) for frames without detections).
    Returns the a similar array, where the last column is the object ID.
    NOTE: The number of objects returned may differ from the number of detections provided.
    """

    self.frame_count += 1
    # get predicted locations from existing trackers.

    to_del = []
    ret = []

    """
    Predict states (x, y, s, r) of boxes in trackers[]
    """
    # predict the location of boxes in trackers[], and saved them in trks
    trks = np.zeros((len(self.trackers), 5))
    for t, trk in enumerate(trks):
        pos = self.trackers[t].predict()[0]
        trk[:] = [pos[0], pos[1], pos[2], pos[3], 0]
        # if nan_value exists, this idx is appended in to_del[]
        if np.any(np.isnan(pos)):
            to_del.append(t)
            ROS_WARN_STREAM("NAN value in trackers")

    trks = np.ma.compress_rows(np.ma.masked_invalid(trks))

    # erase the indices in to_del[]
    for t in reversed(to_del):
        self.trackers.pop(t)

    """
    Data Association with IOU matrix
        dets: new bboxes detected in current frame
        trks: predicted boxes from trackers
    """
    matched, unmatched_dets, unmatched_trks = associate_detections_to_trackers(dets,trks)

    """
    Update matched trackers with assigned detections
        update only matched data in trackers
    """
    for m in matched:
        self.trackers[m[1]].update(dets[m[0], :])

    # create and initialise new trackers for unmatched detections
    for i in unmatched_dets:
        trk = KalmanBoxTracker(dets[i,:])
        trk.kf.F = np.array([[1,0,0,0,dt,0,0],[0,1,0,0,0,dt,0],[0,0,1,0,0,0,1],[0,0,0,1,0,0,0],  [0,0,0,0,1,0,0],[0,0,0,0,0,1,0],[0,0,0,0,0,0,1]])
        trk.kf.Q *= dt
        trk.kf.R /= dt
        self.trackers.append(trk)
    i = len(self.trackers)

    for trk in reversed(self.trackers):

        d = trk.get_state()[0]

        trk.dx_vector.append(trk.kf.x[4])
        trk.dy_vector.append(trk.kf.x[5])
        if len(trk.dx_vector)>10:
            del trk.dx_vector[0]
            del trk.dy_vector[0]

        # adaptive moving average filter
        dx_sum = 0
        dy_sum = 0
        cost_sum = 0
        cost = 0
        for idx in range(len(trk.dx_vector)):
            cost += 1
            dx_sum += cost*trk.dx_vector[idx]
            dy_sum += cost*trk.dy_vector[idx]
            cost_sum += cost
            # print("idx : %d" %idx, " dx : %d" %trk.dx_vector[idx], " dy : %d" %trk.dy_vector[idx])

        if(cost_sum != 0):
            dx_avg = dx_sum/cost_sum
            dy_avg = dy_sum/cost_sum
        else :
            dx_avg = 0
            dy_avg = 0

        # moving average filter
        # dx_avg = np.sum(np.array(trk.dx_vector))/len(trk.dx_vector)
        # dy_avg = np.sum(np.array(trk.dy_vector))/len(trk.dy_vector)

        # trk.kf.x[4] = dx_avg
        # trk.kf.x[5] = dy_avg

        if (trk.time_since_update < 1) and (trk.hit_streak >= self.min_hits or self.frame_count <= self.min_hits):
            trk.class_ = 'TRACKED'
            if trk.hits <= self.min_hits:
                trk.kf.x[4] = 0
                trk.kf.x[5] = 0
            ret.append(np.concatenate((d,[trk.id, dx_avg, dy_avg, trk.class_])).reshape(1,-1)) # +1 as MOT benchmark requires positive
            # ret.append(np.concatenate((d,[trk.id, trk.kf.x[4], trk.kf.x[5], trk.class_])).reshape(1,-1)) # +1 as MOT benchmark requires positive
            # print("tracked ID: %d" %trk.id, d, trk.kf.x[4], trk.kf.x[5])

        # elif (trk.class_ == 'NEWDET'):
        #     ret.append(np.concatenate((d,[trk.id, trk.kf.x[4], trk.kf.x[5], trk.class_])).reshape(1,-1)) # +1 as MOT benchmark requires positive
        # # elif (trk.time_since_update >= 1):
        #     trk.class_ = 'UNMATCHED'
        #     # print("unmatched ID: %d" %trk.id)
        #     ret.append(np.concatenate((d,[trk.id, trk.kf.x[4], trk.kf.x[5], trk.class_])).reshape(1,-1)) # +1 as MOT benchmark requires positive

        i -= 1
        # remove dead tracklet
        if(trk.time_since_update > self.max_age):
            self.trackers.pop(i)

    if(len(ret)>0):
        return np.concatenate(ret)

    return np.empty((0,5))





if __name__ == '__main__':

    mot_tracker = Sort(max_age=10, min_hits=1) #create instance of the SORT tracker

    while True:
        try:
            mot_tracker.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt):
            sys.exit(0)
#        except:
#            pass
