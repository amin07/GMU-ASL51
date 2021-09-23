"""script for visualizing openpose poses on rgb frames
   run : python draw_pose.py vid_loc pose_loc
"""
import sys
import os
import numpy as np
import cv2
import random
from random import shuffle
import matplotlib.pyplot as plt
from collections import defaultdict
import glob
import json

vid_file = sys.argv[1]
pose_loc = sys.argv[2]

vid_name = os.path.basename(vid_file)
vid_name = vid_name.split('.')[0]

vid = cv2.VideoCapture(vid_file)
fcount = int(vid.get(cv2.CAP_PROP_FRAME_COUNT))


pose_file = open(pose_loc, "r")

"""
dat_array = np.zeros((frame_count, 2, 25, 12))
lean_array = np.zeros((frame_count, 2, 2))
pdat_dict = {}
for fr in range(frame_count):
  pcount = int(f.readline().strip())
  for p in range(pcount):
    subid = int(f.readline().strip().split(' ')[0])      # person details line
    if subid not in pdat_dict:
      #print ('key not found')
      pdat_dict[subid]=np.zeros((frame_count, 1, 25, 12))
    jcount = int(f.readline().strip())
    fdat = np.array([[[float(item) for item in f.readline().strip().split(' ')] for _ in range(jcount)]]) 
    if np.isnan(fdat).any():        
      pdat_dict[subid][fr] = pdat_dict[subid][fr-1]     #if any value nan, filled with previous frame's data. what if first row has nan?
    else:
      pdat_dict[subid][fr] = fdat
"""


frame_ar = []
pose_ar = []
conf_arr = []
for i in range(fcount):
  ret, frame = vid.read()
  frame_ar.append(frame)
  org_frame = frame.copy()
  
  pose_dat = pose_file.readline().strip().split(',')
  for j in range(25):
    joint_x, joint_y = pose_dat[j].strip().split()[-4:-2]
    joint_x, joint_y = int(joint_x.strip()), int(joint_y.strip())
      
    cv2.circle(frame, (joint_x, joint_y), 8, (55,20,250), -1)

  cv2.imshow('frames', cv2.resize(np.hstack((frame, org_frame)), (0,0), fx=0.5, fy=0.5))
  k = cv2.waitKey(-1)
  if k==27:
    cv2.destroyAllWindows()
    sys.exit()
  continue
  sys.exit()
  with open(jf, 'r') as f:
    json_dat = json.load(f)['people'][0]    # top dict has keys: version, people
    pose_kps = np.split(np.array(json_dat['pose_keypoints_2d']), 25)
    lhand_kps = np.split(np.array(json_dat['hand_left_keypoints_2d']), 21)
    rhand_kps = np.split(np.array(json_dat['hand_right_keypoints_2d']), 21)
    pose_ar.append((pose_kps, lhand_kps, rhand_kps))
      
    for jp  in pose_kps:
      joint_x, joint_y = int(jp[0]), int(jp[1])
      cv2.circle(frame, (joint_x, joint_y), 6, (255,0,255), -1)
    for jp  in lhand_kps+rhand_kps:
      joint_x, joint_y = int(jp[0]), int(jp[1])
      cv2.circle(frame, (joint_x, joint_y), 4, (255,0,2), -1)
    lhand_med = sum(lhand_kps)/len(lhand_kps)
    rhand_med = sum(rhand_kps)/len(rhand_kps)
    cv2.circle(frame, (int(lhand_med[0]), int(lhand_med[1])), 6, (25,250,2), -1)
    cv2.circle(frame, (int(rhand_med[0]), int(rhand_med[1])), 6, (25,250,2), -1)
    lh_conf = sum([j[-1] for j in lhand_kps])/len(lhand_kps)
    rh_conf = sum([j[-1] for j in rhand_kps])/len(rhand_kps)
    body_conf = sum([j[-1] for j in pose_kps])/len(pose_kps)
    conf_arr.append((lh_conf, rh_conf, body_conf))
    
    cv2.imshow('frames', cv2.resize(np.hstack((frame, org_frame)), (0,0), fx=0.5, fy=0.5))
    k = cv2.waitKey(-1)
    if k==27:
      cv2.destroyAllWindows()
      sys.exit()

