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
json_loc = sys.argv[2]

vid_name = os.path.basename(vid_file)
vid_name = vid_name.split('.')[0]

vid = cv2.VideoCapture(vid_file)
fcount = int(vid.get(cv2.CAP_PROP_FRAME_COUNT))
json_files = sorted(glob.glob(os.path.join(json_loc, vid_name)+'*'))

if len(json_files)!=fcount:
  print ('frame count mismatch. do something!', len(json_files), fcount)
  sys.exit()

motion_ar = []
first_frame = (None, None)
frame_ar = []
pose_ar = []
conf_arr = []
for i, jf in enumerate(json_files):
  ret, frame = vid.read()
  frame_ar.append(frame)
  org_frame = frame.copy()
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

