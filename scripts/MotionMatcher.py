#import GitHub.parkour.utils.types.constants

import numpy as np
import os
import re
import bpy
import json
import random

#Combined Files Path
COMBINED_FILE_PATH = os.path.abspath('dataSet.npy')
INTERVAL_TIME = 10
UPDATE_TIME = 0

pose_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + '/dataset/PoseDB.json'
with open(pose_path, 'r') as f:
    poses = json.load(f)
feature_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + '/dataset/FeatureDB.json'
with open(feature_path, 'r') as f:
    features = json.load(f)


class MotionMatcher:
    motion = ''
    time = UPDATE_TIME
    # 메인 캐릭터
    mainObj = ''
    matched_frame_index = -1
     
    def __init__(self):
        self.motion = 'idle'
        self.mainObj = ''

    def start():
        print('start')

    def update():
        print('update')

    def updateMatchedMotion(self, query, crouch, jump):
        print('Function CALL - UpdateMatchedMotion:   ',self.time, len(poses))
        # min_diff = 100000000
        obj = bpy.context.object
        bone_struct = obj.pose.bones
        joint_names = bone_struct.keys()
        
        if self.time == UPDATE_TIME:
            print('---------------UPDATE TIME ----------------')
            # TODO: 매칭 알고리즘 (임시)
            # for index, frame in enumerate(features):
            #     desired_direction_data = frame[0][0]['desired_direction']
            #     now_diff = np.linalg.norm(query - desired_direction_data)
            #     if (now_diff < min_diff):
            #         min_diff = now_diff
            #         self.matched_frame_index = index
            
            self.matched_frame_index = random.randint(1,70)
            print('선택된 프레임 = ',self.matched_frame_index) 
        
            # TODO: 입력이 멈추면 천천히 속도를 줄이도록 변경해야함
            self.time = INTERVAL_TIME
        else: 
            self.matched_frame_index =  (self.matched_frame_index + 1) % len(poses)

        print('CHANGE FRAME TO => ',self.matched_frame_index )

        for joint in joint_names:
            bone_struct[joint].rotation_quaternion = poses[self.matched_frame_index]['joints'][joint]['rotation']
            if joint != 'mixamorig:Hips':
                bone_struct[joint].location = poses[self.matched_frame_index]['joints'][joint]['location']
        
        self.time -= 1

     