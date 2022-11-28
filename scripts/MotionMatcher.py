#import GitHub.parkour.utils.types.constants

import numpy as np
import os
import re
import bpy
import json
import random
from utils.common import *

#Combined Files Path
COMBINED_FILE_PATH = os.path.abspath('dataSet.npy')

upper_dir_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

pose_path =  upper_dir_path + '/dataset/PoseDB.json'
with open(pose_path, 'r') as f: poses = json.load(f)

feature_path = upper_dir_path + '/dataset/FeatureDB.json'
with open(feature_path, 'r') as f: features = json.load(f)


class MotionMatcher:
    motion = ''
    time = UPDATE_TIME
    matched_frame_index = -1
    prevRootLocation = [0,0,0]
    updateDiff = [0,0,0]
     
    def __init__(self):
        self.motion = 'idle'

    def start():
        print('START')

    def update():
        print('UPDATE')

    def getCurrentPose(self):
        return poses[self.matched_frame_index]['joints']

    def updateMatchedMotion(self, query, crouch, jump):
        print('Function CALL - UpdateMatchedMotion:   ',self.time, len(poses))
        obj = bpy.context.object
        bone_struct = obj.pose.bones
        joint_names = bone_struct.keys()
        
        # 매칭 프레임 찾기
        if self.time == UPDATE_TIME:
            # TODO: 매칭 알고리즘            
            self.matched_frame_index = random.randint(1,1440)
            print('---------UPDATE TIME --------- [선택된 프레임] :',self.matched_frame_index) 
        else: 
            self.matched_frame_index =  (self.matched_frame_index + 1) % len(poses)
        print('CHANGE FRAME TO => ',self.matched_frame_index )

        # 해당하는 프레임으로 애니메이션 교체 & 재생 
        for joint in joint_names:
            # 조인트 회전 정보 업데이트
            bone_struct[joint].rotation_quaternion = poses[self.matched_frame_index]['joints'][joint]['rotation']

            jointLocation = poses[self.matched_frame_index]['joints'][joint]['location']
            # 힙 조인트 위치정보 업데이트                
            if joint == 'mixamorig2:Hips': 

                if self.time == UPDATE_TIME:
                    self.updateDiff = substractArray3(self.prevRootLocation,jointLocation)
                elif np.linalg.norm(substractArray3(addArray3(jointLocation, self.updateDiff),self.prevRootLocation)) > 100:
                    print('[임시로 예외처리]:',np.linalg.norm(substractArray3(addArray3(jointLocation, self.updateDiff),self.prevRootLocation)))
                    self.updateDiff = substractArray3(self.prevRootLocation,jointLocation)
                
                # 위치 보정
                bone_struct[joint].location = addArray3(jointLocation, self.updateDiff)
                # 현재 힙 위치 저장해두기
                self.prevRootLocation = bone_struct[joint].location.copy()

            # 나머지 조인트 위치정보 업데이트
            else: bone_struct[joint].location = jointLocation

        
        # 시간 업데이트
        self.time = (self.time + 1) % (UPDATE_TIME + 1)

     