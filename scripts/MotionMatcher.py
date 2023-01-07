#import GitHub.parkour.utils.types.constants

import numpy as np
import os
import re
import bpy
import json
import random
from utils.common import *
from scipy import spatial 
import mathutils

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
    firstPoseRotation = [0,0,0,0] # 초기 pose의 rotation
    firstPoseLocation = [0,0,0] # 초기 pose의 location
    firstNowLocation = [0,0,0] 
    firstNowRotation = [0,0,0,0]

    rotationDiff = [0,0,0,0]
    locationDiff = [0,0,0]
    tree = ''
    featureIndex = 0
     
    def __init__(self):
        self.motion = 'idle'
        # 트리 생성해주기
        point_list = []
        for feature in features:
            #point = feature['rootSpeed'].copy()
            point = []
            # point = feature['footLocation']['left'] + feature['footLocation']['right'] # foot 추가

            for location in feature['trajectoryLocation']:
                point += ([location[0]/100, location[1]/100, location[2]/100])

            # for direction in feature['trajectoryDirection']:
            #     point += direction


            point_list.append(point)
        
           
        # [([speed], [], [], []), (speed, [location]), ()]
        print('point_list : ', point_list)
        self.tree = spatial.KDTree(point_list)


    def start():
        print('START')

    def update():
        print('UPDATE')

    def getCurrentPose(self):
        return poses[self.matched_frame_index]['joints']

    def updateMatchedMotion(self, query, crouch, jump):
        print('Function CALL - UpdateMatchedMotion:   ', query)
        obj = bpy.context.object
        bone_struct = obj.pose.bones
        joint_names = bone_struct.keys()
        
        # 매칭 프레임 찾기
        if self.time == UPDATE_TIME:
            # 쿼리벡터 넣어주기
            distance, findIndex = self.tree.query(query)
            # print('쿼리...',query)

            # self.matched_frame_index = random.randint(1,728)
            # findIndex = 10
            self.matched_frame_index = features[findIndex]['poseIndex']
            self.featureIndex = findIndex

            # bruteforce
            # min = 2147483647
            # minIdx = -1
            # for idx, feature in enumerate(features):
            #     res = calculateDistance(feature['trajectoryLocation'], query)
            #     if res < min:
            #         min = res
            #         minIdx = idx
            #     print(res, poses[features[idx]['poseIndex']]['animInfo'][0]['name'])
            
            # findIndex = minIdx
            # self.matched_frame_index = features[findIndex]['poseIndex']
            # self.featureIndex = findIndex

                
        else: 
            self.matched_frame_index =  (self.matched_frame_index + 1) % len(poses)

        #  피쳐의 궤적 출력
        print('매칭된 인덱스', self.matched_frame_index)
        for index, point in enumerate(features[self.featureIndex]['trajectoryLocation']):
            bpy.data.objects['Feature'+ str(index+1)].location = local2global([point[0]/100,point[1]/100,point[2]/100])

        print('애니메이션 이름 : ', poses[self.matched_frame_index]['animInfo'][0]['name'])
        # 해당하는 프레임으로 애니메이션 교체 & 재생 
        for joint in joint_names:
            # 조인트 회전 정보 업데이트
            jointRotation = mathutils.Quaternion(poses[self.matched_frame_index]['joints'][joint]['rotation'])
            jointLocation = poses[self.matched_frame_index]['joints'][joint]['location']
            # 힙 조인트 위치정보 업데이트                
            if joint == 'mixamorig2:Hips': 
                # exceptionCheck = np.linalg.norm(substractArray3(addArray3(jointLocation, self.locationDiff),self.prevRootLocation)) > 100
                if self.time == UPDATE_TIME: # update time 20까지는 괜찮음
                    self.firstPoseRotation = jointRotation # 초기 pose의 rotation
                    self.firstPoseLocation = jointLocation # 초기 pose의 location
                    # 현재 힙 정보 저장해두기
                    self.firstNowLocation = bone_struct[joint].location.copy() # 초기 현재의 location
                    self.firstNowRotation = bone_struct[joint].rotation_quaternion.copy() # 초기 현재의 rotation
                    
                self.locationDiff = self.firstNowRotation.to_matrix()@self.firstPoseRotation.to_matrix().inverted_safe()@mathutils.Vector((substractArray3(jointLocation, self.firstPoseLocation)))
                # 보정
                bone_struct[joint].location = addArray3(self.firstNowLocation, self.locationDiff)
                
                # bone_struct[joint].rotation_quaternion = jointRotation 
                if np.isnan(self.firstPoseRotation.w):
                    bone_struct[joint].rotation_quaternion = jointRotation 
                else:
                    print('회전 확인', (self.firstPoseRotation).to_euler())
                    bone_struct[joint].rotation_quaternion = (self.firstNowRotation.to_matrix()@self.firstPoseRotation.to_matrix().inverted_safe()@jointRotation.to_matrix()).to_quaternion()
                # bone_struct[joint].rotation_quaternion.rotate(self.rotationDiff)

                
                
                
            # 나머지 조인트 위치정보 업데이트
            else: 
                bone_struct[joint].location = jointLocation            
                bone_struct[joint].rotation_quaternion = jointRotation



        # bpy.data.objects['Cone'].location = [bone_struct['mixamorig2:Hips'].matrix[0][0],(-1)*bone_struct['mixamorig2:Hips'].matrix[2][0],bone_struct['mixamorig2:Hips'].matrix[1][0]]
        # bpy.data.objects['Cylinder'].location =  [bone_struct['mixamorig2:Hips'].matrix[0][1],(-1)*bone_struct['mixamorig2:Hips'].matrix[2][1],bone_struct['mixamorig2:Hips'].matrix[1][1]]
        # bpy.data.objects['Cube'].location =  [bone_struct['mixamorig2:Hips'].matrix[0][2],(-1)*bone_struct['mixamorig2:Hips'].matrix[2][2],bone_struct['mixamorig2:Hips'].matrix[1][2]]



        # 궤도 특징 채우기 => 지수 함수 생성으로 예측
        
        # 시간 업데이트
        self.time = (self.time + 1) % (UPDATE_TIME + 1)

     