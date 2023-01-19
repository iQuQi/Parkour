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
        for i in range(len(features)-1):
            #point = feature['rootSpeed'].copy()
            point = []
            # point = features[i]['footLocation']['left'] + features[i]['footLocation']['right'] # foot 추가

            for location in features[i]['trajectoryLocation']:
                # point += [location[0]/100, location[1]/100, location[2]/100]
                point += [location[0], location[1], location[2]]

            print('이건 포인트', point)
            point_list.append(point)
        
           
        # [([speed], [], [], []), (speed, [location]), ()]
        # print('point_list : ', point_list)
        self.tree = spatial.KDTree(point_list)


    def start():
        print('START')
       

    def __del__(self):
        print('MOTION matcher DEL')
  

    def update(): 
        print('UPDATE')

    def getCurrentPose(self):
        return poses[self.matched_frame_index]['joints']

    def getCurrentFeature(self):
        return features[self.featureIndex]

    def updateMatchedMotion(self, query, crouch, jump):
        print('Function CALL - UpdateMatchedMotion:   ', query)
        obj = bpy.context.object
        bone_struct = obj.pose.bones
        #print('HEAD:', bone_struct['mixamorig2:LeftFoot'].tail)

        joint_names = bone_struct.keys()
        
        # 매칭 프레임 찾기
        if self.time == UPDATE_TIME:
            # 쿼리벡터 넣어주기
            distance, findIndex = self.tree.query(query)
            # print('쿼리...',query)

            # self.matched_frame_index = random.randint(1,728)

            # 이름이 같을 때
            # if poses[self.matched_frame_index]['animInfo'][0]['index']<poses[self.matched_frame_index]['animInfo'][0]['end']-1 and poses[features[findIndex]['poseIndex']]['animInfo'][0]['name']==poses[self.matched_frame_index]['animInfo'][0]['name']:
            #     self.matched_frame_index = self.matched_frame_index+1
                
            # else:
            self.matched_frame_index = features[findIndex]['poseIndex']
            self.featureIndex = findIndex

            # # bruteforce
            # min = 2147483647
            # minIdx = -1
            # for idx, feature in enumerate(features):
            #     featureArray = feature['footLocation']['left'] + feature['footLocation']['right'] # foot 추가
            #     for location in feature['trajectoryLocation']:
            #         featureArray += ([location[0]/100, location[1]/100, location[2]/100])

            #     res = calculateDistance(featureArray, query)
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
            bpy.data.objects['Feature'+ str(index+1)].location = local2global([point[0],point[1],point[2]])

        print('애니메이션 이름 : ', poses[self.matched_frame_index]['animInfo'][0]['name'])
        # 해당하는 프레임으로 애니메이션 교체 & 재생 
        for joint in joint_names:
            # 조인트 회전 정보 업데이트
            jointRotation = mathutils.Quaternion(poses[self.matched_frame_index]['joints'][joint]['rotation'])
            jointLocation = poses[self.matched_frame_index]['joints'][joint]['location']
            # 힙 조인트 위치정보 업데이트                
            if joint == 'mixamorig2:Hips': 
                if self.time == UPDATE_TIME: # update time 20까지는 괜찮음
                    self.firstPoseRotation = jointRotation # 초기 pose의 rotation
                    self.firstPoseLocation = jointLocation # 초기 pose의 location
                    # 현재 힙 정보 저장해두기
                    self.firstNowLocation = bone_struct[joint].location.copy() # 초기 현재의 location
                    self.firstNowRotation = bone_struct[joint].rotation_quaternion.copy() # 초기 현재의 rotation
                    
                self.locationDiff = self.firstNowRotation.to_matrix()@self.firstPoseRotation.to_matrix().inverted_safe()@mathutils.Vector((substractArray3(jointLocation, self.firstPoseLocation)))
                # 보정
                bone_struct[joint].location = addArray3(self.firstNowLocation, self.locationDiff)
                # bone_struct[joint].location[0] = self.firstPoseLocation[0]
                bone_struct[joint].location[1] = self.firstPoseLocation[1]

                mat = self.firstNowRotation.to_matrix()@self.firstPoseRotation.to_matrix().inverted_safe()@jointRotation.to_matrix()
                now_mat = self.firstPoseRotation.to_matrix()
                for i in range(3):
                    mat[i][1]=now_mat[i][1]
                    # mat[i][0]=now_mat[i][0]
                    # mat[i][2]=now_mat[i][2]
    
                bone_struct[joint].rotation_quaternion = mat.to_quaternion()
                
            # 나머지 조인트 위치정보 업데이트
            else: 
                bone_struct[joint].location = jointLocation            
                bone_struct[joint].rotation_quaternion = jointRotation
        
        # 시간 업데이트
        self.time = (self.time + 1) % (UPDATE_TIME + 1)

     