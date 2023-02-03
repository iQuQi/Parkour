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
from math import radians

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
    
    radian_xz = 0

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
                # point += [location[0], location[1], location[2]]
                point += [location[0]/100, location[1]/100, location[2]/100]
                
                # point += (np.array(location)*10).tolist()
                # point += (np.array(location)).tolist()

            
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
        # print('Function CALL - UpdateMatchedMotion:   ', query)
        obj = bpy.context.object

        bone_struct = obj.pose.bones
        #print('HEAD:', bone_struct['mixamorig2:LeftFoot'].tail)

        joint_names = bone_struct.keys()
        
        # 매칭 프레임 찾기
        if self.time == UPDATE_TIME:
            # 쿼리벡터 넣어주기
            distance, findIndex = self.tree.query(query)
            print('이게 포인트', features[findIndex]['trajectoryLocation'])
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
            #     if idx == (len(features)-1): break
            #     featureArray = []
            #     for location in feature['trajectoryLocation']:
            #         featureArray.append([location[0]/100, location[1]/100, location[2]/100])

            #     res = calculateDistance(featureArray, np.reshape(query, (6,3)) )
            #     if res < min:
            #         min = res
            #         minIdx = idx
            #     print(res, poses[features[idx]['poseIndex']]['animInfo'][0]['name'])

            # print('MIN => ',min, poses[features[minIdx]['poseIndex']]['animInfo'][0]['name'])
            
            # findIndex = minIdx
            # self.matched_frame_index = features[findIndex]['poseIndex']
            # self.featureIndex = findIndex

                
        else: 
            self.matched_frame_index =  (self.matched_frame_index + 1) % len(poses)


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
                    obj.location = addArray3(obj.location,obj.rotation_euler.to_matrix()@mathutils.Vector([bone_struct[joint].location[0]/100,bone_struct[joint].location[1]/100,bone_struct[joint].location[2]/100]))
                    obj.location[2] = 0
                    
                    bone_origin = mathutils.Quaternion(bone_struct[joint].rotation_quaternion).to_matrix()
                    bone_vector = bone_origin[0]+bone_origin[1]+bone_origin[2]
                    
                    joint_origin = mathutils.Quaternion(jointRotation).to_matrix()
                    joint_vector = joint_origin[0]+joint_origin[1]+joint_origin[2]

                    joint_xz = 0
                    bone_xz = 0
                    if joint_vector[0]>0:
                        joint_xz = np.arctan(joint_vector[2]/joint_vector[0])
                    elif joint_vector[0]<0:
                        joint_xz = np.arctan(joint_vector[2]/joint_vector[0]) + np.deg2rad(180)
                    if bone_vector[0]>0:
                        bone_xz = np.arctan(bone_vector[2]/bone_vector[0])
                    elif bone_vector[0]<0:
                        bone_xz = np.arctan(bone_vector[2]/bone_vector[0]) + np.deg2rad(180)

                    self.radian_xz = joint_xz - bone_xz
                    obj.rotation_euler.rotate(mathutils.Euler([0.0,0.0,-self.radian_xz],'XYZ'))

                bone_struct[joint].location = substractArray3(jointLocation, self.firstPoseLocation) # 수평방향 고정
                bone_struct[joint].rotation_quaternion = jointRotation

                for index in range(6): # 0~5까지
                    featurePoint = bpy.data.objects['Feature'+ str(index+1)]
                    if self.matched_frame_index + index*4 <= poses[self.matched_frame_index]['animInfo'][0]['end']:
                        poseLocation = poses[self.matched_frame_index + index*4]['joints'][joint]['location']
                        diff = obj.rotation_euler.to_matrix() @ mathutils.Vector(substractArray3(poseLocation, self.firstPoseLocation))
                        featurePoint.hide_viewport = False
                        featurePoint.location = [diff[0]/100 + obj.location[0], diff[1]/100 + obj.location[1], 0]
                    else:
                        featurePoint.hide_viewport = True
                
            # 나머지 조인트 위치정보 업데이트
            else: 
                bone_struct[joint].location = jointLocation            
                bone_struct[joint].rotation_quaternion = jointRotation
        
        # 시간 업데이트
        self.time = (self.time + 1) % (UPDATE_TIME + 1)

     