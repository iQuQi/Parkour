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

from Inertialization import Inertialization

#Combined Files Path
COMBINED_FILE_PATH = os.path.abspath('dataSet.npy')
TPOSE_NECK_ROTATION = [0.9987869262695312, 0.04907594621181488, -0.0013538144994527102, 0.0037928603123873472]

class MotionMatcher:
    motion = ''
    time = UPDATE_TIME
    matched_frame_index = 0
    firstPoseRotation = [0,0,0,0] # 초기 pose의 rotation
    firstPoseLocation = [0,0,0] # 초기 pose의 location
    firstNowLocation = [0,0,0] 
    firstNowRotation = [0,0,0,0]
    
    radian_xz = 0

    rotationDiff = [0,0,0,0]
    locationDiff = [0,0,0]
    tree = ''
    firstPoseIndex = 0

    isUpdated = False

    stopEnd = False

    inertialization = Inertialization
    inertialize = False
    inertializeHalfLife = 0.1
    inertializeDeltaTime = 1/30
     
    def __init__(self, IDLE_INDEX):
        self.matched_frame_index = IDLE_INDEX
        self.motion = 'idle'

        # 트리 생성해주기
        point_list = []
        for i in range(len(features)-1):
            point = [features[i]['hipHeight']/2.15]
            point += (np.array(features[i]['footLocation']['left'])).tolist() + (np.array(features[i]['footLocation']['right'])).tolist() # foot 추가
            point += (np.array(features[i]['footSpeed']['left'])/2).tolist() + (np.array(features[i]['footSpeed']['right'])/2).tolist() # foot 추가
            for location in features[i]['trajectoryLocation']:                
                point += (np.array(location)/15.5).tolist()
            point_list.append(point)
        

        self.tree = spatial.KDTree(point_list)


    def start():
        print('START')
       

    def __del__(self):
        print('MOTION matcher DEL')
  

    def update(): 
        print('UPDATE')

    def getCurrentPose(self):
        return poses[self.matched_frame_index]['joints']

    def updateMatchedMotion(self, query, specialIndex=-1):
        obj = bpy.data.objects['Armature']
        bone_struct = obj.pose.bones
        joint_names = bone_struct.keys()
    
        nowAnimInfo = poses[self.matched_frame_index]['animInfo'][0]
        sourcePose = poses[self.matched_frame_index] # for Inertialization

          
        # 매칭 프레임 찾기
        if self.time == UPDATE_TIME or self.matched_frame_index + 1 > nowAnimInfo['end']:
            # 쿼리벡터 넣어주기
            distance, findIndex = self.tree.query(query) # 트리에서 검색
            
            # 현재 애니메이션/포즈 정보 저장해두기
            newPoseIndex = features[findIndex]['poseIndex']     
            newAnimInfo = poses[newPoseIndex]['animInfo'][0]

            # 분기 조건 정의
            DIFF_ANIMATION = nowAnimInfo['name'] != newAnimInfo['name']
            SAME_ANIMATION = nowAnimInfo['name'] == newAnimInfo['name']
            IDX_DIFF_UNDER_20 = np.abs(newPoseIndex - nowAnimInfo['index']) < 20
            IDX_DIFF_OVER_20 = np.abs(newPoseIndex - nowAnimInfo['index']) >= 20
            WINNING = nowAnimInfo['name'] == 'Victory Idle.fbx' and specialIndex!=-1

            # 동일한 애니메이션이고 가까운(20이하) 프레임 OR 현재 세레모니 중인 경우 ====> 이어서 재생
            if  WINNING:
                if self.matched_frame_index + 1 <= nowAnimInfo['end']:
                    print('=========동일한 애니메이션 계속 실행===========',)
                    self.matched_frame_index =  (self.matched_frame_index + 1) % len(poses)
                else:
                    print('=========동일한 애니메이션 처음부터 실행===========',)
                    self.matched_frame_index = nowAnimInfo['start']
                    self.isUpdated = True
            
            # 특정 포즈를 지정해줘야 하는 경우
            elif specialIndex!=-1:
                self.isUpdated = True
                self.matched_frame_index = specialIndex

            # 다른 애니메이션이거나, 같은 애니메이션이지만 먼 프레임인 경우 ====> 교체
            else: 
                self.isUpdated = True
                self.matched_frame_index = newPoseIndex    

        # 계속 실행 ===> 프레임 1 씩 증가
        else: 
            self.matched_frame_index =  (self.matched_frame_index + 1) % len(poses)



        targetPose = poses[self.matched_frame_index] # for Inertialization
        print('애니메이션 이름 : ', poses[self.matched_frame_index]['animInfo'][0]['name'], poses[self.matched_frame_index]['animInfo'][0]['index'])
 


        # Inertialization - 블렌딩
        self.inertialization = Inertialization()
        if self.isUpdated:
            self.inertialization.poseTransition(sourcePose, targetPose)
            self.inertialization.update(targetPose, self.inertializeHalfLife, self.inertializeDeltaTime)



        # 해당하는 프레임으로 애니메이션 교체 & 재생 
        for joint in joint_names:
            # 조인트 회전 정보 업데이트
            jointRotation = mathutils.Quaternion(poses[self.matched_frame_index]['joints'][joint]['rotation'])
            jointLocation = poses[self.matched_frame_index]['joints'][joint]['location']
            # 힙 조인트 위치정보 업데이트                
            if joint == 'mixamorig2:Hips': 
                # T pose 업데이트
                if self.isUpdated:
                    # 현재 힙 정보 저장해두기
                    self.firstPoseRotation = jointRotation # 초기 pose의 rotation
                    self.firstPoseLocation = jointLocation # 초기 pose의 location

                    # T 포즈의 위치 이동 & 높이 고정
                    obj.location = addArray3(obj.location,obj.rotation_euler.to_matrix()@mathutils.Vector([bone_struct[joint].location[0]/100,bone_struct[joint].location[1]/100,bone_struct[joint].location[2]/100]))
                    obj.location[2] = 0
                    
                    # T 포즈 회전각 구하는 과정  ====> 현재 포즈와 바꿀 포즈 사이의 각도 구하기 
                    # 현재 포즈의 회전 벡터
                    bone_origin = mathutils.Quaternion(bone_struct[joint].rotation_quaternion).to_matrix()
                    bone_vector = bone_origin[:][0]+bone_origin[:][1]+bone_origin[:][2]

                    # 타겟 포즈의 회전 벡터
                    joint_origin = mathutils.Quaternion(jointRotation).to_matrix()
                    joint_vector = joint_origin[:][0]+joint_origin[:][1]+joint_origin[:][2]

                    joint_xz = 0
                    bone_xz = 0

                    # 아크 탄젠트로 각각의 각도 구해서 빼주기
                    if joint_vector[0]>0:
                        joint_xz = np.arctan(joint_vector[2]/joint_vector[0])
                    elif joint_vector[0]<0:
                        joint_xz = np.arctan(joint_vector[2]/joint_vector[0]) + np.deg2rad(180)
                    if bone_vector[0]>0:
                        bone_xz = np.arctan(bone_vector[2]/bone_vector[0])
                    elif bone_vector[0]<0:
                        bone_xz = np.arctan(bone_vector[2]/bone_vector[0]) + np.deg2rad(180)

                    self.radian_xz = joint_xz - bone_xz

                    # 구한 각도만큼 글로벌 Z축에 대해서 회전 하기
                    obj.rotation_euler.rotate(mathutils.Euler([0.0,0.0,-self.radian_xz],'XYZ'))

                # 힙은 inertialization 무시
                bone_struct[joint].location = substractArray3(jointLocation, self.firstPoseLocation) # 수평방향 고정
                bone_struct[joint].rotation_quaternion = jointRotation

                for index in range(5): # 0~4까지 피쳐 위치 출력 -> 파란색 점들
                    featurePoint = bpy.data.objects['Feature'+ str(index+1)]
                    if self.matched_frame_index + index*4 <= poses[self.matched_frame_index]['animInfo'][0]['end']:
                        poseLocation = poses[self.matched_frame_index + index*4]['joints'][joint]['location']
                        diff = obj.rotation_euler.to_matrix() @ mathutils.Vector(substractArray3(poseLocation, self.firstPoseLocation))
                        featurePoint.hide_viewport = False
                        featurePoint.location = [diff[0]/100 + obj.location[0], diff[1]/100 + obj.location[1], 0]
                    else:
                        featurePoint.hide_viewport = True # 인덱스가 벗어난 경우 해당 점은 숨긴다

            # # 목 돌아가는 막기 (고정)
            # elif joint == 'mixamorig2:Neck':   
            #     bone_struct[joint].location = jointLocation            
            #     bone_struct[joint].rotation_quaternion = TPOSE_NECK_ROTATION

            # 나머지 조인트 위치정보 업데이트
            else: 
                if self.isUpdated: # == inertialize
                    # print('기존 회전\n', jointRotation)
                    # print('블렌딩 회전\n',self.inertialization.inertializedRotations[joint])
                    jointRotation = self.inertialization.inertializedRotations[joint]
                bone_struct[joint].location = jointLocation            
                bone_struct[joint].rotation_quaternion = jointRotation

        
        # 시간 업데이트 & 변수 초기화
        self.time = (self.time + 1) % (UPDATE_TIME + 1)
        self.isUpdated = False

     