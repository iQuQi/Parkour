import sys
import os
import json

sys.path.insert(1, os.path.dirname(os.path.dirname(os.path.realpath(__file__)))+'/scripts')

from MotionMatcher import MotionMatcher
import numpy as np
import bpy
from utils.common import *
import mathutils
import scipy.stats as ss 


JUMP = 'V'
CROUCH = 'C'
RUN = 'X'
ZERO_VELOCITY = 0.00000001



class QueryVector:
    def __init__(self, rootSpeed,
            RfootLocation, RfootSpeed, LfootLocation, LfootSpeed, 
            trajectoryLocation, trajectoryDirection):

        # 속도
        self.rootSpeed = rootSpeed

        # 루트의 미래+과거 궤적  ====> 과거 -> 현재 -> 미래 (4개)
        # TODO 정사영 시켜야 함
        self.trajectoryLocation = trajectoryLocation # [-10, -5, 0, 5, 10, 20]
        self.trajectoryDirection = trajectoryDirection

        self.footLocation = {'left': LfootLocation, 'right': RfootLocation}
        self.footSpeed =  {'left': LfootSpeed, 'right': RfootSpeed}

        # TODO: 손 정보 추가
    
    def toString(self):
        # print('--- 쿼리 벡터 정보 출력하기 ---')
        # print('rootSpeed:', self.rootSpeed)
        # print('trajectoryLocation:', self.trajectoryLocation)
        # print('trajectoryDirection:', self.trajectoryDirection)
        # print('footLocation:', self.footLocation)
        # print('footSpeed:', self.footSpeed)
        # print('-------------------------')
        print()


class ModalOperator(bpy.types.Operator):
    bl_idname = "object.modal_operator"
    bl_label = "Simple Modal Operator"

    KEY_MAP = {'UP_ARROW': False, 'DOWN_ARROW': False, 'LEFT_ARROW': False, 'RIGHT_ARROW': False, CROUCH: False, JUMP: False, RUN: False }

    # 지수 곡선 인자
    nowLocation =[0,0,0] 
    nowVelocity = -1
    desiredLocation = [0,0,0] 
    smoothTime = 1    # 최대 속도일 때, 목표물에 도달하는 예상 시간
    init_pose = -1
    prevVelocity = [0,-1,0] 
    
    # 모션 매쳐 객체
    motionMatcher = ''

    def __init__(self):
        print("INIT--")
        for index in range(len(poses)-1):
            if poses[index]['animInfo'][0]['name'] == 'T-Pose.fbx' and self.init_pose == -1 :
                self.init_pose = index
                self.motionMatcher = MotionMatcher(index)
                print('INIT POSE =>', index)

    
    def __del__(self):
        self.initMotion() 


    def execute(self, context):
        return {'FINISHED'}
    
    def setMove(self, type, value):
        if self.KEY_MAP.keys().__contains__(type):         
            if value == 'PRESS':
                self.KEY_MAP[type] = True
            elif value == 'RELEASE':
                self.KEY_MAP[type] = False

    def calculateStandard(self, data, mean, std):
        locationList = []
        for location in data:
            locationList.append((location-mean)/std)
        return locationList

    def initMotion(self):
        obj = bpy.context.object
        bone_struct = obj.pose.bones
        joint_names = bone_struct.keys()

        print('END INIT -> ', self.init_pose)
        if self.init_pose != -1:
            for joint in joint_names:
                jointRotation = poses[self.init_pose]['joints'][joint]['rotation']
                jointLocation = poses[self.init_pose]['joints'][joint]['location']
                bone_struct[joint].location = jointLocation            
                bone_struct[joint].rotation_quaternion = jointRotation
                obj.location = [0,0,0]
                obj.rotation_euler = DEFAULT_EULER

            for index in range(6):
                bpy.data.objects['Point'+ str(index+1)].location = [0,index/-6,0]
                bpy.data.objects['Feature'+ str(index+1)].location = [0,index/-6.1,0]

    def move(self):
        input_direction = np.array([0.0, 0.0, 0.0])

        # 여기를 local 기준으로 바꾸고, Y = -Z
        if self.KEY_MAP['UP_ARROW']:
            input_direction[Z] += GOAL 
        if self.KEY_MAP['DOWN_ARROW']:
            input_direction[Z] += (-1*GOAL) 
        if self.KEY_MAP['LEFT_ARROW']:
            input_direction[X] += GOAL
        if self.KEY_MAP['RIGHT_ARROW']:
            input_direction[X] += (-1*GOAL) 
        if self.KEY_MAP[RUN]:
            input_direction *= 2.5
        
        if np.linalg.norm(input_direction) > 0.0: 
            queryVector = self.createQueryVector(input_direction)
            self.motionMatcher.updateMatchedMotion(queryVector, self.KEY_MAP[CROUCH], self.KEY_MAP[JUMP])
        else:
            self.motionMatcher.time = UPDATE_TIME

    def createQueryVector(self, input_direction):

        # 현재 캐릭터의 정보 가져오기
        obj = bpy.context.object
        bone_struct = obj.pose.bones
        boneLocation = np.array(obj.rotation_euler.to_matrix()) @ np.array(bone_struct['mixamorig2:Hips'].location)/100
        globalLocation = [boneLocation[0]+obj.location[0],boneLocation[1]+obj.location[1],boneLocation[2]+obj.location[2]]
        axes = obj.rotation_euler.to_matrix() @ mathutils.Matrix([
            bone_struct['mixamorig2:Hips'].x_axis,
            bone_struct['mixamorig2:Hips'].y_axis,
            bone_struct['mixamorig2:Hips'].z_axis,
        ])

        # 현재 매트릭스 업데이트
        updateM(globalLocation, axes)

        # 포즈 특징 채우기
        poseStructs = self.motionMatcher.getCurrentPose()
        speed = 1
        if self.KEY_MAP[RUN]: speed *= 2
            
        rootVelocity = speed*bone_struct['mixamorig2:Hips'].z_axis
        RfootLocation = poseStructs['mixamorig2:RightFoot']['tailLocation']
        # TODO RfootVelocity= poseStructs['mixamorig2:RightFoot']['velocity'] 속도는 우선 보류
        LfootLocation = poseStructs['mixamorig2:LeftFoot']['tailLocation']
        # TODO LfootVelocity= poseStructs['mixamorig2:LeftFoot']['velocity']


        #스프링 댐퍼 변수값 초기화
        self.nowLocation = [0, 0, 0]
        self.nowVelocity = [rootVelocity[0],rootVelocity[1],rootVelocity[2]]
        self.desiredLocation =  input_direction

        #댐퍼 함수를 통해 궤적 예측 포인트 생성
        printPoint = []
        trajectoryLocation = []
        #TODO trajectoryDirection = []
        for index in range(10): # 최종적으로는 점 5개 생성
            updatePosition =  self.calculateFutureTrajectory(0.13)
            if index % 2 != 0: 
                diff = obj.rotation_euler.to_matrix() @ mathutils.Vector(updatePosition)
                printPoint.append([diff[0] + globalLocation[0], diff[1] + globalLocation[1], 0])

                trajectoryLocation.append(updatePosition)
                #TODO trajectoryDirection.extend(substractArray3(updatePosition,self.nowLocation))
                self.desiredLocation = self.nowLocation + input_direction

            self.nowLocation = updatePosition
        
        # 표준화 작업을 위해 기존 데이터의 평균, 표준편차 값을 이용하기
        normalizedTrajectory = normalizeMatrix(features[-1]['mean']['hip'], features[-1]['std']['hip'], trajectoryLocation).flatten().tolist()
        print('NORMALIZE HIP=>', features[-1]['mean']['hip'],features[-1]['std']['hip'],trajectoryLocation)
        
        normalizedLFoot = normalizeMatrix(features[-1]['mean']['Lfoot'], features[-1]['std']['Lfoot'], LfootLocation).tolist()
        normalizedRFoot = normalizeMatrix(features[-1]['mean']['Rfoot'], features[-1]['std']['Rfoot'], RfootLocation).tolist()

        # 노란색 점 위치 설정
        for index, point in enumerate(printPoint):
            bpy.data.objects['Point'+ str(index+1)].location = point

        # 쿼리 반환 전 flatten하게 합쳐주는 작업
        return normalizedLFoot +  normalizedRFoot + normalizedTrajectory 

   

    def calculateFutureTrajectory(self,timeDelta):
        omega = 2.0/self.smoothTime
        x = omega*timeDelta
        exp = 1.0/(1.0+x+0.48*x*x+0.235*x*x*x)
        change = self.nowLocation - self.desiredLocation
        temp = (self.nowVelocity+omega*change)*timeDelta
        self.nowVelocity = (self.nowVelocity-omega*temp)*exp
        return self.desiredLocation+(change+temp)*exp

    def modal(self, context, event):
        if not event.is_repeat:
            self.weight = 0.3
        if event.type == 'LEFTMOUSE':  # Confirm
            return {'FINISHED'}
        elif event.type in {'RIGHTMOUSE', 'ESC'}:  # Cancel
            context.object.location.x = self.init_loc_x
            return {'CANCELLED'}
        else:
            self.setMove(event.type, event.value) # Apply
            
        self.move()
        return {'RUNNING_MODAL'}

    def invoke(self, context, event):
        self.init_loc_x = context.object.location.x
        self.value = event.mouse_x
        self.execute(context)

        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}


# Only needed if you want to add into a dynamic menu.
def menu_func(self, context):
    self.layout.operator(ModalOperator.bl_idname, text="Modal Operator")

def invokeMotionMatcher():
    print('invokeMotionMatcher')
    
    
    
# Register and add to the object menu (required to also use F3 search "Modal Operator" for quick access).
bpy.utils.register_class(ModalOperator)
bpy.types.VIEW3D_MT_object.append(menu_func)

# test call
bpy.ops.object.modal_operator('INVOKE_DEFAULT')


