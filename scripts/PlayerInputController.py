import sys
import os
import json

sys.path.insert(1, os.path.dirname(os.path.dirname(os.path.realpath(__file__)))+'/scripts')

from MotionMatcher import MotionMatcher
import numpy as np
import bpy
from utils.common import *
import mathutils

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
        self.trajectoryLocation = trajectoryLocation # [-10, -5, 0, 5, 10, 20]
        self.trajectoryDirection = trajectoryDirection

        self.footLocation = {'left': LfootLocation, 'right': RfootLocation}
        self.footSpeed =  {'left': LfootSpeed, 'right': RfootSpeed}

        # TODO: 손 정보 추가



class ModalOperator(bpy.types.Operator):
    bl_idname = "object.modal_operator"
    bl_label = "Simple Modal Operator"

    KEY_MAP = {'UP_ARROW': False, 'DOWN_ARROW': False, 'LEFT_ARROW': False, 'RIGHT_ARROW': False, CROUCH: False, JUMP: False, RUN: False }

    # 지수 곡선 인자
    nowLocation =[0,0,0] 
    nowVelocity = -1
    desiredLocation = [0,0,0] 
    smoothTime = 1    # 최대 속도일 때, 목표물에 도달하는 예상 시간

    #Special pose index
    init_pose = -1
    finish_pose = -1
    crouch_init_pose = -1
    jumping_down_pose = -1

    
    # 모션 매쳐 객체
    motionMatcher = ''

    first = True
    prevInput = [-1, -1, -1]
    prevVelocity = [0,-1,0] 
    idle = True

    # 생성자
    def __init__(self):
        print("Start")
        self.find_fix_pose()


    # special pose index 찾는 함수 (기본 초기화 포즈/crouch 초기화 포즈/점프 초기화 포즈/승리 포즈)
    def find_fix_pose(self):
        for index in range(len(poses)-1):
            if self.init_pose != -1 and self.finish_pose != -1 and self.crouch_init_pose != -1 and self.jumping_down_pose != -1:
                break
            if self.init_pose == -1 and poses[index]['animInfo'][0]['name'] == 'Idle.fbx':
                self.init_pose = index
                self.motionMatcher = MotionMatcher(index)

            if self.finish_pose == -1 and poses[index]['animInfo'][0]['name'] == 'Victory Idle.fbx':
                self.finish_pose = index
            if self.crouch_init_pose == -1 and poses[index]['animInfo'][0]['name'] == 'Idle Crouching.fbx':
                self.crouch_init_pose = index
            if self.jumping_down_pose == -1 and poses[index]['animInfo'][0]['name'] == 'Jumping Down.fbx':
                self.jumping_down_pose = index


    # 소멸자
    def __del__(self):
        print("End")      
        obj = bpy.data.objects['Armature']
        bone_struct = obj.pose.bones
        joint_names = bone_struct.keys()
        self.find_fix_pose()

        # 조인트 별로 위치/회전 정보 초기화 & T 포즈 초기화
        for joint in joint_names:
            jointRotation = poses[self.init_pose]['joints'][joint]['rotation']
            jointLocation = poses[self.init_pose]['joints'][joint]['location']
            bone_struct[joint].location = jointLocation            
            bone_struct[joint].rotation_quaternion = jointRotation
            obj.location = [0,0,0]
            obj.rotation_euler = DEFAULT_EULER

        # 파란색/노란색 점들 원위치
        for index in range(5):
            bpy.data.objects['Point'+ str(index+1)].location = [0,index/-6,0]
            bpy.data.objects['Feature'+ str(index+1)].location = [0,index/-6.1,0]
            

    def execute(self, context):
        return {'FINISHED'}
    
    def setMove(self, type, value):        
        if self.KEY_MAP.keys().__contains__(type):         
            if value == 'PRESS':
                self.KEY_MAP[type] = True
                self.idle = False
            elif value == 'RELEASE':
                self.KEY_MAP[type] = False
                self.idle = True


    def calculateStandard(self, data, mean, std):
        locationList = []
        for location in data:
            locationList.append((location-mean)/std)
        return locationList


    def move(self):
        obj = bpy.data.objects['Armature']
        bone_struct = obj.pose.bones
        boneLocation = obj.rotation_euler.to_matrix() @ mathutils.Vector([bone_struct['mixamorig2:Hips'].location[0]/100,
                                                                            bone_struct['mixamorig2:Hips'].location[1]/(100),
                                                                            bone_struct['mixamorig2:Hips'].location[2]/100])
        globalYLocation = boneLocation[1]+obj.location[1]

        input_direction = np.array([0.0, 0.0, 0.0])
        # 상하
        if self.KEY_MAP['UP_ARROW']:
            input_direction[Z] += GOAL 
        if self.KEY_MAP['DOWN_ARROW']:
            input_direction[Z] += (-1*GOAL) 


        # 좌우
        if self.KEY_MAP['LEFT_ARROW']:
            input_direction[X] += GOAL
            if input_direction[Z]>0: input_direction[Z] -= GOAL/2
            else: input_direction[Z] += GOAL/2
        if self.KEY_MAP['RIGHT_ARROW']:
            input_direction[X] += (-1*GOAL) 
            if input_direction[Z]>0: input_direction[Z] -= GOAL/2
            else: input_direction[Z] += GOAL/2

        # 웅크리기 / 점프 / 달리기
        if self.KEY_MAP[CROUCH]:
            if input_direction[Z] < 0: input_direction[Z] -= GOAL
            elif input_direction[Z] > 0 : input_direction[X] += GOAL/3
        elif self.KEY_MAP[JUMP]:
            input_direction = np.array([0, GOAL*3, input_direction[Z]/3])
            if input_direction[Z] < 0: input_direction[Z] -= GOAL*3
        elif self.KEY_MAP[RUN]:
                input_direction *= 2.5


        # 이전 인풋과 현재 인풋이 다른 경우 즉시 교체
        if not np.array_equal(self.prevInput, input_direction):
                self.motionMatcher.time = UPDATE_TIME

        # 입력의 크기가 0보다 큰 경우 -> 유효한 쿼리 생성후 업데이트 함수 호출
        if np.linalg.norm(input_direction) > 0.0: 
            self.idle = False
            queryVector = self.createQueryVector(input_direction, self.KEY_MAP[CROUCH])
            self.motionMatcher.updateMatchedMotion(queryVector,  -1)
            self.prevInput = input_direction

        # 골에 도착한 경우 세레모니 동작 
        elif globalYLocation < FINISH_LINE:
            queryVector = self.createQueryVector(input_direction)
            self.motionMatcher.updateMatchedMotion(queryVector, self.finish_pose)

        # 웅크리기 상태로 인풋이 없는 경우 -> 웅크리기 초기화 자세 / 이외의 상태일 때 인풋이 없는 경우 -> 기본 초기화 자세
        else:
            queryVector = self.createQueryVector(input_direction)
            if self.KEY_MAP[CROUCH]: self.motionMatcher.updateMatchedMotion(queryVector, self.crouch_init_pose)
            elif self.idle and not self.KEY_MAP[RUN]:
                self.motionMatcher.time = UPDATE_TIME
                self.prevInput = [-1,-1,-1]
                self.motionMatcher.updateMatchedMotion(queryVector, self.init_pose)
        
        if self.first: self.first = False



    def createQueryVector(self, input_direction, crouch = False):
        obj = bpy.data.objects['Armature']
        bone_struct = obj.pose.bones

        # 캐릭터의 현재 글로벌 위치 및 축 구하기 => M update
        boneLocation = obj.rotation_euler.to_matrix() @ mathutils.Vector([bone_struct['mixamorig2:Hips'].location[0]/100,
                                                                            bone_struct['mixamorig2:Hips'].location[1]/(100),
                                                                            bone_struct['mixamorig2:Hips'].location[2]/100])
        globalLocation = [boneLocation[0]+obj.location[0],boneLocation[1]+obj.location[1],boneLocation[2]+obj.location[2]]
        axes = obj.rotation_euler.to_matrix() @ mathutils.Matrix([
            bone_struct['mixamorig2:Hips'].x_axis,
            bone_struct['mixamorig2:Hips'].y_axis,
            bone_struct['mixamorig2:Hips'].z_axis,
        ])
        updateM(globalLocation, axes)
        

        # 포즈 DB에서 현재 포즈 정보 가져오기
        poseStructs = self.motionMatcher.getCurrentPose()
        poseDBVelocity = poseStructs['mixamorig2:Hips']['velocity']

        # run 상태에 따라 스피드 조정
        speed = 1
        if self.KEY_MAP[RUN]: speed *= 2
        # 웅크리기 상태에 따라 힙 위치 조정
        if crouch: hipHeight = -30
        else: hipHeight = -5

        # 각종 포즈 정보들 채우기
        rootVelocity = speed*bone_struct['mixamorig2:Hips'].z_axis
        RfootLocation = poseStructs['mixamorig2:RightFoot']['tailLocation']
        RfootVelocity= poseStructs['mixamorig2:RightFoot']['tailVelocity']
        LfootLocation = poseStructs['mixamorig2:LeftFoot']['tailLocation']
        LfootVelocity= poseStructs['mixamorig2:LeftFoot']['tailVelocity']


        # 궤적 예측하기
        self.nowLocation = [0, 0, 0]
        self.nowVelocity = [rootVelocity[0],rootVelocity[1],rootVelocity[2]]
        self.desiredLocation =  input_direction

        printPoint = []
        trajectoryLocation = []
        trajectoryDirection = []
       
        for index in range(10): # 5개의 점을 계산하기 위한 루프 -> 방향계산을 위해서 10개 구하고 홀수번째만 사용
            updatePosition =  self.calculateFutureTrajectory(0.13) # 4/30 초 = 0.13
            if index % 2 != 0: 
                nowRotationMatrix = mathutils.Quaternion(poses[self.motionMatcher.matched_frame_index]['joints']['mixamorig2:Hips']['rotation']).to_matrix() 
                diff = obj.rotation_euler.to_matrix() @  mathutils.Matrix(nowRotationMatrix) @ mathutils.Vector(updatePosition)
                printPoint.append([diff[0] + globalLocation[0], diff[1] + globalLocation[1],  diff[2] + globalLocation[2]])
                trajectoryLocation.extend(updatePosition)
                trajectoryDirection.extend(substractArray3(updatePosition,self.nowLocation))
                self.desiredLocation = self.nowLocation + input_direction
            self.nowLocation = updatePosition
            
        # 궤적 예측에 맞게 노란색 점 위치 설정
        for index, point in enumerate(printPoint):
            bpy.data.objects['Point'+ str(index+1)].location = point


        return [hipHeight/2] + (np.array(LfootLocation)).tolist() + (np.array(RfootLocation)).tolist() + (np.array(LfootVelocity)/2).tolist() + (np.array(RfootVelocity)/2).tolist() + (np.array(trajectoryLocation)*7).tolist() 
      

    # spring damper 책 참고
    def calculateFutureTrajectory(self,timeDelta):
        omega = 2.0/self.smoothTime
        x = omega*timeDelta
        exp = 1.0/(1.0+x+0.48*x*x+0.235*x*x*x)
        change = self.nowLocation - self.desiredLocation
        temp = (self.nowVelocity+omega*change)*timeDelta
        self.nowVelocity = (self.nowVelocity-omega*temp)*exp
        return self.desiredLocation+(change+temp)*exp


    def modal(self, context, event):
        if event.type == 'LEFTMOUSE':  # Confirm
            return {'FINISHED'}
        elif event.type in {'RIGHTMOUSE', 'ESC'}:  # Cancel
            return {'CANCELLED'}
        elif event.type == 'MOUSEMOVE':
            pass
        else:
            self.setMove(event.type, event.value) # Apply
        if event.type not in {'MOUSEMOVE', 'INBETWEEN_MOUSEMOVE'} or self.first:
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
    print()
    
    
    
# Register and add to the object menu (required to also use F3 search "Modal Operator" for quick access).
bpy.utils.register_class(ModalOperator)
bpy.types.VIEW3D_MT_object.append(menu_func)

# test call
bpy.ops.object.modal_operator('INVOKE_DEFAULT')


