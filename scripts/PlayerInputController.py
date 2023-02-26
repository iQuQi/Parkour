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
    init_pose = 0
    prevVelocity = [0,-1,0] 
    
    # 모션 매쳐 객체
    motionMatcher = ''

    def __init__(self):
        print("Start")
        for index in range(len(poses)-1):
            if poses[index]['animInfo'][0]['name'] == 'T-Pose.fbx':
                self.init_pose = index
                self.motionMatcher = MotionMatcher(index)


    def __del__(self):
        print("End")      
        obj = bpy.context.object
        bone_struct = obj.pose.bones
        joint_names = bone_struct.keys()
        self.motionMatcher.matched_frame_index = self.init_pose

        for joint in joint_names:
            jointRotation = poses[self.init_pose]['joints'][joint]['rotation']
            jointLocation = poses[self.init_pose]['joints'][joint]['location']
            bone_struct[joint].location = jointLocation            
            bone_struct[joint].rotation_quaternion = jointRotation
            obj.location = [0,0,0]
            obj.rotation_euler = DEFAULT_EULER

        for index in range(6):
            print('포인트 전체 초기화')
            bpy.data.objects['Point'+ str(index+1)].location = [0,index/-6,0]
            bpy.data.objects['Feature'+ str(index+1)].location = [0,index/-6.1,0]

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

    def move(self):
        input_direction = np.array([0.0, 0.0, 0.0])

        # 여기를 local 기준으로 바꾸고, Y = -Z
        # 그걸 다시 global 기준으로 바꿔서 query 생성에 넘기기
        if self.KEY_MAP['UP_ARROW']:
            input_direction[Z] += GOAL 
        if self.KEY_MAP['DOWN_ARROW']:
            input_direction[Z] += (-1*GOAL) 
        if self.KEY_MAP['LEFT_ARROW']:
            input_direction[X] += GOAL
            if input_direction[Z]==0: input_direction[Z] += GOAL/2
        if self.KEY_MAP['RIGHT_ARROW']:
            input_direction[X] += (-1*GOAL) 
            if input_direction[Z]==0: input_direction[Z] += GOAL/2
        if self.KEY_MAP[RUN]:
            input_direction *= 2.5

        print('INPUT:', input_direction)
        
        if np.linalg.norm(input_direction) > 0.0: 
            queryVector = self.createQueryVector(input_direction)
            self.motionMatcher.updateMatchedMotion(queryVector, self.KEY_MAP[CROUCH], self.KEY_MAP[JUMP])
        else:
            self.motionMatcher.time = UPDATE_TIME

    def createQueryVector(self, input_direction):
        obj = bpy.context.object
        bone_struct = obj.pose.bones
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

        # 포즈 특징 채우기
        poseStructs = self.motionMatcher.getCurrentPose()
        poseDBVelocity = poseStructs['mixamorig2:Hips']['velocity']
        #speed = np.linalg.norm([poseDBVelocity[0]/100,poseDBVelocity[1]/100,poseDBVelocity[2]/100]) 
        speed = 1
        if speed == 0: speed = ZERO_VELOCITY 
        if self.KEY_MAP[RUN]: speed *= 2
        
        print('SPEED:', speed)
        rootVelocity = speed*bone_struct['mixamorig2:Hips'].z_axis

        RfootLocation = poseStructs['mixamorig2:RightFoot']['tailLocation']
        RfootVelocity= poseStructs['mixamorig2:RightFoot']['velocity']

        LfootLocation = poseStructs['mixamorig2:LeftFoot']['tailLocation']
        LfootVelocity= poseStructs['mixamorig2:LeftFoot']['velocity']

  
        self.nowLocation = [0, 0, 0]
        self.nowVelocity = [rootVelocity[0],rootVelocity[1],rootVelocity[2]]
        self.desiredLocation =  input_direction

        printPoint = []
        trajectoryLocation = []
        trajectoryDirection = []
       
        for index in range(10):
            updatePosition =  self.calculateFutureTrajectory(0.13)
            # if index == 1:
            #     printPoint.append([globalLocation[0], globalLocation[1],0])
            #     trajectoryLocation.extend(self.nowLocation)
            if index % 2 != 0: 
                diff = obj.rotation_euler.to_matrix() @ mathutils.Vector(updatePosition)
                printPoint.append([diff[0] + globalLocation[0], diff[1] + globalLocation[1], 0])

                trajectoryLocation.extend(updatePosition)
                trajectoryDirection.extend(substractArray3(updatePosition,self.nowLocation))
                self.desiredLocation = self.nowLocation + input_direction


            self.nowLocation = updatePosition
            

        for index, point in enumerate(printPoint):
            bpy.data.objects['Point'+ str(index+1)].location = point

        print('궤적 예측 포인트-Local', self.calculateStandard(trajectoryLocation, features[-1]['mean']['hips']['location'], features[-1]['std']['hips']['location']))
        # return rootVelocity + LfootVelocity + RfootVelocity + LfootLocation + RfootLocation + trajectoryLocation + trajectoryDirection
        # return LfootLocation + RfootLocation + (np.array(trajectoryLocation)).tolist()
        # return self.calculateStandard(LfootLocation, features[-1]['mean']['Lfoot']['tailLocation'], features[-1]['std']['Lfoot']['tailLocation']) +self.calculateStandard(RfootLocation, features[-1]['mean']['Lfoot']['tailLocation'], features[-1]['std']['Lfoot']['tailLocation']) + self.calculateStandard(trajectoryLocation, features[-1]['mean']['hips']['location'], features[-1]['std']['hips']['location'])
        # return self.calculateStandard(LfootLocation, features[-1]['mean']['Lfoot']['tailLocation'], features[-1]['std']['Lfoot']['tailLocation']) +self.calculateStandard(RfootLocation, features[-1]['mean']['Lfoot']['tailLocation'], features[-1]['std']['Lfoot']['tailLocation']) + (np.array(trajectoryLocation)*10).tolist()
        return trajectoryLocation
        # return self.calculateStandard(trajectoryLocation, features[-1]['mean']['hips']['location'], features[-1]['std']['hips']['location'])

   

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
        print('EVNET MODAL!!!!!!!!')

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


