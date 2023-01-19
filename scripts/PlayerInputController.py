import sys
import os
import json

sys.path.insert(1, os.path.dirname(os.path.dirname(os.path.realpath(__file__)))+'/scripts')

from MotionMatcher import MotionMatcher
import numpy as np
import bpy
from utils.common import *

JUMP = 'V'
CROUCH = 'C'
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
        print('--- 쿼리 벡터 정보 출력하기 ---')
        print('rootSpeed:', self.rootSpeed)
        print('trajectoryLocation:', self.trajectoryLocation)
        print('trajectoryDirection:', self.trajectoryDirection)
        print('footLocation:', self.footLocation)
        print('footSpeed:', self.footSpeed)
        print('-------------------------')


    

class ModalOperator(bpy.types.Operator):
    bl_idname = "object.modal_operator"
    bl_label = "Simple Modal Operator"

    KEY_MAP = {'UP_ARROW': False, 'DOWN_ARROW': False, 'LEFT_ARROW': False, 'RIGHT_ARROW': False, CROUCH: False, JUMP: False }

    # 지수 곡선 인자
    nowLocation =[0,0,0] 
    nowVelocity = -1
    desiredLocation = [0,0,0] 
    smoothTime = 0.25    # 최대 속도일 때, 목표물에 도달하는 예상 시간

    prevVelocity = [0,-1,0] 
    
    # 모션 매쳐 객체
    motionMatcher = MotionMatcher()

    def __init__(self):
        print("Start")

    def __del__(self):
        print("End")      
        obj = bpy.context.object
        bone_struct = obj.pose.bones
        joint_names = bone_struct.keys()

        for joint in joint_names:
            jointRotation = poses[IDLE_INDEX]['joints'][joint]['rotation']
            jointLocation = poses[IDLE_INDEX]['joints'][joint]['location']
            bone_struct[joint].location = jointLocation            
            bone_struct[joint].rotation_quaternion = jointRotation


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
        input_direction = np.array([0, 0, 0])

        # 여기를 local 기준으로 바꾸고, Y = -Z
        # 그걸 다시 global 기준으로 바꿔서 query 생성에 넘기기
        if self.KEY_MAP['UP_ARROW']:
            input_direction[Z] += 1
        if self.KEY_MAP['DOWN_ARROW']:
            input_direction[Z] += (-1) 
        if self.KEY_MAP['LEFT_ARROW']:
            input_direction[X] += 1
        if self.KEY_MAP['RIGHT_ARROW']:
            input_direction[X] += (-1) 
        if self.KEY_MAP[JUMP]:
            input_direction[Y] += 1
        if self.KEY_MAP[CROUCH]:
            input_direction[Y] += (-1) 

        print('입력!!---', input_direction)
        
        if np.linalg.norm(input_direction) > 0: 
            #normalized_input_direction = input_direction/np.linalg.norm(input_direction)
            queryVector = self.createQueryVector(input_direction)
            self.motionMatcher.updateMatchedMotion(queryVector, self.KEY_MAP[CROUCH], self.KEY_MAP[JUMP])
        else:
            self.motionMatcher.time = UPDATE_TIME

    def createQueryVector(self, input_direction):
        obj = bpy.context.object
        bone_struct = obj.pose.bones
        globalLocation = [bone_struct['mixamorig2:Hips'].location[0]/100,
                        bone_struct['mixamorig2:Hips'].location[2]/(-100),
                        bone_struct['mixamorig2:Hips'].location[1]/100]

        # 포즈 특징 채우기
        poseStructs = self.motionMatcher.getCurrentPose()
        
        poseDBVelocity = poseStructs['mixamorig2:Hips']['velocity']
        speed = np.linalg.norm([poseDBVelocity[0]/100,poseDBVelocity[1]/100,poseDBVelocity[2]/100]) 
        if speed == 0: speed = ZERO_VELOCITY
        rootVelocity = np.array([bone_struct['mixamorig2:Hips'].z_axis[0]*speed,(-1*speed)*bone_struct['mixamorig2:Hips'].z_axis[2],speed*bone_struct['mixamorig2:Hips'].z_axis[1]])
        #rootVelocity = np.array([bone_struct['mixamorig2:Hips'].z_axis[0],(-1)*bone_struct['mixamorig2:Hips'].z_axis[2],bone_struct['mixamorig2:Hips'].z_axis[1]])

        axes = [
            [bone_struct['mixamorig2:Hips'].x_axis[0], (-1)*bone_struct['mixamorig2:Hips'].x_axis[2], bone_struct['mixamorig2:Hips'].x_axis[1]],
            [bone_struct['mixamorig2:Hips'].y_axis[0], (-1)*bone_struct['mixamorig2:Hips'].y_axis[2], bone_struct['mixamorig2:Hips'].y_axis[1]],
            [bone_struct['mixamorig2:Hips'].z_axis[0], (-1)*bone_struct['mixamorig2:Hips'].z_axis[2], bone_struct['mixamorig2:Hips'].z_axis[1]]
        ]
        updateM(globalLocation, axes)
        rootVelocity = global2local(rootVelocity.tolist())
        print('루트 속도&글로벌 위치',rootVelocity, globalLocation )

        RfootLocation = poseStructs['mixamorig2:RightFoot']['tailLocation']
        RfootVelocity= poseStructs['mixamorig2:RightFoot']['velocity']

        LfootLocation = poseStructs['mixamorig2:LeftFoot']['tailLocation']
        LfootVelocity= poseStructs['mixamorig2:LeftFoot']['velocity']


  
        self.nowLocation = [0, 0, 0]
        self.nowVelocity = [rootVelocity[0],rootVelocity[1],rootVelocity[2]]
        #self.nowVelocity = [0.1,0.1,0.1]
        self.desiredLocation =  input_direction

        printPoint = []
        trajectoryLocation = []
        trajectoryDirection = []
        for index in range(12):
            updatePosition =  self.calculateFutureTrajectory(0.13)
            print('UPDATE POINT:', updatePosition)
            if index % 2 != 0: 
                printPoint.append(local2global(updatePosition.tolist()))
                trajectoryLocation.extend(updatePosition)
                trajectoryDirection.extend(substractArray3(updatePosition,self.nowLocation))
            self.nowLocation = updatePosition


        for index, point in enumerate(printPoint):
            print('궤적 예측 포인트 출력:', point)
            bpy.data.objects['Point'+ str(index+1)].location = point

        #return rootVelocity + LfootVelocity + RfootVelocity + LfootLocation + RfootLocation + trajectoryLocation + trajectoryDirection

        # return self.calculateStandard(LfootLocation, features[-1]['mean']['Lfoot']['tailLocation'], features[-1]['std']['Lfoot']['tailLocation']) +self.calculateStandard(RfootLocation, features[-1]['mean']['Lfoot']['tailLocation'], features[-1]['std']['Lfoot']['tailLocation']) + self.calculateStandard(trajectoryLocation, features[-1]['mean']['hips']['location'], features[-1]['std']['hips']['location'])
        # return self.calculateStandard(trajectoryLocation, features[-1]['mean']['hips']['location'], features[-1]['std']['hips']['location'])
        return trajectoryLocation

   

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


