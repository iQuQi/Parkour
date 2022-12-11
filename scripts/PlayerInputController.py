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

def global2local(fir,M ):
    tmp = fir.copy()
    tmp.append(1)
    return (M@tmp)[:-1].tolist()

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
    smoothTime = 0.3    # 최대 속도일 때, 목표물에 도달하는 예상 시간
    
    # 모션 매쳐 객체
    motionMatcher = MotionMatcher()

    def __init__(self):
        print("Start")

    def __del__(self):
        print("End")

    def execute(self, context):
        return {'FINISHED'}
    
    def setMove(self, type, value):
        if self.KEY_MAP.keys().__contains__(type):         
            if value == 'PRESS':
                self.KEY_MAP[type] = True
            elif value == 'RELEASE':
                self.KEY_MAP[type] = False

    def move(self):
        input_direction = np.array([0, 0, 0])

        # 여기를 local 기준으로 바꾸고,
        # 그걸 다시 global 기준으로 바꿔서 query 생성에 넘기기
        if self.KEY_MAP['UP_ARROW']:
            input_direction[Z] += 1 
        if self.KEY_MAP['DOWN_ARROW']:
            input_direction[Z] += (-1) 
        if self.KEY_MAP['LEFT_ARROW']:
            input_direction[X] += (-1) 
        if self.KEY_MAP['RIGHT_ARROW']:
            input_direction[X] += 1 
        if self.KEY_MAP[JUMP]:
            input_direction[Y] += 1 
        if self.KEY_MAP[CROUCH]:
            input_direction[Y] += (-1) 

        print('입력!!---', input_direction)
        
        if np.linalg.norm(input_direction) > 0: 
            normalized_input_direction = input_direction/np.linalg.norm(input_direction)
            
            queryVector = self.createQueryVector(normalized_input_direction)
            self.motionMatcher.updateMatchedMotion(queryVector, self.KEY_MAP[CROUCH], self.KEY_MAP[JUMP])
        else:
            self.motionMatcher.time = UPDATE_TIME

    def createQueryVector(self, normalized_input_direction):

        # 포즈 특징 채우기
        boneStructs = self.motionMatcher.getCurrentPose()
        rootSpeed = np.linalg.norm(boneStructs['mixamorig2:Hips']['velocity'])

        RfootLocation = boneStructs['mixamorig2:RightFoot']['location']
        RfootSpeed=np.linalg.norm(boneStructs['mixamorig2:RightFoot']['velocity'])  

        LfootLocation = boneStructs['mixamorig2:LeftFoot']['location']
        LfootSpeed=np.linalg.norm(boneStructs['mixamorig2:LeftFoot']['velocity'])  
        
        # 궤도 특징 채우기 => 지수 함수 생성으로 예측
        obj = bpy.context.object
        bone_struct = obj.pose.bones
        globalLocation = np.array(bone_struct['mixamorig2:Hips'].location)
        self.nowLocation = [0, 0, 0]
        self.nowVelocity = rootSpeed
        self.desiredLocation = self.nowLocation + normalized_input_direction

        BEFORE10 = self.calculateFutureTrajectory(-10)
        BEFORE5 = self.calculateFutureTrajectory(-5)
        NOW = self.calculateFutureTrajectory(0)
        FUTURE5 = self.calculateFutureTrajectory(5)
        FUTURE10 = self.calculateFutureTrajectory(10)
        FUTURE20 = self.calculateFutureTrajectory(20)

        predict = [globalLocation+BEFORE10, globalLocation+BEFORE5, globalLocation+NOW, globalLocation+FUTURE5, globalLocation+FUTURE10, globalLocation+FUTURE20]

        beforeDirection = [self.calculateFutureTrajectory(-11), self.calculateFutureTrajectory(-6),
                        self.calculateFutureTrajectory(-1), self.calculateFutureTrajectory(4),
                        self.calculateFutureTrajectory(9), self.calculateFutureTrajectory(19) ]


        for point in predict:
            print('궤적 예측 포인트 출력:', point)
            #bpy.ops.mesh.primitive_circle_add( radius=0.1, location = point )

        z_w = self.calculateFutureTrajectory(1)-NOW
        x_u = np.cross([0,0,1], z_w)
        x_u = x_u/np.linalg.norm(x_u)
        y_v = np.cross(z_w, x_u)
        M = [[x_u[0], z_w[0], y_v[0], NOW[0]],
            [x_u[1], z_w[1], y_v[1], NOW[1]],
            [x_u[2], z_w[2], y_v[2], NOW[2]],
            [0, 0, 0, 1]]

        M = np.linalg.inv(M)

        # trajectoryLocation = [
        #                      global2local(BEFORE10, M),global2local(BEFORE5, M),
        #                      global2local(NOW, M),global2local(FUTURE5, M),
        #                      global2local(FUTURE10, M),global2local(FUTURE20, M)
        #                      ]
        # trajectoryDirection = [global2local(BEFORE10-beforeDirection[0], M) , global2local(BEFORE5-beforeDirection[1], M) , global2local(NOW-beforeDirection[2], M) , global2local(FUTURE5-beforeDirection[3], M) , global2local(FUTURE10-beforeDirection[4], M) , global2local(FUTURE20-beforeDirection[5], M)]
        trajectoryLocation = [*(BEFORE10), *(BEFORE5), *(NOW), *(FUTURE5), *(FUTURE10), *(FUTURE20)]
        trajectoryDirection = [*(BEFORE10-beforeDirection[0]) , *(BEFORE5-beforeDirection[1]) , *(NOW-beforeDirection[2]) , *(FUTURE5-beforeDirection[3]) , *(FUTURE10-beforeDirection[4]) , *(FUTURE20-beforeDirection[5])]

        #queryVector = QueryVector(rootSpeed, RfootLocation, RfootSpeed, LfootLocation, LfootSpeed, 
        #                     trajectoryLocation, trajectoryDirection)

        return [rootSpeed,LfootSpeed, RfootSpeed] + LfootLocation + RfootLocation + trajectoryLocation + trajectoryDirection

    def calculateFutureTrajectory(self,timeDelta):
        omega = 2/self.smoothTime
        x = omega*timeDelta
        exp = 1/(1+x+0.48*x*x+0.235*x*x*x)
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


