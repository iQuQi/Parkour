import sys
import os
import json

# TODO:경로 수정해야 함
sys.path.insert(1, os.path.dirname(os.path.dirname(os.path.realpath(__file__)))+'/scripts')

from MotionMatcher import MotionMatcher
import numpy as np
import bpy

X = 0
Y = 1
Z = 2
UPDATE_TIME = 0

def calculateFutureTrajectory(timeDelta, nowLocation, desiredLocation, nowVelocity, smoothTime):
    omega = 2/smoothTime
    x = omega*timeDelta
    exp = 1/(1+x+0.48*x*x+0.235*x*x*x)
    change = nowLocation - desiredLocation
    temp = (nowVelocity+omega*change)*timeDelta
    nowVelocity = (nowVelocity-omega*temp)*exp

    return desiredLocation+(change+temp)*exp


class QueryVector:
    def __init__(self, rootSpeed,
            RfootLocation, RfootSpeed, LfootLocation, LfootSpeed, 
            trajectoryLocation, trajectoryDirection):

        # 속도
        self.rootSpeed = rootSpeed

        # 루트의 미래+과거 궤적(바닥에 투영된 2D)   ====> 과거 -> 현재 -> 미래 (4개)
        self.trajectoryLocation = trajectoryLocation # [-10, -5, 0, 5, 10, 20]
        self.trajectoryDirection = trajectoryDirection

        self.footLocation = {'left': LfootLocation, 'right': RfootLocation}
        self.footSpeed =  {'left': LfootSpeed, 'right': RfootSpeed}

        # TODO: 손 정보 추가
    
    # def __str__(self):
    #     return('쿼리입니다 '+ json.dumps(self.rootSpeed)+' '+ json.dumps(self.footLocation)+' '+ json.dumps(self.footSpeed)+ ' '+ json.dumps(self.trajectoryLocation)+ ' '+ json.dumps(self.trajectoryDirection))
    

class ModalOperator(bpy.types.Operator):
    bl_idname = "object.modal_operator"
    bl_label = "Simple Modal Operator"
    
    weight = 0
    up = False
    left = False
    down = False
    right = False
    crouch = False
    jump = False
    input_location = [0,0,0]
    desired_direction = [0,0,0]
    
    motionMatcher = MotionMatcher()

    def __init__(self):
        print("Start")

    def __del__(self):
        print("End")

    def execute(self, context):
#        context.object.location.x = self.value / 100.0
        return {'FINISHED'}
    
    def setMove(self, type, value):
        if type=='UP_ARROW':         
            if value == 'PRESS':
                self.up = True
            elif value == 'RELEASE':
                self.up = False
        if type=='DOWN_ARROW':
            if value == 'PRESS':
                self.down = True
            elif value == 'RELEASE':
                self.down = False         
        if type=='LEFT_ARROW':
            if value == 'PRESS':
                self.left = True
            elif value == 'RELEASE':
                self.left = False       
        if type=='RIGHT_ARROW':
            if value == 'PRESS':
                self.right = True
            elif value == 'RELEASE':
                self.right = False

        if type=='C':
            if value == 'PRESS':
                self.crouch = True
            elif value == 'RELEASE':
                self.crouch = False
          
        if type=='V':
            if value == 'PRESS':
                self.jump = True
            elif value == 'RELEASE':
                self.jump = False
            

    def move(self, context):


        input_location = [0, 0, 0]

        if self.up:
            input_location[Y] += (-1)
        if self.down:
            input_location[Y] += 1 
        if self.left:
            input_location[X] += 1 
        if self.right:
            input_location[X] += (-1) 

        desired_direction = np.array(input_location)
        
        if np.linalg.norm(desired_direction) > 0: 
            normalized_direction = desired_direction/np.linalg.norm(desired_direction)
            queryVector = self.createQueryVector(normalized_direction)
            print(queryVector.rootSpeed,
            queryVector.footLocation, queryVector.footSpeed, 
            queryVector.trajectoryLocation, queryVector.trajectoryDirection)
            self.motionMatcher.updateMatchedMotion(queryVector, self.crouch, self.jump)
        else:
            self.motionMatcher.time = UPDATE_TIME
            # bpy.ops.screen.animation_cancel(restore_frame=False)

    def createQueryVector(self, normalized_direction):
        boneStructs = self.motionMatcher.getCurrentPose()
        rootSpeed = np.linalg.norm(boneStructs['mixamorig:Hips']['velocity'])

        RfootLocation = boneStructs['mixamorig:RightFoot']['location']
        RfootSpeed=np.linalg.norm(boneStructs['mixamorig:RightFoot']['velocity'])  

        LfootLocation = boneStructs['mixamorig:LeftFoot']['location']
        LfootSpeed=np.linalg.norm(boneStructs['mixamorig:LeftFoot']['velocity'])  
        
        w = 100

        # 현재 위치, 키보드 방향
        nowLocation = np.array(bpy.context.object.location)
        nowVelocity = rootSpeed
        desiredLocation = nowLocation + normalized_direction
        smoothTime = 0.3    # the expected time to reach the target when at maximum velocity

        BEFORE10 = calculateFutureTrajectory(-10, nowLocation, desiredLocation, nowVelocity, smoothTime)
        BEFORE5 = calculateFutureTrajectory(-5, nowLocation, desiredLocation, nowVelocity, smoothTime)
        NOW = calculateFutureTrajectory(0, nowLocation, desiredLocation, nowVelocity, smoothTime)
        FUTURE5 = calculateFutureTrajectory(5, nowLocation, desiredLocation, nowVelocity, smoothTime)
        FUTURE10 = calculateFutureTrajectory(10, nowLocation, desiredLocation, nowVelocity, smoothTime)
        FUTURE20 = calculateFutureTrajectory(20, nowLocation, desiredLocation, nowVelocity, smoothTime)

        forDirection = [calculateFutureTrajectory(-11, nowLocation, desiredLocation, nowVelocity, smoothTime),
                        calculateFutureTrajectory(-6, nowLocation, desiredLocation, nowVelocity, smoothTime),
                        calculateFutureTrajectory(-1, nowLocation, desiredLocation, nowVelocity, smoothTime),
                        calculateFutureTrajectory(4, nowLocation, desiredLocation, nowVelocity, smoothTime),
                        calculateFutureTrajectory(9, nowLocation, desiredLocation, nowVelocity, smoothTime),
                        calculateFutureTrajectory(19, nowLocation, desiredLocation, nowVelocity, smoothTime) ]

        trajectoryLocation =[BEFORE10,BEFORE5,NOW,FUTURE5,FUTURE10,FUTURE20]
        trajectoryDirection = [BEFORE10-forDirection[0],BEFORE5-forDirection[1],NOW-forDirection[2],
                                    FUTURE5-forDirection[3],FUTURE10-forDirection[4],FUTURE20-forDirection[5]]

        return QueryVector(rootSpeed,
            RfootLocation, RfootSpeed, LfootLocation, LfootSpeed, 
            trajectoryLocation, trajectoryDirection)
            

    def modal(self, context, event):
        if not event.is_repeat:
            self.weight = 0.3
        if event.type == 'LEFTMOUSE':  # Confirm
            return {'FINISHED'}
        elif event.type in {'RIGHTMOUSE', 'ESC'}:  # Cancel
            context.object.location.x = self.init_loc_x
            return {'CANCELLED'}
        else:
            if event.is_repeat and self.weight < 1:
                self.weight += 0.1
            self.setMove(event.type, event.value) # Apply
            
        self.move(context)
        return {'RUNNING_MODAL'}

    def invoke(self, context, event):
#        print('invoke')
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


