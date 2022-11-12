import sys
import os

# TODO:경로 수정해야 함
sys.path.insert(1, os.path.dirname(os.path.dirname(os.path.realpath(__file__)))+'/scripts')

from MotionMatcher import MotionMatcher
import numpy as np
import bpy

X = 0
Y = 1
Z = 2
UPDATE_TIME = 0

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


        input_location = [context.object.location.x, context.object.location.y, context.object.location.z]

        if self.up:
            input_location[Y] += (-1) * self.weight
        if self.down:
            input_location[Y] += 1 * self.weight
        if self.left:
            input_location[X] += 1 * self.weight
        if self.right:
            input_location[X] += (-1) * self.weight

        desired_direction = np.array([input_location[X]-context.object.location.x, input_location[Y]-context.object.location.y, input_location[Z]-context.object.location.z])

        if np.linalg.norm(desired_direction) > 0: 
            normalized_direction = desired_direction/np.linalg.norm(desired_direction)
            self.motionMatcher.updateMatchedMotion(normalized_direction, self.crouch, self.jump)
        else:
            self.motionMatcher.time = UPDATE_TIME
            bpy.ops.screen.animation_cancel(restore_frame=False)

        
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


