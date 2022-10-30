import bpy


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
        if self.up:
            context.object.location.y += (-1) * self.weight
        if self.down:
            context.object.location.y += 1 * self.weight
        if self.left:
            context.object.location.x += 1 * self.weight
        if self.right:
            context.object.location.x += (-1) * self.weight
        if self.crouch:
            context.object.location.z += (-1) * self.weight
        if self.jump:
            context.object.location.z += 1 * self.weight

        input_location = [context.object.location.x, context.object.location.y, context.object.location.z]
        
    def modal(self, context, event):
        if not event.is_repeat:
            self.weight = 0.5
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


