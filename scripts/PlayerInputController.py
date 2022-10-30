import bpy


class ModalOperator(bpy.types.Operator):
    bl_idname = "object.modal_operator"
    bl_label = "Simple Modal Operator"
    
    weight = 0
    
    def __init__(self):
        print("Start")

    def __del__(self):
        print("End")

    def execute(self, context):
#        context.object.location.x = self.value / 100.0
        return {'FINISHED'}
    
    def move(self, context, type):
        if type=='UP_ARROW':
            context.object.location.y -= 1 * self.weight
        elif type=='DOWN_ARROW':
            context.object.location.y += 1 * self.weight
        elif type=='LEFT_ARROW':
            context.object.location.x += 1 * self.weight
        elif type=='RIGHT_ARROW':
            context.object.location.x -= 1 * self.weight
        elif type=='C':
            context.object.location.z -= 1 * self.weight
        elif type=='V':
            context.object.location.z += 1 * self.weight

    def modal(self, context, event):
        if not event.is_repeat:
            self.weight = 0.1
        if event.type == 'LEFTMOUSE':  # Confirm
            return {'FINISHED'}
        elif event.type in {'RIGHTMOUSE', 'ESC'}:  # Cancel
            context.object.location.x = self.init_loc_x
            return {'CANCELLED'}
        else:
            if event.is_repeat and self.weight < 1:
                self.weight += 0.1
            self.move(context, event.type) # Apply
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


# Register and add to the object menu (required to also use F3 search "Modal Operator" for quick access).
bpy.utils.register_class(ModalOperator)
bpy.types.VIEW3D_MT_object.append(menu_func)

# test call
bpy.ops.object.modal_operator('INVOKE_DEFAULT')
