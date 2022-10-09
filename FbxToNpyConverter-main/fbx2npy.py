import bpy
import os
import time
import sys
import json
from mathutils import Vector
import numpy as np 

HOME_FILE_PATH = os.path.abspath('homefile.blend')
MIN_NR_FRAMES = 64
RESOLUTION = (512, 512)
X = 0
Y = 1
Z = 2

#Crucial joints sufficient for visualisation #FIX ME - Add more joints if desirable for MixamRig
BASE_JOINT_NAMES = ['Head', 'Neck',
                    'RightArm', 'RightHand', 
                    'LeftArm', 'LeftHand',
                    'Hips', 
                    'RightUpLeg',  'RightFoot', 
                    'LeftUpLeg',  'LeftFoot',
                    ]


#Source directory where .fbx exist
SRC_DATA_DIR ='regular'

#Ouput directory where .fbx to JSON dict will be stored
OUT_DATA_DIR ='fbx2json'

#Final directory where NPY files will ve stored
FINAL_DIR_PATH ='json2npy'

#Number of joints to be used from MixamoRig
joint_names = ['mixamorig:' + x for x in BASE_JOINT_NAMES]

def fbx2jointDict():
    
    #Remove 'Cube' object if exists in the scene
    if bpy.data.objects.get('Cube') is not None:
        cube = bpy.data.objects['Cube']
        bpy.data.objects.remove(cube)
    
    #Intensify Light Point in the scene
    if bpy.data.objects.get('Light') is not None:
        bpy.data.objects['Light'].data.energy = 2
        bpy.data.objects['Light'].data.type = 'POINT'
    
    #Set resolution and it's rendering percentage
    bpy.data.scenes['Scene'].render.resolution_x = RESOLUTION[0]
    bpy.data.scenes['Scene'].render.resolution_y = RESOLUTION[1]
    bpy.data.scenes['Scene'].render.resolution_percentage = 100
    
    #Base file for blender
    bpy.ops.wm.save_as_mainfile(filepath=HOME_FILE_PATH)
    
    #Get animation(.fbx) file paths
    anims_path = os.listdir(SRC_DATA_DIR)
    
    #Make OUT_DATA_DIR
    if not os.path.exists(OUT_DATA_DIR):
        os.makedirs(OUT_DATA_DIR)    
    
    # for anim_name in anims_path:
    for anim_name in anims_path: 
        anim_file_path = os.path.join(SRC_DATA_DIR,anim_name)
        save_dir = os.path.join(OUT_DATA_DIR,anim_name.split('.')[0],'JointDict')
        
        #Make save_dir
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        
        #Load HOME_FILE and .fbx file
        bpy.ops.wm.read_homefile(filepath=HOME_FILE_PATH)
        bpy.ops.import_scene.fbx(filepath=anim_file_path)
        
        #End Frame Index for .fbx file
        frame_end = bpy.data.actions[0].frame_range[1]
        
        prev_location = {}
        prev_rotation = {}
        for i in range(int(frame_end)+1):
    
            bpy.context.scene.frame_set(i)
            bone_struct = bpy.data.objects['Armature'].pose.bones
            out_dict = {'pose_keypoints_3d': []}
        
            
            # TEST 1
            print('TEST 1 ====================== JSON FILE, Frame - ', i+1)
            for name in joint_names:
                local_location = bone_struct[name].matrix_basis @ Vector((0, 0, 0))
                local_rotation = bone_struct[name].rotation_quaternion

                
                
                location = [local_location[X], local_location[Y], local_location[Z]]
                rotation = [local_rotation.w,local_rotation.x,local_rotation.y,local_rotation.z]
                if(i==0):
                    velocity=[0,0,0]
                    angular_velocity=[0,0,0,0]
                else:
                    velocity = [(prev_location[name][X]-local_location[X])*30,(prev_location[name][Y]-local_location[Y])*30, (prev_location[name][Z]-local_location[Z])*30]    
                    angular_velocity = [(prev_rotation[name].w-local_rotation.w)*30,(prev_rotation[name].x-local_rotation.x)*30,(prev_rotation[name].y-local_rotation.y)*30, (prev_rotation[name].z-local_rotation.z)*30]    
                    
                # print('frame:',i,':', anim_name ,name)
                # print(local_location[X],local_location[Y],local_location[Z])
                # print(local_rotation.w,local_rotation.x,local_rotation.y,local_rotation.z)
                # print('velocity: ',velocity)
                # print('angular_velocity: ',angular_velocity)
                
                prev_location[name] = location
                prev_rotation[name] = local_rotation.copy()
                
                out_dict['pose_keypoints_3d'].extend(location + rotation + velocity + angular_velocity)
            
            save_path = os.path.join(save_dir,'%04d_keypoints.json'%i)
            with open(save_path,'w') as f:
                json.dump(out_dict, f)


def jointDict2npy():
    
    json_dir = OUT_DATA_DIR
    npy_dir = FINAL_DIR_PATH
    if not os.path.exists(npy_dir):
        os.makedirs(npy_dir)
        
    anim_names = os.listdir(json_dir)
    
    for anim_name in anim_names:
        files_path = os.path.join(json_dir,anim_name,'jointDict')
        frame_files = os.listdir(files_path)
        frame_files.sort()
        
        motion = []
        for frame_file in frame_files:
            file_path = os.path.join(files_path,frame_file)
            
            with open(file_path) as f:
                info = json.load(f)
                joint = np.array(info['pose_keypoints_3d']).reshape((-1,14))
            motion.append(joint[:11,:])

            
        motion = np.stack(motion,axis=0)
        save_path = os.path.join(npy_dir,anim_name)
        if not os.path.exists(save_path):
            os.makedirs(save_path)
            
        print(save_path)
        
        np.save(save_path+'/'+'{i}.npy'.format(i=anim_name),motion)
        
if __name__ == '__main__':
    #Convert .fbx files to JSON dict
    fbx2jointDict()

    #Convert JSON dict to NPY 
    jointDict2npy()       

    #TEST 2 => 아니왜 hips 결과가 다르지
    print('TEST 2 ====================== NPY file')
    frames=np.load('./json2npy/Aim Pistol/Aim Pistol.npy') 

    for i in range(len(frames)):
        print(i,": ",frames[i])    


