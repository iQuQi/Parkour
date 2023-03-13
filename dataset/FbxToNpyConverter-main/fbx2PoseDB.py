from locale import normalize
from math import comb
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

WRONG_PREFIX = ['mixamorig:','mixamorig1:','mixamorig3:','mixamorig4:','mixamorig5:','mixamorig6:',
'mixamorig7:','mixamorig8:','mixamorig9:','mixamorig10:','mixamorig11:','mixamorig12:']
CORRECT_PREFIX = 'mixamorig2:'

X = 0
Y = 1
Z = 2

def global2local(fir,M ):
    tmp = fir.copy()
    tmp.append(1)
    return (M@tmp)[:-1].tolist()

class AnimInfo:
    def __init__(self, name, index, start, end):
        self.name = name
        self.start = start
        self.end = end
        self.index = index

class Joint:
    def __init__(self, location, rotation, angular_velocity, velocity, tail, tail_velocity):
        self.location = location
        self.rotation = rotation
        self.tailLocation = tail
        self.tailVelocity = tail_velocity
        self.angularVelocity = angular_velocity
        self.velocity = velocity  

#Source directory where .fbx exist
SRC_DATA_DIR ='regular'

#Ouput directory where .fbx to JSON dict will be stored
OUT_DATA_DIR ='fbx2json'

#Final directory where NPY files will ve stored
FINAL_DIR_PATH ='json2npy'

#Combined Files Path
COMBINED_FILE_PATH = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))+'/'

INITIALIZE_VECTOR = [0,0,0]

def fbx2PoseDB():
    
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

    combined_out_dict_list = []

    global_index = 0
    start_index = 0 
    end_index = 0
    
    # for anim_name in anims_path:
    for anim_name in anims_path: 
        if anim_name == '.DS_Store': continue
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
        start_index = global_index
        end_index = start_index + int(frame_end)

        prev_location = {}
        prev_rotation = {}
        prev_tail_location = {}
        out_dict_list = []
        
        for i in range(int(frame_end)+1):
            bpy.context.scene.frame_set(i)
            bone_struct = bpy.data.objects['Armature'].pose.bones
            joint_names = bone_struct.keys()

            out_dict = {'joints': {}, 'animInfo': []}       
            for name in joint_names:
                correct_name = name
                for wrong in WRONG_PREFIX:
                    if wrong in name:
                        correct_name = name.replace(wrong, CORRECT_PREFIX)

                local_location = bone_struct[name].matrix_basis @ Vector((0,0,0))
                local_rotation = bone_struct[name].rotation_quaternion

                location = [local_location[X], local_location[Y], local_location[Z]]
                tail_location = [bone_struct[name].tail[X]/100, bone_struct[name].tail[Y]/100, bone_struct[name].tail[Z]/100]
                rotation = [local_rotation.w,local_rotation.x,local_rotation.y,local_rotation.z]

                # 속도 구하기 위해
                if(i==0):
                    velocity = [0,0,0]
                    angular_velocity = [0,0,0,0]
                    tail_velocity = [0,0,0]
                else:
                    velocity = [(local_location[X]-prev_location[name][X])*30,(local_location[Y]-prev_location[name][Y])*30, (local_location[Z]-prev_location[name][Z])*30]
                    q1 = prev_rotation[name]
                    q2 = local_rotation
                    dt = 1/30
                    # angular_velocity = [(local_rotation.w-prev_rotation[name].w)*30,(local_rotation.x-prev_rotation[name].x)*30,(local_rotation.y-prev_rotation[name].y)*30, (local_rotation.z-prev_rotation[name].z)*30]
                    angular_velocity = (np.array([
                                        q1[0]*q2[1] - q1[1]*q2[0] - q1[2]*q2[3] + q1[3]*q2[2],
                                        q1[0]*q2[2] + q1[1]*q2[3] - q1[2]*q2[0] - q1[3]*q2[1],
                                        q1[0]*q2[3] - q1[1]*q2[2] + q1[2]*q2[1] - q1[3]*q2[0]])*(2 / dt)).tolist()
                    tail_velocity = [(tail_location[X]-prev_tail_location[name][X])*30,(tail_location[Y]-prev_tail_location[name][Y])*30, (tail_location[Z]-prev_tail_location[name][Z])*30]

                    # i!=0 일 때 구한 속도 대입
                    out_dict_list[i-1]['joints'][correct_name]['angularVelocity'] = angular_velocity
                    out_dict_list[i-1]['joints'][correct_name]['velocity'] = velocity
                    out_dict_list[i-1]['joints'][correct_name]['tailVelocity'] = tail_velocity


                # Hip의 local z축 정보
                if name[-4:]=='Hips':
                    out_dict['axes'] = np.array([bone_struct[name].x_axis, bone_struct[name].y_axis, bone_struct[name].z_axis]).tolist()
                    
                # M matrix 생성
                x_u = out_dict['axes'][0]
                y_v = out_dict['axes'][1]
                z_w = out_dict['axes'][2] # z_w
                M = [[x_u[0], y_v[0], z_w[0], location[0]],
                    [x_u[1], y_v[1], z_w[1],  location[1]],
                    [x_u[2],  y_v[2], z_w[2], location[2]],
                    [0, 0, 0, 1]]

                M = np.linalg.inv(M)

                # 새로운 joint 추가
                joint = Joint(location, rotation, angular_velocity, velocity, global2local(tail_location,M), global2local(tail_velocity,M))
                out_dict['joints'][correct_name] = joint.__dict__
                
                prev_location[name] = location # name, correct_name 관련 없지만 일단 name으로 둠
                prev_rotation[name] = local_rotation.copy()
                prev_tail_location[name] = tail_location.copy()
            
            animInfo = AnimInfo(anim_name, index = global_index, start = start_index, end = end_index)
            out_dict['animInfo'].append(animInfo.__dict__)
            out_dict_list.append(out_dict)
            global_index += 1

        for i in range(len(out_dict_list)):
            save_path = os.path.join(save_dir,'%04d_keypoints.json'%i)
            with open(save_path,'w') as f:
                json.dump(out_dict_list[i], f)

        combined_out_dict_list.extend(out_dict_list)

    # json 파일 생성
    save_path = os.path.join(COMBINED_FILE_PATH,'PoseDB.json')
    with open(save_path,'w') as f:
        json.dump(combined_out_dict_list, f)


def poseDB2npy():
    
    json_dir = OUT_DATA_DIR
    npy_dir = FINAL_DIR_PATH
    if not os.path.exists(npy_dir):
        os.makedirs(npy_dir)
        
    anim_names = os.listdir(json_dir)
    
    for anim_name in anim_names:
        if anim_name == '.DS_Store': continue
        files_path = os.path.join(json_dir,anim_name,'jointDict')
        frame_files = os.listdir(files_path)
        frame_files.sort()
        
        motion = []
        for frame_file in frame_files:
            file_path = os.path.join(files_path,frame_file)
            
            with open(file_path) as f:
                info = json.load(f)
                joint_2_npy = np.array(info['joints'])
                animInfo_2_npy = np.array(info['animInfo'])
            
            motion.append([joint_2_npy] + [animInfo_2_npy])
        
        
        save_path = os.path.join(npy_dir,anim_name)
        if not os.path.exists(save_path):
            os.makedirs(save_path)
        
        np.save(save_path+'/'+'{i}.npy'.format(i=anim_name), motion)





def combineFiles():
    anims_path = os.listdir(FINAL_DIR_PATH)
    combined_npy_matrix = []

    for anim_name in anims_path:
        if anim_name == '.DS_Store': continue
        anim_file_path = os.path.join(FINAL_DIR_PATH, anim_name, anim_name + '.npy')
        frames = np.load(anim_file_path, allow_pickle=True)
        combined_npy_matrix.extend(frames[:-1]) 

    np.save(COMBINED_FILE_PATH + 'PoseDB.npy', combined_npy_matrix)


      
        
if __name__ == '__main__':
    #Convert .fbx files to JSON dict
    fbx2PoseDB()

    #Convert JSON dict to NPY 
    poseDB2npy()       

    #Combine into a single file
    combineFiles()

    # TEST 2 최종 파일출력
    print('TEST 2 ====================== Combined Data file') 
    frames=np.load(COMBINED_FILE_PATH  + 'PoseDB.npy' , allow_pickle=True)
    print('포즈DB 최종 개수-npy:', len(frames))

    file_path = os.path.join(COMBINED_FILE_PATH, 'PoseDB.json')
    with open(file_path, 'r') as f:
        frames_2=json.load(f)
    print('포즈DB 최종 개수-json:', len(frames_2))
    # for i in range(len(frames)):
    #     print(frames[i])  



