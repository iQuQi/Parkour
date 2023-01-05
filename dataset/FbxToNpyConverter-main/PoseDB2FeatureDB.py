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
X = 0
Y = 1
Z = 2

def global2local(fir,M ):
    tmp = fir.copy()
    tmp.append(1)
    return (M@tmp)[:-1].tolist()

def getJoint(frames, frameNum,joint):
    return frames[frameNum]['joints'][joint]

class Feature:
    def __init__(self, 
    root_speed,
    Rfoot_location, Rfoot_speed, 
    Lfoot_location, Lfoot_speed, 
    trajectory_location, trajectory_direction, index):

        # 속도
        self.rootSpeed = root_speed

        # 루트의 미래+과거 궤적(바닥에 투영된 2D)   ====> 과거 -> 현재 -> 미래 (4개)
        self.trajectoryLocation = trajectory_location # [-10, -5, 0, 5, 10, 20] ,[[xyz],[],[]] => [x y z x y z x y z]
        self.trajectoryDirection = trajectory_direction # 각 지점에서의 순간 방향

        self.footLocation = {'left': Lfoot_location, 'right': Rfoot_location}
        self.footSpeed =  {'left': Lfoot_speed, 'right': Rfoot_speed}

        self.poseIndex = index

        # TODO: 손 정보 추가
    
#Source directory where .fbx exist
SRC_DATA_DIR ='regular'

#Ouput directory where .fbx to JSON dict will be stored
OUT_DATA_DIR ='fbx2json'

#Final directory where NPY files will ve stored
FINAL_DIR_PATH ='json2npy'

#Combined Files Path
COMBINED_FILE_PATH = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))+'/'

INITIALIZE_VECTOR = [0,0,0]


def poseDB2featureDB():

    # 파일 로드
    file_path = os.path.join(COMBINED_FILE_PATH, 'PoseDB.json')
    with open(file_path, 'r') as f:
        frames=json.load(f)
    print('Pose2Feature Count:', len(frames))

    features_npy = []
    features_json = []

    now_start_index = 0
    now_end_index = 0

    for i in range(len(frames)):
        FRAME = frames[i]['joints']
        ANIM_INFO = frames[i]['animInfo'][0]
        HIP_KEY = 'mixamorig2:Hips' 
        print('anim info',ANIM_INFO)

        if i == 0:
            now_end_index = ANIM_INFO['end']

        if ANIM_INFO['start'] != now_start_index:
            now_start_index = ANIM_INFO['start']
            now_end_index =  ANIM_INFO['end']

        # if i < (now_start_index + 10) or i > (now_end_index - 20):
        #     continue
        if i < (now_start_index) or i > (now_end_index - 20):
             continue

        # 포즈 특징 채워주기
        root_velocity = FRAME[HIP_KEY]['velocity'] 

        Rfoot_location = FRAME['mixamorig2:RightFoot']['location']
        Rfoot_velocity = FRAME['mixamorig2:RightFoot']['velocity']

        Lfoot_location = FRAME['mixamorig2:LeftFoot']['location']
        Lfoot_velocity = FRAME['mixamorig2:LeftFoot']['velocity']


        # 궤적 특징 채워주기
       
        NOW = frames[i]['joints'][HIP_KEY]
        FUTURE4 = frames[i+4]['joints'][HIP_KEY]
        FUTURE8 = frames[i+8]['joints'][HIP_KEY]
        FUTURE12 = frames[i+12]['joints'][HIP_KEY]
        FUTURE16 = frames[i+16]['joints'][HIP_KEY]
        FUTURE20 = frames[i+20]['joints'][HIP_KEY]


        # z_w = np.array([frames[i]['zAxis'][0], (-1)*frames[i]['zAxis'][2], frames[i]['zAxis'][1]]) # z_w
        x_u = np.array(frames[i]['axes'][0])
        y_v = np.array(frames[i]['axes'][1])
        z_w = np.array(frames[i]['axes'][2]) # z_w
        M = [[x_u[0], y_v[0], z_w[0], NOW['location'][0]],
            [x_u[1], y_v[1], z_w[1],  NOW['location'][1]],
            [x_u[2],  y_v[2], z_w[2], NOW['location'][2]],
            [0, 0, 0, 1]]

        M = np.linalg.inv(M)
        trajectory_location =[
                             global2local(NOW['location'], M),global2local(FUTURE4['location'], M),
                             global2local(FUTURE8['location'], M),global2local(FUTURE12['location'], M),
                             global2local(FUTURE16['location'], M),global2local(FUTURE20['location'], M)
                             ]
        trajectory_direction = [global2local(NOW['velocity'], M), global2local(FUTURE4['velocity'], M), 
                                global2local(FUTURE8['velocity'], M), global2local(FUTURE12['velocity'],M), 
                                global2local(FUTURE16['velocity'], M), global2local(FUTURE20['velocity'], M)]

        new_feature = Feature(global2local(root_velocity,M), Rfoot_location, Rfoot_velocity, Lfoot_location, 
                        Lfoot_velocity, trajectory_location, trajectory_direction, ANIM_INFO['index'])

        # 각 파일에 들어갈 피쳐 배열에 추가
        features_npy.append(np.array(new_feature.__dict__))
        features_json.append(new_feature.__dict__)
        print('featureIndex', len(features_json)-1)

    # npy, json 파일로 저장하기
    np.save(COMBINED_FILE_PATH + 'featureDB.npy', features_npy)
    save_path = os.path.join(COMBINED_FILE_PATH,'featureDB.json')
    with open(save_path,'w') as f:
        json.dump(features_json, f)

        
if __name__ == '__main__':
    poseDB2featureDB()  

    # TEST 2 최종 파일출력
    print('TEST 2 ====================== featureDB Data file') 
    frames=np.load(COMBINED_FILE_PATH  + 'featureDB.npy' , allow_pickle=True)
    print('피쳐DB 최종 개수:', len(frames))
    # for i in range(len(frames)):
    #     print(frames[i])  



