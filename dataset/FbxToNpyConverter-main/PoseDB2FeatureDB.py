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


class Feature:
    def __init__(self, 
    root_speed,
    Rfoot_location, Rfoot_speed, 
    Lfoot_location, Lfoot_speed, 
    trajectory_location, trajectory_direction):

        # 속도
        self.rootSpeed = root_speed

        # 루트의 미래+과거 궤적(바닥에 투영된 2D)   ====> 과거 -> 현재 -> 미래 (4개)
        self.trajectoryLocation = trajectory_location # [-10, -5, 0, 5, 10, 20]
        self.trajectoryDirection = trajectory_direction

        self.footLocation = {'left': Lfoot_location, 'right': Rfoot_location}
        self.footSpeed =  {'left': Lfoot_speed, 'right': Rfoot_speed}

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

def normalizedVector(vector):
    return [vector[X]/np.linalg.norm(vector),vector[Y]/np.linalg.norm(vector),vector[Z]/np.linalg.norm(vector)]
 

def poseDB2featureDB():
    file_path = os.path.join(COMBINED_FILE_PATH, 'PoseDB.json')
    with open(file_path, 'r') as f:
        frames=json.load(f)
    features_npy = []
    features_json = []

    for i in range(10, len(frames)-20):
        FRAME = frames[i]['joints']
        key='mixamorig:Hips'
        print('TEST', FRAME )
        root_speed = np.linalg.norm(FRAME['mixamorig:Hips']['velocity']) 

        Rfoot_location = FRAME['mixamorig:RightFoot']['location']
        Rfoot_speed = np.linalg.norm(FRAME['mixamorig:RightFoot']['velocity'])  

        Lfoot_location = FRAME['mixamorig:LeftFoot']['location']
        Lfoot_speed = np.linalg.norm(FRAME['mixamorig:LeftFoot']['velocity']) 

        BEFORE10 = frames[i-10]['joints']['mixamorig:Hips']
        BEFORE5 = frames[i-5]['joints']['mixamorig:Hips']
        NOW = frames[i]['joints']['mixamorig:Hips']
        FUTURE5 = frames[i+5]['joints']['mixamorig:Hips']
        FUTURE10 = frames[i+10]['joints']['mixamorig:Hips']
        FUTURE20 = frames[i+20]['joints']['mixamorig:Hips']

        trajectory_location =[BEFORE10['location']-NOW['location'],BEFORE5['location']-NOW['location'],NOW['location'],
                                    FUTURE5['location']-NOW['location'],FUTURE10['location']-NOW['location'],FUTURE20['location']-NOW['location']]
        trajectory_direction = [BEFORE10['velocity'],BEFORE5['velocity'],NOW['velocity'],
                                    FUTURE5['velocity'],FUTURE10['velocity'],FUTURE20['velocity']]

        new_feature = Feature(root_speed,
            Rfoot_location, Rfoot_speed, Lfoot_location, Lfoot_speed, 
            trajectory_location, trajectory_direction)

        features_npy.append(np.array(new_feature.__dict__))
        features_json.append(new_feature.__dict__)

    np.save(COMBINED_FILE_PATH + 'featureDB.npy', features_npy)
    
    save_path = os.path.join(COMBINED_FILE_PATH,'featureDB.json')
    with open(save_path,'w') as f:
        json.dump(features_json, f)

        
if __name__ == '__main__':
    poseDB2featureDB()  

    # TEST 2 최종 파일출력
    print('TEST 2 ====================== featureDB Data file') 
    frames=np.load(COMBINED_FILE_PATH  + 'featureDB.npy' , allow_pickle=True)
    for i in range(len(frames)):
        print(frames[i])  



