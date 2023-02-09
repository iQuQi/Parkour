from locale import normalize
from math import comb
import bpy
import os
import time
import sys
import json
from mathutils import Vector
import numpy as np 
import scipy.stats as ss 

HOME_FILE_PATH = os.path.abspath('homefile.blend')
MIN_NR_FRAMES = 64
RESOLUTION = (512, 512)
X = 0
Y = 1
Z = 2
WEIGHT  = 2
L = 'location'
V = 'velocity'
T_L = 'tailLocation'
HIP_KEY = 'mixamorig2:Hips' 
RFOOT_KEY = 'mixamorig2:RightFoot'
LFOOT_KEY = 'mixamorig2:LeftFoot'

def normalizeMatrix(mean,std,M):
    transposeM = np.transpose(M)
    normalized = [(transposeM[0] - mean[0])/std[0],(transposeM[1]  - mean[1])/std[1], (transposeM[2]  - mean[2])/std[2]]

    return np.transpose(normalized).tolist()

def normalizeVector(mean,std,V):
    return ((np.array(V)-mean) / std).tolist()

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

    # 변수값 초기화
    features_npy = []
    features_json = []
    now_start_index = 0
    now_end_index = 0
    frames_np = np.array(frames)
    local_hips = {L:[], V:[]}
    local_RFoot = {T_L:[], V:[]}
    local_Lfoot = {T_L:[], V:[]}
    
    # 속성(위치, 속도)에 맞게 분류해서 배열에 담아주기
    for i in range(len(frames_np)):
        frame = frames_np[i]['joints']

        # 현재 프레임 매트릭스 정보
        x_u = np.array(frames[i]['axes'][0])
        y_v = np.array(frames[i]['axes'][1])
        z_w = np.array(frames[i]['axes'][2]) # z_w
        M = np.linalg.inv([[x_u[0], y_v[0], z_w[0], frame[HIP_KEY][L][0]],
            [x_u[1], y_v[1], z_w[1],  frame[HIP_KEY][L][1]],
            [x_u[2],  y_v[2], z_w[2], frame[HIP_KEY][L][2]],
            [0, 0, 0, 1]])
            
        # 아래 170라인 참고
        # local_hips[L].append(global2local(frame[HIP_KEY][L],M))
        # local_hips[V].append(global2local(frame[HIP_KEY][V],M))
        local_RFoot[T_L].append(frame[RFOOT_KEY][T_L],M)
        local_Lfoot[T_L].append(frame[LFOOT_KEY][T_L],M)
        local_RFoot[V].append(frame[RFOOT_KEY][V],M)
        local_Lfoot[V].append(frame[LFOOT_KEY][V],M)

        
    # 각 열의 평균과 표준편차 구하기 (axis = 0) => x는 x끼리 y는 y끼리..
    hipLocationMean = np.mean(local_hips[L], axis=0).tolist() # 보류
    hipLocationStd = np.std(local_hips[L], axis=0).tolist() # 보류
    RfootLocationMean = np.mean(local_RFoot[T_L], axis=0).tolist()
    LfootLocationMean = np.mean(local_Lfoot[T_L], axis=0).tolist()
    RfootLocationStd = np.std(local_RFoot[T_L], axis=0).tolist()
    LfootLocationStd = np.std(local_Lfoot[T_L], axis=0).tolist()

    mean = {'hip': hipLocationMean, 'Rfoot': RfootLocationMean, 'Lfoot': LfootLocationMean}
    std = {'hip': hipLocationStd, 'Rfoot': RfootLocationStd, 'Lfoot': LfootLocationStd}

    # 표준화 작업 -> HIP을 여기서 안해주는 이유 아래 170라인 참고
    normalizedRfootLocation = normalizeMatrix(RfootLocationMean, RfootLocationStd, local_RFoot[T_L])
    normalizedLfootLocation = normalizeMatrix(LfootLocationMean, LfootLocationStd, local_Lfoot[T_L])

    # TODO location이 잘 작동하면 속도도 적용해보기#############
    local_RFoot[V] = ss.zscore(local_RFoot[V]).tolist()
    local_Lfoot[V] = ss.zscore(local_Lfoot[V]).tolist()

    # 표준화된 결과값을 딕셔너리로 저장
    norm_Rfoot = []
    norm_Lfoot = []
    for i in range(len(frames_np)):
        norm_Rfoot.append({T_L: normalizedRfootLocation[i], V:local_RFoot[V][i]}) 
        norm_Lfoot.append({T_L: normalizedLfootLocation[i], V:local_Lfoot[V][i]})

    for i in range(len(frames)):
        #FRAME = frames[i]['joints']
        ANIM_INFO = frames[i]['animInfo'][0]
        print(ANIM_INFO)

        # 루프의 첫 시작시 설정
        if i == 0:
            now_end_index = ANIM_INFO['end']

        # 새 애니메이션의 정보로  현재 end, start 인덱스 업데이트
        if ANIM_INFO['start'] != now_start_index:
            now_start_index = ANIM_INFO['start']
            now_end_index =  ANIM_INFO['end']

        # 뒤에 16개는 날림
        if i < (now_start_index) or i > (now_end_index - 16):
             continue

        # 포즈 특징 채워주기
        root_velocity = local_hips[V][i] # TODO 표준화 필요 => 보류
        Rfoot_location = norm_Rfoot[i][T_L]
        Rfoot_velocity = norm_Rfoot[i][V]
        Lfoot_location = norm_Lfoot[i][T_L]
        Lfoot_velocity = norm_Lfoot[i][V]

        # TODO 궤적 특징 채워주기 
        # 기존의 방식: 궤적 위치를 전체 위치 데이터(포즈DB 전체)의 평균과 표준편차를 이용해서 표준화 후 로컬 변환 
        #           => 표준화 된 값을 global2local해서 제대로 된 값이 나올리가 없음 
        # 대안: 로컬 변환을 먼저 한 후 궤적 위치 표준화 해주기 => 로컬변환을 하기 위해서는 i번째 M가 필요, 즉 for문안에서 표준화를 해야함
        #      => 특정 프레임에서의 캐릭터 위치에 대한 미래 궤적의 상대위치를 구해놓은 것이므로, 기존의 방식처럼 전체 궤적 위치 데이터의 평균과 표준편차를 사용하지 못함
        #      => i, i+4, ... i+16 이렇게 5개 점에 대한 평균과 표준편차를 사용해야함 - 여기까지는 문제 x
        #      => FeatureDB는 잘 만들어짐. 다만 Input으로 쿼리를 만들 때 문제 발생
        #      => 받아온 인풋을 표준화할 때는 어떤 평균과 표준편차를 사용해야할까 ?
        #        1) 현재 시점에서의 Feature의 궤적 평균과 표준편차를 사용 -> 모든 포즈 프레임이 대응되는 피쳐 프레임 정보를 갖지 않음(16개 사용x라서)
        #        2) ... 잘모르겠움ㅠㅠㅠ
        trajectory_location = []
        for cnt in range(5):
            trajectory_location.append(local_hips[L][i+4*cnt])

        # 궤적 정보 채우기
        # TODO Mean과 Std 값 고치기 -> 위 문제 해결해서
        norm_trajectory_location = normalizeMatrix(hipLocationMean, hipLocationStd, trajectory_location)
        
        # TODO 보류
        trajectory_direction = []

        # 피쳐 객체 생성    
        new_feature = Feature(root_velocity, Rfoot_location, Rfoot_velocity, Lfoot_location, 
                        Lfoot_velocity, norm_trajectory_location, trajectory_direction, ANIM_INFO['index'])


        # 각 파일에 들어갈 피쳐 배열에 추가
        features_npy.append(np.array(new_feature.__dict__))
        features_json.append(new_feature.__dict__)
        print('피쳐인덱스:',len(features_json))
    features_json.append({'mean':mean,'std':std})



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



