import numpy as np 
from scipy.spatial import distance
import os
import json

VECTOR3 = [0,0,0]
KEY_CODE = {'Z': 6,'X': 7, 'C': 8,'V': 9, 'SPACE': 49, 'LEFT': 123, 'RIGHT': 124, 'DOWN' : 125, 'UP': 126}
X = 0
Y = 1
Z = 2
UPDATE_TIME = 20
IDLE_INDEX = 30

upper_dir_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

pose_path =  upper_dir_path + '/dataset/PoseDB.json'
with open(pose_path, 'r') as f: poses = json.load(f)

feature_path = upper_dir_path + '/dataset/FeatureDB.json'
with open(feature_path, 'r') as f: features = json.load(f)

def substractArray3(fir,sec):
    return [fir[0]-sec[0],fir[1]-sec[1],fir[2]-sec[2]]

def addArray3(fir,sec):
    return [fir[0]+sec[0],fir[1]+sec[1],fir[2]+sec[2]]

def substractArray4(fir,sec):
    return [fir[0]-sec[0],fir[1]-sec[1],fir[2]-sec[2], fir[3]-sec[3]]

def addArray4(fir,sec):
    return [fir[0]+sec[0],fir[1]+sec[1],fir[2]+sec[2], fir[3]+sec[3]]

def normalizedVector(vector):
    return [vector[X]/np.linalg.norm(vector),vector[Y]/np.linalg.norm(vector),vector[Z]/np.linalg.norm(vector)]
 

M = np.array([])
def global2local(matrix):
    global M
    tmp = matrix.copy()
    tmp.append(1)
    M_inverse = np.linalg.inv(M)
    return (M_inverse@tmp)[:-1].tolist()

def local2global(matrix):
    global M
    tmp = matrix.copy()
    tmp.append(1)
    return (M@tmp)[:-1].tolist()

def updateM(globalLocation, axes):
    global M
    x_u = np.array(axes[0])
    y_v = np.array(axes[1])
    z_w = np.array(axes[2]) # z_w

    print(x_u)

    M = np.array([[x_u[0],y_v[0], z_w[0], globalLocation[0]],
        [x_u[1], y_v[1], z_w[1], globalLocation[1]],
        [x_u[2], y_v[2], z_w[2],  globalLocation[2]],
        [0, 0, 0, 1]])

def calculateDistance(feature, query):
    total = 0
    # print('피쳐의 trajectory 출력 : ', feature)
    # print('쿼리 벡터 출력 : ', query)
    for i in range(len(feature)):
        total += distance.euclidean(feature[i], query[i])
        print('각 차이 : idx = ', i, distance.euclidean(feature[i], query[i]))
    return total