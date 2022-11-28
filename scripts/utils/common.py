import numpy as np 
VECTOR3 = [0,0,0]
KEY_CODE = {'Z': 6,'X': 7, 'C': 8,'V': 9, 'SPACE': 49, 'LEFT': 123, 'RIGHT': 124, 'DOWN' : 125, 'UP': 126}
X = 0
Y = 1
Z = 2

def substractArray3(fir,sec):
    return [fir[0]-sec[0],fir[1]-sec[1],fir[2]-sec[2]]

def addArray3(fir,sec):
    return [fir[0]+sec[0],fir[1]+sec[1],fir[2]+sec[2]]

def normalizedVector(vector):
    return [vector[X]/np.linalg.norm(vector),vector[Y]/np.linalg.norm(vector),vector[Z]/np.linalg.norm(vector)]
 