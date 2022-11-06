#import GitHub.parkour.utils.types.constants

import numpy as np
import os

#Combined Files Path
COMBINED_FILE_PATH = os.path.abspath('dataSet.npy')

frames=np.load(COMBINED_FILE_PATH, allow_pickle=True)

class MotionMatcher:
    motion = ''



    def __init__(self):
        self.motion = 'idle'

    def start():
        print('start')

    def update():
        print('update')

    def acquireMatchedMotion(self, query, crouch, jump):
        print('hi')
        print('acquireMatchedMotion',query, crouch, jump)
        print(len(frames))
        for i in range(len(frames)):
            print(frames[i])  



