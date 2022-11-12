#import GitHub.parkour.utils.types.constants

import numpy as np
import os
import bpy

#Combined Files Path
COMBINED_FILE_PATH = os.path.abspath('dataSet.npy')
INTERVAL_TIME = 10
UPDATE_TIME = 0

frames=np.load('/Users/yujin/Documents/GitHub/parkour/dataset/DataSet.npy', allow_pickle=True)

class MotionMatcher:
    motion = ''
    time = UPDATE_TIME

    def __init__(self):
        self.motion = 'idle'

    def start():
        print('start')

    def update():
        print('update')

    def updateMatchedMotion(self, query, crouch, jump):
        print('acquireMatchedMotion',self.time)
        print(len(frames))
        min_diff = 100000000
        matched_frame_index = -1
        
        if self.time == UPDATE_TIME:
            print('update time')
            
            # 임시 매칭 알고리즘
            for index, frame in enumerate(frames):
                desired_direction_data = frame[0][0]['desired_direction']
                now_diff = np.linalg.norm(query - desired_direction_data)
                if (now_diff < min_diff):
                    min_diff = now_diff
                    matched_frame_index = index

            print('선택된 프레임', matched_frame_index) 
            
            #bpy.ops.import_scene.fbx( filepath = '/Users/yujin/Documents/GitHub/parkour/dataset/FbxToNpyConverter-main/regular/Aim Pistol.fbx' )
            obj = bpy.context.object
            obj.animation_data.action = bpy.data.actions.get('Climb')
            # for action in bpy.data.actions:
            #     print(action)
            # for obj in bpy.data.objects:
            #     print(obj.animation_data.action)
            # 조인트 이름이 다르면 적용이 안됨
            # 프레임 데이터 셋에 애니메이션과 프레임 번호 데이터 추가하기
            # 액션 리스트에 원하는 애니메이션 데이터를 어떻게 불러올 지 정하기
            bpy.context.scene.frame_set(matched_frame_index)
            bpy.ops.screen.animation_play()

            self.time = INTERVAL_TIME

        self.time -= 1

        

