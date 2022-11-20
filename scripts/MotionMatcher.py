#import GitHub.parkour.utils.types.constants

import numpy as np
import os
import re
import bpy
import json

#Combined Files Path
COMBINED_FILE_PATH = os.path.abspath('dataSet.npy')
INTERVAL_TIME = 10
UPDATE_TIME = 0

# TODO:경로 수정해야 함
# frames=np.load('/Users/yujin/Documents/GitHub/parkour/dataset/DataSet.npy', allow_pickle=True)
pose_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + '/dataset/PoseDB.json'
print('test!!!', pose_path)

with open(pose_path, 'r') as f:
    poses = json.load(f)

feature_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + '/dataset/FeatureDB.json'
with open(feature_path, 'r') as f:
    features = json.load(f)


class MotionMatcher:
    motion = ''
    time = UPDATE_TIME
    # 메인 캐릭터
    mainObj = ''

    def __init__(self):
        self.motion = 'idle'
        self.mainObj = ''

    def start():
        print('start')

    def update():
        print('update')

    def updateMatchedMotion(self, query, crouch, jump):
        print('Function CALL - UpdateMatchedMotion:   ',self.time)
        min_diff = 100000000
        matched_frame_index = 30
        
        if self.time == UPDATE_TIME:
            obj = bpy.context.object
            print('---------------UPDATE TIME ----------------')
            
            # TODO: 매칭 알고리즘 (임시)
            # for index, frame in enumerate(features):
            #     desired_direction_data = frame[0][0]['desired_direction']
            #     now_diff = np.linalg.norm(query - desired_direction_data)
            #     if (now_diff < min_diff):
            #         min_diff = now_diff
            #         matched_frame_index = index
            
            print('선택된 프레임 = ', matched_frame_index) 
            
            bone_struct = obj.pose.bones
            joint_names = bone_struct.keys()

            for joint in joint_names:
                bone_struct[joint].location = poses[matched_frame_index]['joints'][joint]['location']
                bone_struct[joint].rotation_quaternion = poses[matched_frame_index]['joints'][joint]['rotation']
        
            # TODO: 조인트 이름 재설정 (조인트 이름이 다르면 액션 적용이 안됨!) - 완료
            # TODO: 입력이 멈추면 애니메이션도 멈추기 - 완료 => 천천히 속도를 줄이도록 변경해야함
            # TODO: 프레임 데이터 셋에 애니메이션과 프레임 번호 데이터 추가하기 -> 오늘




            self.time = INTERVAL_TIME

        self.time -= 1