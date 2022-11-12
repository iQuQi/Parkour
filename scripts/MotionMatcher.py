#import GitHub.parkour.utils.types.constants

import numpy as np
import os
import re
import bpy

#Combined Files Path
COMBINED_FILE_PATH = os.path.abspath('dataSet.npy')
INTERVAL_TIME = 10
UPDATE_TIME = 0
WRONG_PREFIX = 'mixamorig9'
CORRECT_PREFIX = 'mixamorig'

# TODO:경로 수정해야 함
# frames=np.load('/Users/yujin/Documents/GitHub/parkour/dataset/DataSet.npy', allow_pickle=True)
frames = np.load(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))+'/dataset/DataSet.npy', allow_pickle=True)

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
        print('Function CALL - UpdateMatchedMotion:   ',self.time)
        print(len(frames))
        min_diff = 100000000
        matched_frame_index = -1
        
        if self.time == UPDATE_TIME:
            obj = bpy.context.object
            print('---------------UPDATE TIME ----------------')
            
            # TODO: 매칭 알고리즘 (임시)
            for index, frame in enumerate(frames):
                desired_direction_data = frame[0][0]['desired_direction']
                now_diff = np.linalg.norm(query - desired_direction_data)
                if (now_diff < min_diff):
                    min_diff = now_diff
                    matched_frame_index = index

            print('선택된 프레임 = ', matched_frame_index) 
            
            # TODO: 액션 리스트에 원하는 애니메이션 데이터를 어떻게 불러올 지 정하기 -> 오늘
                # bpy.ops.import_scene.fbx( filepath = '/Users/yujin/Documents/GitHub/parkour/dataset/FbxToNpyConverter-main/regular/Aim Pistol.fbx' )
                # bpy.context.object.hide_viewport = False => 가시성 숨기기
                # 불러온 액션 이름을 애니메이션 이름과 동일하게 변경
                # armature.animation_data.action.name = 'NEW NAME'
                # import한 fbx는 삭제하고 남아있는 action만 사용하기? 가능할까

            
            self.renameAnimation()

            # TODO: 조인트 이름 재설정 (조인트 이름이 다르면 액션 적용이 안됨!) - 완료
            # TODO: 프레임 번호 설정 - 완료
            # TODO: 입력이 멈추면 애니메이션도 멈추기 - 완료 => 천천히 속도를 줄이도록 변경해야함
            # TODO: 프레임 데이터 셋에 애니메이션과 프레임 번호 데이터 추가하기 -> 오늘

            

            obj.animation_data.action = bpy.data.actions.get('Climb') #-> 임시 코드
            # TODO: obj.animation_data.action = bpy.data.actions.get('매칭된 애니메이션 이름') -> 실제 코드
            bpy.context.scene.frame_set(matched_frame_index)
            bpy.ops.screen.animation_play()

            self.time = INTERVAL_TIME

        self.time -= 1


    def renameAnimation(self):        
        # 선택된 armature 가져오기
        armature = bpy.context.active_object
        if armature is None:
            return

        # 연결된 작업을 기억하고 일단 연결을 끊기
        object_actions = {}
        for obj in bpy.data.objects:
            if obj.animation_data is not None:
                object_actions[obj.name] = obj.animation_data.action
                obj.animation_data_clear()

        # 선택한 Armature의 골격 이름 변경
        for bone in armature.data.bones:
            if WRONG_PREFIX in bone.name:
                bone.name = CORRECT_PREFIX + bone.name[len(WRONG_PREFIX):]

        # 저장해둔 액션 재할당
        for obj_name, action_name in object_actions.items():
            bpy.data.objects[obj_name].animation_data_create()
            bpy.data.objects[obj_name].animation_data.action = action_name

