import sys
sys.path.append('/Users/yujin/Documents/GitHub/parkour')
sys.path.append('/Users/yujin/Documents/GitHub/parkour/scripts')
sys.path.append('/Users/yujin/Library/Python/3.9/lib/python/site-packages/keyboard')

from utils.types.constants import *
from PlayerController import *

import keyboard

class PlayerInputController:
    def __init__(self):
        self.inputText = ''
        self.mainCameraForward = VECTOR3        
        self.mainMove = VECTOR3
        self.mainJump = False 

        #self.mainCharacter = PlayerController()
        # m_Cam                  
            

    def start():
        print('start')

    def update():
        print('update')

    def fixedUpdate(self):
        print('fixed update')


def keyboardTest():
    for key in KEY_CODE.keys():
        keyboard.add_hotkey(KEY_CODE[key], print, args=[' %s was pressed!' % key])
    keyboard.wait()


if __name__ == '__main__':
        controller = PlayerInputController()
        keyboardTest()

        

