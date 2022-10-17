# import sys
# print(sys.path)
from PlayerController import PlayerController
import keyboard

VECTOR3 = (0,0,0)
KEY_CODE = {'Z': 6,'X': 7, 'C': 8,'V': 9, 'SPACE': 49, 'UP': 123, 'DOWN': 124, 'RIGHT' : 125, 'LEFT': 126}

class PlayerInputController:
    def __init__(self):
        self.inputText = ''
        self.mCamForward = VECTOR3        
        self.mMove = VECTOR3
        self.mJump = False 

        self.mCharacter = PlayerController()
        # m_Cam                  
            

    def start():
        print('start')


    def update():
        print('update')

    def fixedUpdate(self):
        print('fixed update')



def keyboardTest():
    for key in KEY_CODE.keys():
        print(key, KEY_CODE[key])
        print('%s was pressed!' % key)
        keyboard.add_hotkey(KEY_CODE[key], lambda: print(' %s was pressed!' % key))
    keyboard.wait()



if __name__ == '__main__':
        controller = PlayerInputController()
        print('test', controller.mCharacter.player)
        keyboardTest()

        

