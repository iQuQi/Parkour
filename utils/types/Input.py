from constants import *


class PlayerInput:
      def __init__(self):
        self.velocity = 0.0
        self.acceleration = 0.0     
        self.brake = 0.0
        self.turn = 0.0
        self.jump = True
        self.crouch = False

        self.direction = VECTOR3

        self.angularVelocity = 0.0
        self.m_TurnAmount = 0.0
        self.m_ForwardAmount = 0.0
