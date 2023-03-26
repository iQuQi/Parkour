import numpy as np
import mathutils
import bpy
# import torch
from utils.common import *
HALF_LIFE = 0.2
DELTA_TIME_DEFAULT = 1/30
DELTA_TIME_HIP = 6/30

class Inertialization:
    inertializedRotations = {}
    inertializedAngularVelocities = {}
    inertializedHips = mathutils.Vector()
    inertializedHipsVelocity = mathutils.Vector()

    offsetRotations = {}
    offsetAngularVelocities = {}
    offsetHips = mathutils.Vector()
    offsetHipsVelocity = mathutils.Vector()

    inertializeIndex = 0
    y = 0
    eyedt_default = 0
    eyedt_hip = 0


    def __init__(self):
        self.y = self.halfLifeToDamping() / 2.0; # this could be precomputed
        self.eyedt_default = self.fastNEgeExp(self.y * DELTA_TIME_DEFAULT) # this could be precomputed if several agents use it the same frame
        self.eyedt_hip = self.fastNEgeExp(self.y * DELTA_TIME_HIP) # this could be precomputed if several agents use it the same frame

        self.reset()

    def reset(self):
        obj = bpy.data.objects['Armature']
        bone_struct = obj.pose.bones
        joint_names = bone_struct.keys()
        numJoints = len(joint_names)

        self.inertializeIndex = 0

        for joint in joint_names:
            self.inertializedRotations[joint] = mathutils.Quaternion()
            self.inertializedAngularVelocities[joint] = mathutils.Vector()
            self.offsetRotations[joint] = mathutils.Quaternion()
            self.offsetAngularVelocities[joint] = mathutils.Vector()

        self.inertializedHips = mathutils.Vector()
        self.inertializedHipsVelocity = mathutils.Vector()
        self.offsetHips = mathutils.Vector()
        self.offsetHipsVelocity = mathutils.Vector()

    def allJointTransition(self, sourcePose, targetPose):
        # Set up the inertialization for joint local rotations (no simulation bone)
        for joint in sourcePose['joints'].keys():
            sourcePoseJoint = sourcePose['joints'][joint]
            targetPoseJoint = targetPose['joints'][joint]

            sourceJointRotation = mathutils.Quaternion(sourcePoseJoint['rotation'])
            targetJointRotation = mathutils.Quaternion(targetPoseJoint['rotation'])

            sourceJointAngularVelocity = mathutils.Vector(sourcePoseJoint['angularVelocity'])
            targetJointAngularVelocity = mathutils.Vector(targetPoseJoint['angularVelocity'])

            self.inertializeJointTransition(sourceJointRotation, sourceJointAngularVelocity, 
                                            targetJointRotation, targetJointAngularVelocity,
                                            key = joint) # offsetRotations[i], offsetAngularVelocities[i]
    

    def hipsPositionTransition(self,sourcePose, targetPose):
        # Set up the inertialization for hips
        sourceHips = sourcePose['joints']['mixamorig2:Hips']
        targetHips = targetPose['joints']['mixamorig2:Hips']

        sourceHipsLocation = mathutils.Vector(sourceHips['location'])
        targetHipsLocation = mathutils.Vector(targetHips['location'])

        sourceHipsVelocity = mathutils.Vector(sourceHips['velocity'])
        targetHipsVelocity = mathutils.Vector(targetHips['velocity'])

        self.inertializeJointTransition4Hips(sourceHipsLocation, sourceHipsVelocity, 
                                        targetHipsLocation, targetHipsVelocity,)


    def updateAllJoint(self, targetPose):
        # Update the inertialization for joint local rotations
        for joint in targetPose['joints'].keys():
            targetPoseJoint = targetPose['joints'][joint]
            targetJointRotation = mathutils.Quaternion(targetPoseJoint['rotation'])
            targetJointAngularVelocity = mathutils.Vector(targetPoseJoint['angularVelocity'])
            self.inertializeJointUpdate(targetJointRotation, targetJointAngularVelocity,key = joint)


    def updateHipsPosition(self, targetPose): 
        #Update the inertialization for hips
        targetHips = targetPose['joints']['mixamorig2:Hips']
        targetHipsLocation = mathutils.Vector(targetHips['location'])
        targetRootVelocity = mathutils.Vector(targetHips['velocity'])
        self.inertializeJointUpdate4Hips(targetHipsLocation, targetRootVelocity,)


    def abs(self, q):
        if q.w < 0.0: return mathutils.Quaternion([-q.w, -q.x, -q.y, -q.z]) 
        else: return q

    def inertializeJointTransition(self, sourceRot, sourceAngularVel,
                                    targetRot, targetAngularVel,key): # offsetRot, offsetAngularvel
        targetM = targetRot.to_matrix().inverted()
        sourceM = sourceRot.to_matrix()
        offsetM = self.offsetRotations[key].to_matrix()
 
        matrixMul = self.abs((targetM @ sourceM @ offsetM).to_quaternion())

        self.offsetRotations[key] = matrixMul.normalized()
        self.offsetAngularVelocities[key] = (sourceAngularVel + self.offsetAngularVelocities[key]) - targetAngularVel


    def inertializeJointTransition4Hips(self, source, sourceVel,
                                        target, targetVel,): # offset, offsetVel
        self.offsetHips = (source + self.offsetHips) - target
        self.offsetHipsVelocity = (sourceVel + self.offsetHipsVelocity) - targetVel


    def inertializeJointUpdate(self, targetRot, targetAngularVel, key): 
        # offsetRot = offsetRotations, offsetAngularVel = offsetAngularVelocities, newRot = inertializedRotations, newAngularVel = inertializedAngularVelocities
        self.decaySpringDamperImplicit(key)

        targetM = targetRot.to_matrix()
        offsetM = self.offsetRotations[key].to_matrix()
        matrixMul = targetM @ offsetM

        self.inertializedRotations[key] = matrixMul.to_quaternion()
        self.inertializedAngularVelocities[key] = targetAngularVel + self.offsetAngularVelocities[key]
        

    def inertializeJointUpdate4Hips(self, target, targetVel):
        self.decaySpringDamperImplicit4Hips()
        self.inertializedHips = target + self.offsetHips
        self.inertializedHipsVelocity = targetVel + self.offsetHipsVelocity


    def decaySpringDamperImplicit(self, key):
        rot = self.offsetRotations[key]
        angularVel = self.offsetAngularVelocities[key]
        j0 = self.quaternionToScaledAngleAxis(rot)
        j1 = angularVel + j0 * self.y

        self.offsetRotations[key] = self.quaternionFromScaledAngleAxis(self.eyedt_default * (j0 + j1 * DELTA_TIME_DEFAULT))
        self.offsetAngularVelocities[key] = (self.eyedt_default * (angularVel - j1 * self.y * DELTA_TIME_DEFAULT))
    
    def decaySpringDamperImplicit4Hips(self):

        j1 = self.offsetHipsVelocity + self.offsetHips * self.y

        self.offsetHips = self.eyedt_hip * (self.offsetHips + j1 * DELTA_TIME_HIP)
        self.offsetHipsVelocity = self.eyedt_hip * (self.offsetHipsVelocity - j1 * self.y * DELTA_TIME_HIP)
        
    
    def halfLifeToDamping(self, eps = 1e-5):
        LN2f = 0.69314718056
        return (4.0 * LN2f) / (HALF_LIFE + eps)
        

    def fastNEgeExp(self, x):
        return 1.0 / (1.0 + x + 0.48 * x * x + 0.235 * x * x * x)
        
    def quaternionToScaledAngleAxis(self, q, eps = 1e-8):
        return 2.0 * self.log(q, eps)

    def quaternionFromScaledAngleAxis(self, angleAxis, eps = 1e-8):
        return self.exp(mathutils.Quaternion(angleAxis) * 0.5, eps)
    
    def log(self, q, eps = 1e-8):
        length = np.sqrt(q.x * q.x + q.y * q.y + q.z * q.z)
        if (length < eps):
            return mathutils.Vector([q.x, q.y, q.z])
        else:
            halfangle = np.arccos(self.clamp(q.w, minValue=-1.0, maxValue=1.0))
            return mathutils.Vector(halfangle * (mathutils.Vector([q.x, q.y, q.z]) / length))

    def clamp(self, x, minValue, maxValue):
        return min(max(x, minValue), maxValue)    

    def exp(self, angleAxis, eps = 1e-8):
        halfangle = np.sqrt(angleAxis.x * angleAxis.x + angleAxis.y * angleAxis.y + angleAxis.z * angleAxis.z)
        if (halfangle < eps):
            return mathutils.Quaternion([1.0, angleAxis.x, angleAxis.y, angleAxis.z]).normalized()
        
        else:
            c = np.cos(halfangle)
            s = np.sin(halfangle) / halfangle
            return mathutils.Quaternion([c, s * angleAxis.x, s * angleAxis.y, s * angleAxis.z])
        
        
        