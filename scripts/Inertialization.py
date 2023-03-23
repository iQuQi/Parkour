import numpy as np
import mathutils
import bpy
# import torch
from utils.common import *

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


    def __init__(self):
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

    def poseTransition(self, sourcePose, targetPose):
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
        
        # Set up the inertialization for hips
        sourceHips = sourcePose['joints']['mixamorig2:Hips']
        targetHips = targetPose['joints']['mixamorig2:Hips']

        sourceHipsLocation = mathutils.Vector(sourceHips['location'])
        targetHipsLocation = mathutils.Vector(targetHips['location'])

        sourceHipsVelocity = mathutils.Vector(sourceHips['velocity'])
        targetHipsVelocity = mathutils.Vector(targetHips['velocity'])

        self.inertializeJointTransition4Hips(sourceHipsLocation, sourceHipsVelocity, 
                                        targetHipsLocation, targetHipsVelocity,
                                        self.offsetHips, self.offsetHipsVelocity)


    def update(self, targetPose, halfLife, deltaTime):
        # Update the inertialization for joint local rotations
        for joint in targetPose['joints'].keys():
            targetPoseJoint = targetPose['joints'][joint]
            targetJointRotation = mathutils.Quaternion(targetPoseJoint['rotation'])
            targetJointAngularVelocity = mathutils.Vector(targetPoseJoint['angularVelocity'])
            self.inertializeJointUpdate(targetJointRotation, targetJointAngularVelocity,
                                        halfLife, deltaTime,
                                        key = joint)

        
        #Update the inertialization for hips
        targetHips = targetPose['joints']['mixamorig2:Hips']
        targetHipsLocation = mathutils.Vector(targetHips['location'])
        targetRootVelocity = mathutils.Vector(targetHips['velocity'])
        self.inertializeJointUpdate4Hips(targetHipsLocation, targetRootVelocity,
                                    halfLife, deltaTime,
                                    self.offsetHips, self.offsetHipsVelocity,
                                    self.inertializedHips, self.inertializedHipsVelocity)

    def abs(self, q):
        if q.w < 0.0: return mathutils.Quaternion([-q.w, -q.x, -q.y, -q.z]) 
        else: return q

    def inertializeJointTransition(self, sourceRot, sourceAngularVel,
                                    targetRot, targetAngularVel,
                                    key): # offsetRot, offsetAngularvel
        targetM = targetRot.to_matrix().inverted()
        sourceM = sourceRot.to_matrix()
        offsetM = self.offsetRotations[key].to_matrix()
 
        matrixMul = self.abs((targetM @ sourceM @ offsetM).to_quaternion())

        self.offsetRotations[key] = matrixMul.normalized()
        self.offsetAngularVelocities[key] = (sourceAngularVel + self.offsetAngularVelocities[key]) - targetAngularVel

    def inertializeJointTransition4Hips(self, source, sourceVel,
                                        target, targetVel,
                                        offset, offsetVel): # offset, offsetVel
        self.offsetHips = (source + offset) -target
        self.offsetHipsVelocity = (sourceVel + offsetVel) - targetVel

    def inertializeJointUpdate(self, targetRot, targetAngularVel,
                                halfLife, deltaTime,
                                key): 
        # offsetRot = offsetRotations, offsetAngularVel = offsetAngularVelocities, newRot = inertializedRotations, newAngularVel = inertializedAngularVelocities
        self.decaySpringDamperImplicit(key, halfLife, deltaTime)

        targetM = targetRot.to_matrix()
        offsetM = self.offsetRotations[key].to_matrix()
        matrixMul = targetM @ offsetM

        self.inertializedRotations[key] = matrixMul.to_quaternion()
        self.inertializedAngularVelocities[key] = targetAngularVel + self.offsetAngularVelocities[key]
        
    def inertializeJointUpdate4Hips(self, target, targetVel,
                                    halfLife, deltaTime,
                                    offset, offsetVel,
                                    newValue, newVel):
        self.decaySpringDamperImplicit4Hips(offset, offsetVel, halfLife, deltaTime)
        self.inertializedHips = target + offset
        self.inertializedHipsVelocity = targetVel + offsetVel


    def decaySpringDamperImplicit(self, key, halfLife, deltaTime):
        rot = self.offsetRotations[key]
        angularVel = self.offsetAngularVelocities[key]
        y = self.halfLifeToDamping(halfLife) / 2.0; # this could be precomputed
        j0 = self.quaternionToScaledAngleAxis(rot)
        j1 = angularVel + j0 * y
        eyedt = self.fastNEgeExp(y * deltaTime) # this could be precomputed if several agents use it the same frame

        self.offsetRotations[key] = self.quaternionFromScaledAngleAxis(eyedt * j0 + j1 * deltaTime)
        self.offsetAngularVelocities[key] = (eyedt * (angularVel - j1 * y * deltaTime))
    
    def decaySpringDamperImplicit4Hips(self, pos, velocity, halfLife, deltaTime):

        y = self.halfLifeToDamping(halfLife) / 2.0 # this could be precomputed
        j1 = velocity + pos * y
        eyedt = self.fastNEgeExp(y * deltaTime) # this could be precomputed if several agents use it the same frame

        self.offsetHips = eyedt * (pos + j1 * deltaTime)
        self.offsetHipsVelocity = eyedt * (velocity - j1 * y * deltaTime)
        
    
    def halfLifeToDamping(self, halfLife, eps = 1e-5):
        LN2f = 0.69314718056
        return (4.0 * LN2f) / (halfLife + eps)
        

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
        
        
        