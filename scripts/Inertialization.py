import numpy as np
import mathutils
import bpy
# import torch
from utils.common import *

class Inertialization:
    inertializedRotations = [mathutils.Quaternion()]
    inertializedAngularVelocities = [mathutils.Vector()]
    inertializedHips = mathutils.Vector()
    inertializedHipsVelocity = mathutils.Vector()

    offsetRotations = [mathutils.Quaternion()]
    offsetAngularVelocities = [mathutils.Vector()]
    offsetHips = mathutils.Vector()
    offsetHipsVelocity = mathutils.Vector()

    def __init__(self):
        obj = bpy.context.object
        bone_struct = obj.pose.bones
        joint_names = bone_struct.keys()
        numJoints = len(joint_names)
        self.inertializedRotations = [mathutils.Quaternion() for i in range(numJoints)]
        self.inertializedAngularVelocities = [mathutils.Vector() for i in range(numJoints)]
        self.offsetRotations = [mathutils.Quaternion() for i in range(numJoints)]
        self.offsetAngularVelocities = [mathutils.Vector() for i in range(numJoints)]

    # <summary>
    # It takes as input the current state of the source pose and the target pose.
    # It sets up the inertialization, which can then by updated by calling Update(...).
    # </summary>

    def poseTransition(self, sourcePose, targetPose):
        # Set up the inertialization for joint local rotations (no simulation bone)
        for i in range(len(sourcePose['joints'].keys())):
            joint_names = list(sourcePose['joints'].keys())
            sourcePoseJoint = sourcePose['joints'][joint_names[i]]
            targetPoseJoint = targetPose['joints'][joint_names[i]]

            sourceJointRotation = mathutils.Quaternion(sourcePoseJoint['rotation'])
            targetJointRotation = mathutils.Quaternion(targetPoseJoint['rotation'])
            sourceJointAngularVelocity = mathutils.Vector(sourcePoseJoint['angularVelocity'])
            targetJointAngularVelocity = mathutils.Vector(targetPoseJoint['angularVelocity'])
            self.inertializeJointTransition(sourceJointRotation, sourceJointAngularVelocity, 
                                            targetJointRotation, targetJointAngularVelocity,
                                            index = i) # offsetRotations[i], offsetAngularVelocities[i]
        
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

    # <summary>
    # Updates the inertialization decaying the offset from the source pose (specified in PoseTransition(...))
    # to the target pose.
    # </summary>
    def update(self, targetPose, halfLife, deltaTime):
        # Update the inertialization for joint local rotations
        for i in range(len(targetPose['joints'].keys())):
            joint_names = list(targetPose['joints'].keys())

            targetPoseJoint = targetPose['joints'][joint_names[i]]
            targetJointRotation = mathutils.Quaternion(targetPoseJoint['rotation'])
            targetJointAngularVelocity = mathutils.Vector(targetPoseJoint['angularVelocity'])
            self.inertializeJointUpdate(targetJointRotation, targetJointAngularVelocity,
                                        halfLife, deltaTime,
                                        index = i)
        
        # Update the inertialization for hips
        targetHips = targetPose['joints']['mixamorig2:Hips']
        targetHipsLocation = mathutils.Vector(targetHips['location'])
        targetRootVelocity = mathutils.Vector(targetHips['velocity'])
        self.inertializeJointUpdate4Hips(targetHipsLocation, targetRootVelocity,
                                    halfLife, deltaTime,
                                    self.offsetHips, self.offsetHipsVelocity,
                                    self.inertializedHips, self.inertializedHipsVelocity)

    # <summary>
    # Compute the offsets from the source pose to the target pose.
    # Offsets are in/out since we may start a inertialization in the middle of another inertialization.
    # </summary>
    def abs(self, q):
        if q.w < 0.0: return mathutils.Quaternion([-q.w, -q.x, -q.y, -q.z]) 
        else: return q

    def inertializeJointTransition(self, sourceRot, sourceAngularVel,
                                    targetRot, targetAngularVel,
                                    index): # offsetRot, offsetAngularvel
        self.offsetRotations[index] = (self.abs((targetRot.to_matrix().inverted() @ (sourceRot.to_matrix() @ self.offsetRotations[index].to_matrix())).to_quaternion())).normalized()
        self.offsetAngularVelocities[index] = (sourceAngularVel + self.offsetAngularVelocities[index]) - targetAngularVel

    def inertializeJointTransition4Hips(self, source, sourceVel,
                                        target, targetVel,
                                        offset, offsetVel): # offset, offsetVel
        self.offsetHips = (source + offset) -target
        self.offsetHipsVelocity = (sourceVel + offsetVel) - targetVel

    # <summary>
    # Updates the inertialization decaying the offset and applying it to the target pose
    # </summary>
    def inertializeJointUpdate(self, targetRot, targetAngularVel,
                                halfLife, deltaTime,
                                index): 
        # offsetRot = offsetRotations, offsetAngularVel = offsetAngularVelocities, newRot = inertializedRotations, newAngularVel = inertializedAngularVelocities
        self.decaySpringDamperImplicit(index, halfLife, deltaTime)
        self.inertializedRotations[index] = (targetRot.to_matrix() @ self.offsetRotations[index].to_matrix()).to_quaternion()
        self.inertializedAngularVelocities[index] = targetAngularVel @ self.offsetAngularVelocities[index]
        
    def inertializeJointUpdate4Hips(self, target, targetVel,
                                    halfLife, deltaTime,
                                    offset, offsetVel,
                                    newValue, newVel):
        self.decaySpringDamperImplicit4Hips(offset, offsetVel, halfLife, deltaTime)
        self.inertializedHips = target + offset
        self.inertializedHipsVelocity = targetVel + offsetVel

    

    # spring damper
    # <summary>
    # Special type of SpringDamperImplicit when the desired rotation is the identity
    # </summary>
    def decaySpringDamperImplicit(self, index, halfLife, deltaTime):
        rot = self.offsetRotations[index]
        angularVel = self.offsetAngularVelocities[index]
        y = self.halfLifeToDamping(halfLife) / 2.0; # this could be precomputed
        j0 = self.quaternionToScaledAngleAxis(rot)
        j1 = angularVel + j0 * y
        eyedt = self.fastNEgeExp(y * deltaTime) # this could be precomputed if several agents use it the same frame

        self.offsetRotations[index] = self.quaternionFromScaledAngleAxis(eyedt * j0 + j1 * deltaTime)
        self.offsetAngularVelocities[index] = (eyedt * (angularVel - j1 * y * deltaTime))
    
    # <summary>
    # Special type of SpringDamperImplicit when the desired position is 0
    # </summary>
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
        
        
        