import os
from exudyn.graphics import *
from exudyn.utilities import (
    ObjectGround,
    InertiaCylinder,
    MarkerBodyRigid,
    GenericJoint,
    VObjectJointGeneric,
    SensorBody,
    NodePoint,
    MassPoint,
    MarkerBodyPosition,
    CartesianSpringDamper,
    MarkerNodeRotationCoordinate,
    MarkerNodeCoordinate,
    CoordinateConstraint,
    AddRigidBody,
    ObjectJointRevoluteZ,
    VObjectJointRevoluteZ,
    RigidBodyInertia,
)
import numpy as np

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
GRAPHICS_DIR = os.path.join(CURRENT_DIR, "..", "graphics")

motor_graphics = FromSTLfile(
    os.path.join(CURRENT_DIR, "Motor.stl"),
    color=color.lightgrey,
    scale=0.001)

flywheel_graphics = FromSTLfile(
    os.path.join(CURRENT_DIR, "Flywheel.stl"),
    color=color.blue,
    scale=0.001
)


rotation_matrix_180_x = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

# physical parameters (из компаса)
m_Flywheel = 1.819 # Flywheel's mass
m_Motor = 2.07     # Motor's mass

inertiaTensorFlywheel = np.array([
    [2806684.363152 / 1e9, 0, 0],
    [0, 2916638.411140 / 1e9, 286.226052 / 1e9],
    [0, 286.226052 / 1e9, 5535008.478567 / 1e9]
])

inertiaTensorMotor = np.array([
    [41337972 / 1e9, 0, -198/1e9],
    [0, 41337911 / 1e9, 0],
    [-198/1e9, 0, 2203427 / 1e9]
])
# это надо поправлять
Com0 = [0, 0.015733e-3, 80.764021e-3] # COM Motor global
Com1 = [0.002729e-3, 0, 8.535793e-3] # COM Flywheel global


iFlywheel = RigidBodyInertia(mass=m_Flywheel,
                             inertiaTensor=inertiaTensorFlywheel,
                             com=Com0)
iMotor = RigidBodyInertia(mass=m_Motor, 
                          inertiaTensor=inertiaTensorMotor,
                          com=Com1)

graphicsCOM0 = Basis(iFlywheel.com, length=0.1)

graphicsCOM1 = Basis(iMotor.com, length=0.1)

k = 50000  # stiffness of (all/both) springs
d = 100  # damping constant in N/(m/s)
omegaInitial = 10
g = [0, 0, -9.81]  # gravity
L = 1  # length
w = 0.1  # width
p0 = [0, 0, 0]  # origin of pendulum
#pMid0 = np.array([0, L * 0.5, 0])  # center of mass, body0
#pMid1 = np.array([0, L, 0])  # center of mass, body1
#i0 = InertiaCylinder(5000, L, 0.5 * w, 0)
#i1 = InertiaCylinder(5000, L, 0.5 * w, 2)
