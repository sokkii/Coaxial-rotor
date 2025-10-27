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
)

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
GRAPHICS_DIR = os.path.join(CURRENT_DIR, "..", "graphics")

motor_graphics = FromSTLfile(os.path.join(GRAPHICS_DIR, "Motor.stl"), color=color.black)

flywheel_graphics = FromSTLfile(
    os.path.join(GRAPHICS_DIR, "Flywheel.stl"), color=color.blue
)

rotation_matrix_180_x = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

# physical parameters
mass = 0.5
k = 80000  # stiffness of (all/both) springs
d = 100  # damping constant in N/(m/s)
omegaInitial = 50
g = [0, -9.81, 0]  # gravity
L = 1  # length
w = 0.1  # width
p0 = [0, 0, 0]  # origin of pendulum
pMid0 = np.array([0, L * 0.5, 0])  # center of mass, body0
pMid1 = np.array([0, L, 0])  # center of mass, body1
i0 = InertiaCylinder(5000, L, 0.5 * w, 0)
i1 = InertiaCylinder(5000, L, 0.5 * w, 2)
