import exudyn as exu
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
    Torque,
)
import exudyn.graphics as graphics
import numpy as np
from constants import *

# Initialize SystemContainer and MainSystem
SC = exu.SystemContainer()
mbs = SC.AddSystem()

# Ground and bodies
oGround = mbs.AddObject(ObjectGround(referencePosition=[0, 0, 0]))

[n0, b0] = AddRigidBody(
    mainSys=mbs,
    inertia=i0,
    nodeType=str(exu.NodeType.RotationRotationVector),
    position=pMid0,
    rotationMatrix=rotation_matrix_180_x,
    gravity=g,
    graphicsDataList=[motor_graphics],
)

[n1, b1] = AddRigidBody(
    mainSys=mbs,
    inertia=i1,
    nodeType=str(exu.NodeType.RotationRotationVector),
    position=pMid1,
    rotationMatrix=rotation_matrix_180_x,
    gravity=g,
    graphicsDataList=[flywheel_graphics],
)

# Markers
joint0_pos = [0, L, 0]
joint1_pos = [0, 0, 0]

markerGround0 = mbs.AddMarker(
    MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 0, 0])
)
markerBody0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[0, 0, 0]))

mbs.AddObject(
    GenericJoint(
        markerNumbers=[markerGround0, markerBody0],
        constrainedAxes=[1, 1, 1, 1, 1, 1],  # fully fixed
        visualization=VObjectJointGeneric(axesRadius=0.02, axesLength=0.1),
    )
)

markerMotor = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[0, 0, 0]))
markerFlywheel = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b1, localPosition=[0, 0, 0]))

mbs.AddObject(
    ObjectJointRevoluteZ(
        markerNumbers=[markerMotor, markerFlywheel],
        rotationMarker0=np.eye(3),
        rotationMarker1=np.eye(3),
        visualization=VObjectJointRevoluteZ(axisRadius=0.02, axisLength=0.15),
    )
)

coordMarker = mbs.AddMarker(
    MarkerNodeRotationCoordinate(
        nodeNumber=n1,  # flywheel node
        rotationCoordinate=2,  # 0=x, 1=y, 2=z (Euler parameter nodes use this convention)
    )
)

torque = [0, 0, 0.1]
mbs.AddLoad(Torque(markerNumber=markerFlywheel, loadVector=torque))
mbs.Assemble()
if False:
    mbs.systemData.Info()  # show detailed information
if False:
    mbs.DrawSystemGraph(useItemTypes=True)  # draw nice graph of system

simulationSettings = (
    exu.SimulationSettings()
)  # takes currently set values or default values

tEnd = 4  # simulation time
h = 1e-2  # step size
simulationSettings.timeIntegration.numberOfSteps = int(tEnd / h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
# simulationSettings.timeIntegration.simulateInRealtime = True
simulationSettings.solutionSettings.solutionWritePeriod = 0.005  # store every 5 ms

SC.visualizationSettings.window.renderWindowSize = [1600, 1200]
SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.general.autoFitScene = False

SC.visualizationSettings.nodes.drawNodesAsPoint = False
SC.visualizationSettings.nodes.showBasis = True

# uncomment to start visualization during simulation
# SC.renderer.Start()
# if 'renderState' in exu.sys: #reload old view
#     SC.renderer.SetState(exu.sys['renderState'])

# SC.renderer.DoIdleTasks() #stop before simulating

mbs.SolveDynamic(
    simulationSettings=simulationSettings,
    solverType=exu.DynamicSolverType.TrapezoidalIndex2,
)

# SC.renderer.DoIdleTasks() #stop before closing
# SC.renderer.Stop() #safely close rendering window!

# start post processing
mbs.SolutionViewer()
# mbs.PlotSensor(sensorNumbers=[sens1],components=[0],closeAll=True)
# mbs.PlotSensor(sensorNumbers=[sens2],components=[0],closeAll=True)
# if False:
#    #plot sensor sens1, y-component [1]
