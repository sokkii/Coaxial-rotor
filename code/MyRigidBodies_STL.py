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
    CartesianSpringDamper,
    VCartesianSpringDamper,
    Force,
    AngularVelocity2EulerParameters_t,
    
)
import exudyn.graphics as graphics
import numpy as np
from constants_STL import *

# Initialize SystemContainer and MainSystem
SC = exu.SystemContainer()
mbs = SC.AddSystem()

# Ground and bodies
oGround = mbs.AddObject(ObjectGround(referencePosition=[0, 0, 0]))

[n0, b0] = AddRigidBody(
    mainSys=mbs,
    inertia=iMotor,
    nodeType=str(exu.NodeType.RotationRotationVector),
    position=[0, 0.015733e-3, 45.764021e-3],
    rotationMatrix=rotation_matrix_180_x,
    gravity=g,
    graphicsDataList=[motor_graphics, graphicsCOM0],
)

[n1, b1] = AddRigidBody(
    mainSys=mbs,
    inertia=iFlywheel,
    nodeType=str(exu.NodeType.RotationEulerParameters),
    position=[0.002729e-3, 0, 82.535793e-3],
    rotationMatrix=rotation_matrix_180_x,
    gravity=g,
    graphicsDataList=[flywheel_graphics, graphicsCOM1],
)

#****** Markers ******

markerGround0 = mbs.AddMarker(
    MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 0, -123/1000])
)
markerBody0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[0, 0, 165/1000]))

# запрет поворота мотора относительно земли (ROTX, ROTY, ROTZ fixed) 
mbs.AddObject(
    GenericJoint(
        markerNumbers=[markerGround0, markerBody0],
        constrainedAxes=[0, 0, 0, 1, 1, 1],  # rotation fixed
        visualization=VObjectJointGeneric(axesRadius=0.02, axesLength=0.8),
    )
)
# ограничение линейных перемещений UX, UY, UZ пружинами
mbs.AddObject(CartesianSpringDamper(markerNumbers = [markerGround0, markerBody0],
                                    stiffness = [k,k,k],
                                    damping = [k*0.1,k*0.1,k*0.1], offset = [0,0,0],
                                    visualization=VCartesianSpringDamper(drawSize=0.05)))

markerMotor = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[0, 0, -30e-3]))
markerFlywheel = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b1, localPosition=[0, 0, 10e-3]))

mbs.AddObject(
    ObjectJointRevoluteZ(
        markerNumbers=[markerMotor, markerFlywheel],
        rotationMarker0=np.eye(3),
        rotationMarker1=np.eye(3),
        visualization=VObjectJointRevoluteZ(axisRadius=0.01, axisLength=0.025),
    )
)

coordMarker = mbs.AddMarker(
    MarkerNodeRotationCoordinate(
        nodeNumber=n1,  # flywheel node
        rotationCoordinate=2,  # 0=x, 1=y, 2=z (Euler parameter nodes use this convention)
    )
)

markerFb1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b1, localPosition=[0, 100e-3, 10e-3]))
force = [0, 0, 10]
mbs.AddLoad(Force(markerNumber=markerFb1, loadVector=force, bodyFixed=True))

#torque = [0, 0, 0.05]
#mbs.AddLoad(Torque(markerNumber=markerFlywheel, loadVector=torque))

# Const velocity for node 1  
currentCoords = mbs.GetNodeParameter(n1, 'referenceCoordinates')  

# производные параметров Эйлера  
ep_t = AngularVelocity2EulerParameters_t([0, omegaInitial, 0], currentCoords[3:7])  

# Установить новые скорости  
currentVelocities = mbs.GetNodeParameter(n1, 'initialVelocities')  
currentVelocities[3:7] = ep_t  
mbs.SetNodeParameter(n1, 'initialVelocities', currentVelocities)


mbs.Assemble()
if False:
    mbs.systemData.Info()  # show detailed information
if False:
    mbs.DrawSystemGraph(useItemTypes=True)  # draw nice graph of system

simulationSettings = (
    exu.SimulationSettings()
)  # takes currently set values or default values

tEnd = 3  # simulation time
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
