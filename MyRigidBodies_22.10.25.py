# -*- coding: utf-8 -*-
"""
Created on Wed Oct 22 11:33:12 2025

@author: admin
"""

import exudyn as exu
from exudyn.utilities import ObjectGround, InertiaCylinder, MarkerBodyRigid, GenericJoint, \
                             VObjectJointGeneric, SensorBody, NodePoint, MassPoint, MarkerBodyPosition, CartesianSpringDamper, \
                             MarkerNodeRotationCoordinate, MarkerNodeCoordinate, CoordinateConstraint, AddRigidBody
import exudyn.graphics as graphics
import numpy as np



# Initialize SystemContainer and MainSystem
SC = exu.SystemContainer()
mbs = SC.AddSystem()

#physical parameters
mass = 0.5
k = 80000                 #stiffness of (all/both) springs 
d = 100       #damping constant in N/(m/s)
omegaInitial = 50
g =     [0,-9.81,0] #gravity
L = 1               #length
w = 0.1             #width
p0 =    [0,0,0]     #origin of pendulum
pMid0 = np.array([0,L*0.5,0]) #center of mass, body0
pMid1 = np.array([0,L,0]) #center of mass, body1


# Ground and bodies
oGround = mbs.CreateGround()

i0 = InertiaCylinder(5000, L, 0.5*w, 0)
i1 = InertiaCylinder(5000, L, 0.5*w, 2)

# first link
graphicsBody0 = graphics.RigidLink(p0=[0,0,0],p1=[0,L,0],
                                     axis0=[0,1,0], axis1=[0,1,0], radius=[0.5*w,0.5*w],
                                     thickness = w, width = [1.2*w,1.2*w], color=graphics.color.red)
graphicsCOM0 = graphics.Basis(origin=i0.com, length=2*w)

[n0, b0] = AddRigidBody(
    mainSys=mbs,
    inertia=i0,
    nodeType=str(exu.NodeType.RotationEulerParameters),
    position=pMid0,
    rotationMatrix=np.eye(3),
    gravity=g,
    graphicsDataList=[graphicsCOM0, graphicsBody0]
)

# second link
graphicsBody1 = graphics.RigidLink(p0=[0, L,-0.5*L],p1=[0, L, 0.5*L],
                                     axis0=[0,0,1], axis1=[0,0,1], radius=[0.5*w,0.5*w],
                                     thickness = w, width = [1.2*w,1.2*w], color=graphics.color.lightgreen)
graphicsCOM1 = graphics.Basis(origin=i1.com, length=2*w)

[n1, b1] = AddRigidBody(
    mainSys=mbs,
    inertia=i1,
    nodeType=str(exu.NodeType.RotationEulerParameters),
    position=pMid1,
    rotationMatrix=np.eye(3),
    gravity=g,
    graphicsDataList=[graphicsCOM1, graphicsBody1]
)

# Markers
joint0_pos = [0, L, 0]
joint1_pos = [0, 0, 0]
 

link0_marker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0, coordinate=2))  # Z-position for prismatic joint
link1_marker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n1, coordinate=6))  # Rotation Z for revolute joint 1

markerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 0, 0]))
markerBody0J0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=joint0_pos))

# Joints
jointRevolute1 = mbs.CreateRevoluteJoint(bodyNumbers=[b0, b1], position=joint1_pos,
                                         axis=[0, 1, 0], axisRadius=0.2*w, axisLength=1*w)



mbs.Assemble()
if False:
    mbs.systemData.Info() #show detailed information
if False:
    mbs.DrawSystemGraph(useItemTypes=True) #draw nice graph of system

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

tEnd = 4 #simulation time
h = 1e-2 #step size
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
#simulationSettings.timeIntegration.simulateInRealtime = True
simulationSettings.solutionSettings.solutionWritePeriod = 0.005 #store every 5 ms

SC.visualizationSettings.window.renderWindowSize=[1600,1200]
SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.general.autoFitScene = False

SC.visualizationSettings.nodes.drawNodesAsPoint=False
SC.visualizationSettings.nodes.showBasis=True

# uncomment to start visualization during simulation
# SC.renderer.Start()
# if 'renderState' in exu.sys: #reload old view
#     SC.renderer.SetState(exu.sys['renderState'])

#SC.renderer.DoIdleTasks() #stop before simulating

mbs.SolveDynamic(simulationSettings = simulationSettings,
                 solverType=exu.DynamicSolverType.TrapezoidalIndex2)

# SC.renderer.DoIdleTasks() #stop before closing
# SC.renderer.Stop() #safely close rendering window!

#start post processing
mbs.SolutionViewer()
#mbs.PlotSensor(sensorNumbers=[sens1],components=[0],closeAll=True)
#mbs.PlotSensor(sensorNumbers=[sens2],components=[0],closeAll=True)
#if False:
#    #plot sensor sens1, y-component [1]








