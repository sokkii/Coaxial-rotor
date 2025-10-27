#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  3D rigid body tutorial with 2 bodies and revolute joints, using Marker-style approach
#
# Author:   Johannes Gerstmayr
# Date:     2021-08-05
# Date:     2024-06-04 (updated to MainSystem Python extensions)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import ObjectGround, InertiaCylinder, MarkerBodyRigid, GenericJoint, \
                             VObjectJointGeneric, SensorBody, NodePoint, MassPoint, MarkerBodyPosition, CartesianSpringDamper
#to be sure to have all items and functions imported, just do:
#from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++
#physical parameters
mass = 0.5
k = 80000                 #stiffness of (all/both) springs in rotor in N/m
d = 10       #damping constant in N/(m/s)
g =     [0,-9.81,0] #gravity
L = 1               #length
w = 0.1             #width
bodyDim=[L,w,w] #body dimensions
p0 =    [0,0,0]     #origin of pendulum
pMid0 = np.array([0,L*0.5,0]) #center of mass, body0
pMid1 = np.array([0,0.5*L,0])

#ground body
oGround = mbs.CreateGround()

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++
#first link:
#create inertia paramters (mass, center of mass (COM) and inertia tensor at reference point)
iCylinder0 = InertiaCylinder(5000, L, 0.5*w, 0)
#iCube0 = iCube0.Translated([-0.25*L,0,0]) #transform COM, COM not at reference point!

#graphics for body
graphicsBody0 = graphics.RigidLink(p0=[0,-0.5*L,0],p1=[0,0.5*L,0],
                                     axis0=[0,1,0], axis1=[0,1,0], radius=[0.5*w,0.5*w],
                                     thickness = w, width = [1.2*w,1.2*w], color=graphics.color.red)
graphicsCOM0 = graphics.Basis(origin=iCylinder0.com, length=2*w)

#create rigid body; we could use other formulation, e.g., by selecting nodeType = exu.NodeType.RotationRotationVector
b0=mbs.CreateRigidBody(inertia = iCylinder0, #includes COM
                       referencePosition = pMid0,
                       gravity = g,
                       graphicsDataList = [graphicsCOM0, graphicsBody0])


#%%++++++++++++++++++++++++++
#revolute joint (free z-axis)

#markers for ground and rigid body (not needed for option 3):
markerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
markerBody0J0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[0,-0.5*L,0]))

# revolute joint option 1:
#marker for ground (=fixed):
groundMarker0=mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0,0,0]))

CylinderMarker0 =mbs.AddMarker(MarkerBodyPosition(bodyNumber=b0, localPosition=[0,-L/2,0]))

#++++++++++++++++++++++++++++++++++++
mbs.AddObject(CartesianSpringDamper(markerNumbers=[groundMarker0, CylinderMarker0],
                                    stiffness=[k,k,k], damping=[d, d, d]))


#%%++++++++++++++++++++++++++
#second link:
graphicsBody1 = graphics.RigidLink(p0=[0,0.5*L,-0.5*L],p1=[0,0.5*L,0.5*L],
                                     axis0=[0,0,1], axis1=[0,0,1], radius=[0.05,0.05],
                                     thickness = 0.1, width = [0.12,0.12], color=graphics.color.lightgreen)

#iCylinder0 = InertiaCylinder(5000, L, w, 0)
iCylinder1 = InertiaCylinder(5000, L, 0.5*w, 2)

b1=mbs.CreateRigidBody(inertia = iCylinder1,
                            referencePosition = pMid1,
                            gravity = g,
                            graphicsDataList = [graphicsBody1])

#revolute joint (free x-axis)
# #alternative with GenericJoint:
# #markers for rigid body:
markerBody0J1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[ 0,0.5*L,0]))
markerBody1J0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b1, localPosition=[0,0.5*L,0]))
mbs.AddObject(GenericJoint(markerNumbers=[markerBody0J1, markerBody1J0],
                            constrainedAxes=[1,1,1,1,0,1],
                            visualization=VObjectJointGeneric(axesRadius=0.6*w, axesLength=1.4*w)))

mbs.CreateTorque(bodyNumber=b1,
                 loadVector=[0,5,0]
                 )

#position sensor at tip of body1
sens1=mbs.AddSensor(SensorBody(bodyNumber=b1, localPosition=[0,0.5*L,0.5*L],
                               fileName='sensorPos.txt',
                               outputVariableType = exu.OutputVariableType.Position))
sens2=mbs.AddSensor(SensorBody(bodyNumber=b1, localPosition=[0,0.5*L,0.5*L],
                               fileName='sensorAccel.txt',
                               outputVariableType = exu.OutputVariableType.Acceleration))

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++
#assemble system before solving
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
mbs.PlotSensor(sensorNumbers=[sens1],components=[0],closeAll=True)
