#!/usr/bin/env
import numpy, openravepy

from SimpleRobot import SimpleRobot
from SimpleEnvironment import SimpleEnvironment
from AStarPlanner import AStarPlanner

if __name__ == "__main__":

    # Create an environment
    env = openravepy.Environment()
    env.SetViewer('qtcoin')
    env.GetViewer().SetName('Homework 4 Viewer')

    # Load HERB into it
    robot = env.ReadRobotXMLFile('models/robots/herb2_padded.robot.xml')
    env.Add(robot)

    right_relaxed = [ 5.65, -1.76, -0.26,  1.96, -1.15 , 0.87, -1.43 ]
    left_relaxed = [ 0.64, -1.76,  0.26,  1.96,  1.16,  0.87,  1.43 ]
    right_manip = robot.GetManipulator('right_wam')
    robot.SetActiveDOFs(right_manip.GetArmIndices())
    robot.SetActiveDOFValues(right_relaxed)
        
    left_manip = robot.GetManipulator('left_wam')
    robot.SetActiveDOFs(left_manip.GetArmIndices())
    robot.SetActiveDOFValues(left_relaxed)

    robot.controller = openravepy.RaveCreateController(robot.GetEnv(), 'IdealController')
     
    # add a table and move the robot into place
    table = env.ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
    env.Add(table)
    
    table_pose = numpy.array([[ 0, 0, -1, 0.7], 
                              [-1, 0,  0, 0], 
                              [ 0, 1,  0, 0], 
                              [ 0, 0,  0, 1]])
    table.SetTransform(table_pose)

    resolution = [0.1, 0.1, numpy.pi/4.]
    herb_base = SimpleRobot(env, robot)
    base_env = SimpleEnvironment(herb_base, resolution)

    # start_config = numpy.array([0, 0, 0])
    # goal_config = numpy.array([2, 3, 0])

    # # added by Sumit to convert start configuration to one accessible by node id
    # sid = base_env.discrete_env.ConfigurationToNodeId(start_config)
    # sconfig = base_env.discrete_env.NodeIdToConfiguration(sid)
    # herb_base.SetCurrentConfiguration(sconfig)

    #raw_input('Press enter to continue')
    raw_input('Move robot to start config and press enter')
    sid = base_env.discrete_env.ConfigurationToNodeId(herb_base.GetCurrentConfiguration())
    start_config = base_env.discrete_env.NodeIdToConfiguration(sid)
    herb_base.SetCurrentConfiguration(start_config)

    tstart = robot.GetTransform()
    hstart = openravepy.misc.DrawAxes(env, tstart)
    hstart.SetShow(True)

    # # added by Sumit to convert goal configuration to one accessible by node id
    # gid = base_env.discrete_env.ConfigurationToNodeId(goal_config)
    # gconfig = base_env.discrete_env.NodeIdToConfiguration(gid)
    # herb_base.SetCurrentConfiguration(gconfig)
    

    raw_input('Move robot to goal config and press enter')
    gid = base_env.discrete_env.ConfigurationToNodeId(herb_base.GetCurrentConfiguration())
    goal_config = base_env.discrete_env.NodeIdToConfiguration(gid)
    herb_base.SetCurrentConfiguration(goal_config)

    tgoal = robot.GetTransform()
    hgoal = openravepy.misc.DrawAxes(env, tgoal)
    hgoal.SetShow(True)

    # # changed by Sumit to accept sconfig
    # herb_base.SetCurrentConfiguration(sconfig)
    herb_base.SetCurrentConfiguration(start_config)

    planner = AStarPlanner(base_env, visualize=False)

    # # changed by Sumit to accept sconfig, gconfig
    # plan = planner.Plan(sconfig, gconfig)
    plan = planner.Plan(start_config, goal_config)
    traj = herb_base.ConvertPlanToTrajectory(plan)

    raw_input('Press any key to play trajectory')
    herb_base.ExecuteTrajectory(traj)

    raw_input('Press any key to quit.')

    
    
    
