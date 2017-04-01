import numpy, openravepy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

class Control(object):
    def __init__(self, omega_left, omega_right, duration):
        self.ul = omega_left
        self.ur = omega_right
        self.dt = duration

class Action(object):
    def __init__(self, control, footprint):
        self.control = control
        self.footprint = footprint

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.herb = herb
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5., -numpy.pi], [5., 5., numpy.pi]]
        lower_limits, upper_limits = self.boundary_limits
        self.discrete_env = DiscreteEnvironment(resolution, lower_limits, upper_limits)

        self.resolution = resolution
        self.ConstructActions()

    def GenerateFootprintFromControl(self, start_config, control, stepsize=0.01):

        # Extract the elements of the control
        ul = control.ul
        ur = control.ur
        dt = control.dt

        # Initialize the footprint
        config = start_config.copy()
        footprint = [numpy.array([0., 0., config[2]])]
        timecount = 0.0
        while timecount < dt:
            # Generate the velocities based on the forward model
            xdot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.cos(config[2])
            ydot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.sin(config[2])
            tdot = self.herb.wheel_radius * (ul - ur) / self.herb.wheel_distance
                
            # Feed forward the velocities
            if timecount + stepsize > dt:
                stepsize = dt - timecount
            config = config + stepsize*numpy.array([xdot, ydot, tdot])
            if config[2] > numpy.pi:
                config[2] -= 2.*numpy.pi
            if config[2] < -numpy.pi:
                config[2] += 2.*numpy.pi

            footprint_config = config.copy()
            footprint_config[:2] -= start_config[:2]
            footprint.append(footprint_config)

            timecount += stepsize
            
        # Add one more config that snaps the last point in the footprint to the center of the cell
        nid = self.discrete_env.ConfigurationToNodeId(config)
        snapped_config = self.discrete_env.NodeIdToConfiguration(nid)
        snapped_config[:2] -= start_config[:2]
        footprint.append(snapped_config)

        return footprint

    def PlotActionFootprints(self, idx):

        actions = self.actions[idx]
        fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        
        for action in actions:
            xpoints = [config[0] for config in action.footprint]
            ypoints = [config[1] for config in action.footprint]
            pl.plot(xpoints, ypoints, 'k')
                     
        pl.ion()
        pl.show()

        
    def ConstructActions(self):

        # Actions is a dictionary that maps orientation of the robot to
        #  an action set
        self.actions = dict()
              
        wc = [0., 0., 0.]
        grid_coordinate = self.discrete_env.ConfigurationToGridCoord(wc) 

        # Iterate through each possible starting orientation
        for idx in range(int(self.discrete_env.num_cells[2]+1)):
            self.actions[idx] = []
            grid_coordinate[2] = idx
            start_config = self.discrete_env.GridCoordToConfiguration(grid_coordinate)

            #create a list named control to store a set controls
            controls = [Control(1.0,1.0,.5), Control(1.0,-1.0,.5), Control(-1.0,-1.0,.5), Control(-1.0,1.0,.5)]
            for control in controls:
                self.actions[idx].append(Action(control, self.GenerateFootprintFromControl(start_config, control, 0.01))) 


    def GetSuccessors(self, node_id):

        successors = []

        config = self.discrete_env.NodeIdToConfiguration(node_id)
        coord = self.discrete_env.NodeIdToGridCoord(node_id)
        avail_actions = self.actions[coord[2]]

        for i in range(len(avail_actions)):
            for j in range(len(avail_actions[i].footprint)):
                true_footprint = []
                check = 0;
                if not self.RobotIsInCollisionAt(avail_actions[i].footprint[j]):
                    true_config = [config[0] + avail_actions[i].footprint[j][0], config[1] + avail_actions[i].footprint[j][1], avail_actions[i].footprint[j][2]]
                    true_footprint.append(true_config)
                else:
                    # go to next action
                    check = -1 
                    break
            if (check != -1):
                successor_control = avail_actions[i].control
                final_config = true_footprint[-1]
                successor_id  = self.discrete_env.ConfigurationToNodeId(final_config)
                succ_config = self.discrete_env.NodeIdToConfiguration(successor_id)
                successors.append([successor_id, Action(successor_control, numpy.array(true_footprint))])

        return successors


    def RobotIsInCollisionAt(self, point=None):
        """
        Call self.RobotIsInCollisionAt() to check collision in current state
             self.RobotIsInCollisionAt(np2darray) to check at another point
        """        
        lower_limits, upper_limits = self.boundary_limits

        if point is None:
            return self.robot.GetEnv().CheckCollision(self.robot)

        # If checking collision in point other than current state, move robot
        #  to that point, check collision, then move it back.
        current_state = self.robot.GetTransform()
        # print '-----------------------------------------'
        # print current_state
        check_state = numpy.copy(current_state)
        # print '-----------------------------------------'
        # print point
        check_state[:3, 3] = numpy.array([point[0], point[1], 0.0])
        check_state[:3, :3] = numpy.array([[numpy.cos(point[2]), -numpy.sin(point[2]),0.0],[numpy.sin(point[2]),numpy.cos(point[2]),0.0],[0.0,0.0,1.0]])
        
        self.robot.SetTransform(check_state) 
        in_collision = self.robot.GetEnv().CheckCollision(self.robot) and ((point < lower_limits).any() or (point > upper_limits).any())
        self.robot.SetTransform(current_state)  # move robot back to current state

        return in_collision


    def ComputeDistance(self, start_id, end_id):
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)
        return numpy.linalg.norm(numpy.array(start_config[:len(start_config)-1]) - numpy.array(end_config[:len(end_config)-1]))

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        return self.ComputeDistance(start_id,goal_id)

    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()

