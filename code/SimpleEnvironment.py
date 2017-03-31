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
        for idx in range(int(self.discrete_env.num_cells[2])):
            self.actions[idx] = []
            grid_coordinate[2] = idx
            start_config = self.discrete_env.GridCoordToConfiguration(grid_coordinate)

            # TODO: Here you will construct a set of actions
            #  to be used during the planning process

            #create a list named control to store a set controls
            controls = []
            c1 = Control(1.0,1.0,0.1)
            c2 = Control(1.0,-1.0,0.1)
            c3 = Control(-1.0,-1.0,0.1)
            c4 = Control(-1.0,1.0,0.1)
            controls.append(c1)
            controls.append(c2)
            controls.append(c3)
            controls.append(c4)

            #create coorespondance footprint of each control
            footprints = []
            f1 = GenerateFootprintFromControl(start_config, c1, 0.01)
            f2 = GenerateFootprintFromControl(start_config, c2, 0.01)
            f3 = GenerateFootprintFromControl(start_config, c3, 0.01)
            f4 = GenerateFootprintFromControl(start_config, c4, 0.01)
            footprints.append(f1)
            footprints.append(f2)
            footprints.append(f3)
            footprints.append(f4)

            #create a set of actions
            a1 = Action(controls[0],footprints[0])
            a2 = Action(controls[1],footprints[1])
            a3 = Action(controls[2],footprints[2])
            a4 = Action(controls[3],footprints[3])

            self.actions[idx].append(a1)
            self.actions[idx].append(a2)
            self.actions[idx].append(a3)
            self.actions[idx].append(a4)
            

    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids and controls that represent the neighboring
        #  nodes
        config = self.discrete_env.NodeIdToConfiguration(node_id)
        avail_actions = self.actions[config[2]]

        for i in range(len(avail_actions)):
        	if not self.RobotIsInCollisionAt(avail_actions[i].footprint[-1]):
        		final_config = [config[0] + avail_actions[i].footprint[-1][0], config[1] + avail_actions[i].footprint[-1][1], config[2]]
	        	#successor_id  = self.discrete_env.ConfigurationToNodeId(avail_actions[i].footprint[-1])
	        	successor_id  = self.discrete_env.ConfigurationToNodeId(final_config)
	        	successor_control = [avail_actions[i].control]
	        	successors.append([successor_id, successor_control])

        #successors = [successors[item] for item in successors 
         #               if not self.RobotIsInCollisionAt(item[1])]
        return successors


  	def RobotIsInCollisionAt(self, point=None):
        """
        Call self.RobotIsInCollisionAt() to check collision in current state
             self.RobotIsInCollisionAt(np2darray) to check at another point
        """

        # Point should be a 2D np array with the configuration.
        # Leave empty if checking collision at current point
        if point is None:
            return self.robot.GetEnv().CheckCollision(self.robot)

        # If checking collision in point other than current state, move robot
        #  to that point, check collision, then move it back.
        current_state = self.robot.GetTransform()

        check_state = np.copy(current_state)
        check_state[:2,3] = point
        self.robot.SetTransform(check_state)

        in_collision = self.robot.GetEnv().CheckCollision(self.robot)
        self.robot.SetTransform(current_state)  # move robot back to current state
        return in_collision


    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        
        
        return cost

