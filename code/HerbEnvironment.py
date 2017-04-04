import numpy
import time

class HerbEnvironment(object):
    
    def __init__(self, herb):
        self.herb = herb
        self.robot = herb.robot

        # add a table and move the robot into place
        #table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        #self.robot.GetEnv().Add(table)

        #table_pose = numpy.array([[ 0, 0, -1, 0.6], 
        #                          [-1, 0,  0, 0], 
        #                          [ 0, 1,  0, 0], 
        #                          [ 0, 0,  0, 1]])
        #table.SetTransform(table_pose)

        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
        
        # goal sampling probability
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        

    def GenerateRandomConfiguration(self):
        config = [0] * len(self.robot.GetActiveDOFIndices())

        #
        # TODO: Generate and return a random configuration
        #
	# Brad: Generate and return a random, collision-free, configuration
	lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
	collisionFlag = True
	while collisionFlag is True:
		for dof in range(len(self.robot.GetActiveDOFIndices())):
			config[dof] = lower_limits[dof] + (upper_limits[dof] - lower_limits[dof]) * numpy.random.random_sample()
		#CHECK: Lock environment?
		self.robot.SetActiveDOFValues(numpy.array(config))
		if(self.robot.GetEnv().CheckCollision(self.robot)) is False:
			if(self.robot.CheckSelfCollision()) is False:
				collisionFlag = False
			#else:
				#print "Self Collision detected in random configuration"
		#else:
			#print "Collision Detected in random configuration"	
        return numpy.array(config)

    
    def ComputeDistance(self, start_config, end_config):
        
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        # 
	# Brad: Compute the distance between two configurations as the L2 norm
	distance = numpy.linalg.norm(end_config - start_config)
        return distance


    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
	# Brad: Extend from start to end configuration and return sooner if collision or limits exceeded
	lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
	resolution = 100
	
	#Calculate incremental configuration changes
	config_inc = [0] * len(self.robot.GetActiveDOFIndices())
	for dof in range(len(self.robot.GetActiveDOFIndices())):
		config_inc[dof] = (end_config[dof] - start_config[dof]) / float(resolution)

	#Set initial config state to None to return if start_config violates conditions
	config = None
 
	#Move from start_config to end_config
	for step in range(resolution+1):
		prev_config = config
		config = [0] * len(self.robot.GetActiveDOFIndices())
		for dof in range(len(self.robot.GetActiveDOFIndices())):
			#Calculate new config
			config[dof] = start_config[dof] + config_inc[dof]*float(step)

			#Check joint limits
			if config[dof] > upper_limits[dof]:
				print "Upper joint limit exceeded"
				return prev_config
			if config[dof] < lower_limits[dof]:
				print "Lower joint limit exceeded"
				return prev_config

		#Set config and check for collision
		#CHECK: Lock environment?
		self.robot.SetActiveDOFValues(numpy.array(config))
		if(self.robot.GetEnv().CheckCollision(self.robot)) is True:
			#print "Collision Detected in extend"
			return prev_config
		if(self.robot.CheckSelfCollision()) is True:
			#print "Self Collision Detected in extend"
			return prev_config
        return end_config
        
    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        #Brad
	#print "Start shortening"
	prev_path = list(path)
	prev_len = self.ComputePathLength(path)
	print "Current path length is %f" % prev_len
	elapsed_time = 0
	start_time = time.time();
	while elapsed_time < timeout:
		end_time = time.time();
		elapsed_time = end_time - start_time
		#print path_short
		if len(prev_path) < 3:
			print "Shortened path length is %f" % self.ComputePathLength(prev_path)
			return prev_path
		else:
			curr_path = list(path)
			for attempt in range(len(path)*len(path)):
				#print attempt
				init_idx = numpy.random.randint(0,len(curr_path)-2)
				end_idx = numpy.random.randint(init_idx+2,len(curr_path))
				init_ms = curr_path[init_idx]
				end_ms = curr_path[end_idx]
				result_ms = self.Extend(init_ms,end_ms)
				if numpy.array_equal(end_ms,result_ms):
					for elem in range(init_idx+1,end_idx):
						del curr_path[init_idx+1]
			curr_len = self.ComputePathLength(curr_path)
			if curr_len < prev_len:
				prev_path = curr_path
				prev_len = curr_len
	print "Shortened path length is %f" % self.ComputePathLength(prev_path)
        return prev_path
	
    def ComputePathLength(self, path):
	#Helper function to compute path length
	length = 0
	for milestone in range(1,len(path)):
		#print path[milestone-1], path[milestone]
		distance = self.ComputeDistance(numpy.array(path[milestone-1]),numpy.array(path[milestone]))
		#print distance
		length += distance
	return length
		
