import logging, numpy, openravepy, time, math
from openravepy.databases import inversereachability
from openravepy import IkFilterOptions
#from openravepy.databases import inversereachability


class GraspPlanner(object):

    def __init__(self, robot, base_planner, arm_planner):
        self.robot = robot
        self.base_planner = base_planner
        self.arm_planner = arm_planner

            
    def GetBasePoseForObjectGrasp(self, obj):

        # Load grasp database
        self.gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)
        if not self.gmodel.load():
            self.gmodel.autogenerate()

        base_pose = None
        grasp_config = None
        
        ###################################################################
        # TODO: Here you will fill in the function to compute
        #  a base pose and associated grasp config for the 
        #  grasping the bottle
        ###################################################################
        #get the ordered valid grasp from homework1
        self.order_grasps()
        #choose the first valid grasp
        validgrasp = self.grasps_ordered[0]
        # get the grasp transform
        Tgrasp = self.gmodel.getGlobalGraspTransform(validgrasp,collisionfree=True) 

        # load inverserechability database
        self.irmodel = openravepy.databases.inversereachability.InverseReachabilityModel(robot=self.robot)
        starttime = time.time()
        print 'loading irmodel'
        if not self.irmodel.load():
            print 'do you want to generate irmodel for your robot? it might take several hours'
            print 'or you can go to http://people.csail.mit.edu/liuhuan/pr2/openrave/openrave_database/ to get the database for PR2'
            input = raw_input('[Y/n]')
            if input == 'y' or input == 'Y' or input == '\n' or input == '':
                #class IrmodelOption:
                self.irmodel.autogenerate()
                self.irmodel.load()
            else:
                raise ValueError('')
        
        print 'time to load inverse-reachability model: %fs'%(time.time()-starttime)

        densityfn,samplerfn,bounds = self.irmodel.computeBaseDistribution(Tgrasp)

		#find the valid pose and joint states
		# initialize sampling parameters
        goals = []
        numfailures = 0
        starttime = time.time()
        timeout = inf
        with self.robot:
            while len(goals) < 3:
                if time.time()-starttime > timeout:
                    break
                poses,jointstate = samplerfn(N-len(goals))
                for pose in poses:
                    self.robot.SetTransform(pose)
                    self.robot.SetDOFValues(*jointstate)
                    # validate that base is not in collision
                    if not self.manip.CheckIndependentCollision(CollisionReport()):
                        q = self.manip.FindIKSolution(Tgrasp,filteroptions=IkFilterOptions.CheckEnvCollisions)
                        if q is not None:
                            values = self.robot.GetDOFValues()
                            values[self.manip.GetArmIndices()] = q
                            goals.append((Tgrasp,pose,values))
                        elif self.manip.FindIKSolution(Tgrasp,0) is None:
                            numfailures += 1
        # To do still
        #base_pose = goals[0][1]
        #grasp_config = goals[0][2]
        for i,goal in enumerate(goals):
        	grasp_with_pose,pose,values =goal
        	self.robot.SetTransform(pose)
        	self.robot.SetJointValues(values)
        trans_pose = self.robot.GetTransform()
        angle_pose = openravepy.axisAngleFromRotationMatrix(trans_pose)
        pose = [trans_pose[0,3],trans_pose[1,3],angle_pose[2]]
        base_pose = np.array(pose)

        grasp_config = q

        import IPython
        IPython.embed()

        return base_pose, grasp_config

    def PlanToGrasp(self, obj):

        # Next select a pose for the base and an associated ik for the arm
        base_pose, grasp_config = self.GetBasePoseForObjectGrasp(obj)

        if base_pose is None or grasp_config is None:
            print 'Failed to find solution'
            exit()

        # Now plan to the base pose
        start_pose = numpy.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
        base_plan = self.base_planner.Plan(start_pose, base_pose)
        base_traj = self.base_planner.planning_env.herb.ConvertPlanToTrajectory(base_plan)

        print 'Executing base trajectory'
        self.base_planner.planning_env.herb.ExecuteTrajectory(base_traj)

        # Now plan the arm to the grasp configuration
        start_config = numpy.array(self.arm_planner.planning_env.herb.GetCurrentConfiguration())
        arm_plan = self.arm_planner.Plan(start_config, grasp_config)
        arm_traj = self.arm_planner.planning_env.herb.ConvertPlanToTrajectory(arm_plan)

        print 'Executing arm trajectory'
        self.arm_planner.planning_env.herb.ExecuteTrajectory(arm_traj)

        # Grasp the bottle
        task_manipulation = openravepy.interfaces.TaskManipulation(self.robot)
        task_manipultion.CloseFingers()



      #Code copied from hw1(the following two functions) 

      # order the grasps - call eval grasp on each, set the 'performance' index, and sort
    def order_grasps(self):
        self.grasps_ordered = self.gmodel.grasps.copy() #you should change the order of self.grasps_ordered
        for grasp in self.grasps_ordered:
            grasp[self.gmodel.graspindices.get('performance')] = self.eval_grasp(grasp)
    
    	# sort!
        order = np.argsort(self.grasps_ordered[:,self.gmodel.graspindices.get('performance')[0]])
        order = order[::-1]
        self.grasps_ordered = self.grasps_ordered[order]

    def eval_grasp(self, grasp):
      	with self.robot:
      #contacts is a 2d array, where contacts[i,0-2] are the positions of contact i and contacts[i,3-5] is the direction
	        try:
	          contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=False)

	          obj_position = self.gmodel.target.GetTransform()[0:3,3]   
	        
	          num_contacts = len(contacts)
	          # for each contact
	          G = np.zeros([6, num_contacts]) #the wrench matrix

	          for idx, c in enumerate(contacts):
	            pos = c[0:3] - obj_position
	            # print pos
	            dir = -c[3:] #this is already a unit vector
	          
	            #TODO fill G
	            G[0:3,idx] = dir.T
	            G[3:6,idx] = np.cross(pos,dir).T
	        
	          #TODO use G to compute scrores as discussed in class
	          U, s, V = np.linalg.svd(G, full_matrices=True)
	        
	          # print U.shape, s.shape, V.shape
	          # Metric 1 minimum singular value
	          if s.all() >= 0:
	            m1 = np.amin(s)
	          else:
	            m1 = 0

	        # Metric 2: volume of the ellipsoid  
	          if np.linalg.det(np.dot(G,G.T)) >= 0:
	            m2 = np.sqrt(np.linalg.det(np.dot(G,G.T)))
	          else:
	            m2 = 0;
	        
	        #Metric 3: Isotropy
	          sigma_min = np.amin(s)
	          sigma_max = np.amax(s)

	          if sigma_max > 0:
	            m3 = sigma_min / sigma_max
	          else:
	            m3 = 0

	        # print U.shape, s.shape, V.shape
	        #Need to come up with weights for each of the metric for evaluation function
	        # print 'm1: ' + repr(m1) + '\nm2: ' + repr(m2) + '\nm3: ' + repr(m3)
	        # rationale, m1 and m3 are highly correlated so I bring them to about the same order of magnitude
	        # m2, is very small and boosted to about the same order of magnitude as well
	          if np.linalg.matrix_rank(G) == 6:
	            return 100*m1+50000*m2+1000*m3 
	          else:
	            return 0


	        except openravepy.planning_error,e:
	        #you get here if there is a failure in planning
	        #example: if the hand is already intersecting the object at the initial position/orientation
	          return  0.00 # TODO you may want to change this