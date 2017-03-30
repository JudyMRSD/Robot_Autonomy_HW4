import numpy, operator
from RRTPlanner import RRTTree
import time
class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        start_time = time.time();
        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)
        plan = []

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        while(1):           
            #import IPython
            #IPython.embed()
            #drop one vertex while
            v_drop = self.planning_env.GenerateRandomConfiguration()
            # compare the distance from ftree and the rtree
            i_nearst_f,v_nearst_f = ftree.GetNearestVertex(v_drop)
            i_nearst_r,v_nearst_r = rtree.GetNearestVertex(v_drop)
            ext_v_drop_to_f = self.planning_env.Extend(v_nearst_f,v_drop)
            ext_v_drop_to_r = self.planning_env.Extend(v_nearst_r,v_drop)

            if(ext_v_drop_to_f == None or ext_v_drop_to_r == None): continue
            
            dist_f = self.planning_env.ComputeDistance(ext_v_drop_to_f, v_drop)
            dist_r = self.planning_env.ComputeDistance(ext_v_drop_to_r, v_drop)

            # if the v_drop is close enough to both two tree,
            # add this vertice into both trees and then terminate.
            if(dist_f <= epsilon and dist_r <= epsilon): 
                final_vid_v_drop_r = rtree.AddVertex(v_drop)
                rtree.AddEdge(i_nearst_r,final_vid_v_drop_r)
                if(len(v_nearst_r) == 2): #make sure there is 2D config will show the plot
                    self.planning_env.PlotEdge(v_nearst_r,v_drop)

                final_vid_v_drop_f = ftree.AddVertex(v_drop)
                
                ftree.AddEdge(i_nearst_f,final_vid_v_drop_f)
                if(len(v_nearst_f) == 2): #make sure there is 2D config will show the plot
                    self.planning_env.PlotEdge(v_nearst_f,v_drop)
                
                break
            elif(dist_f <= dist_r): # v_drop more close to the ftree
                vid_v_drop_f = ftree.AddVertex(ext_v_drop_to_f)
                ftree.AddEdge(i_nearst_f,vid_v_drop_f)
                if(len(v_nearst_f) == 2):
                    self.planning_env.PlotEdge(v_nearst_f,ext_v_drop_to_f)
            
            else: # v_drop more close to the rtree
                vid_v_drop_r = rtree.AddVertex(ext_v_drop_to_r)
                rtree.AddEdge(i_nearst_r,vid_v_drop_r)
                if(len(v_nearst_r) == 2):
                    self.planning_env.PlotEdge(v_nearst_r,ext_v_drop_to_r)
        #assert(v_drop != None)
        # Terminate condition : the distance from v_drop to both ftree and rtree is less than epsilon 

        # use tree.edge one both tree to find the valid path
            
        # for ftree's path : [start_config -> v_drop) 
        total_time = time.time() - start_time;


        plan_ftree = self.find_path(ftree, 0, final_vid_v_drop_f)
        plan_ftree.reverse()
        
        # for rtree's path: [Goal_config -> v_drop) then reverse
        plan_rtree = self.find_path(rtree, 0, final_vid_v_drop_r)

        # combine ftree's path + v_drop + rtree's path
        for i in plan_ftree:
            plan.append(i)

        plan.append(v_drop)

        for i in plan_rtree:
            plan.append(i)
        
        dist_plan = self.planning_env.ComputePathLength(plan)
        print "total plan distance"
        print dist_plan
        
        print "total vertice in tree "
        print len(ftree.vertices) + len(rtree.vertices) -1
        # end of implement
        print "total plan time = "
        print total_time
        print " "
        return plan
    # help function to find the route
    def find_path(self, tree, start_id, end_id): #[start_id, end_id)
        id_next_v = end_id
        path = []
        while(id_next_v != start_id):
            id_next_v = tree.edges[id_next_v]
            path.append(tree.vertices[id_next_v])
        return path
