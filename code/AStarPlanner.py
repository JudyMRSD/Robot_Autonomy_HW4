import numpy as np
import datetime
import heapq

class NodeInfo:

    def __init__(self, node_id, parent_id, start_id, goal_id, planning_env, control):
        self.node_id = node_id
        self.parent_id = parent_id
        self.start_id = start_id
        self.goal_id = goal_id
        self.control = control

        self.planning_env = planning_env
        self.dist2start = self.computeDist(node_id, start_id)
        self.dist2goal = 10*self.computeDist(node_id, goal_id)
        
    def computeDist(self, node1, node2):
        return self.planning_env.ComputeDistance(node1, node2)
    
    def updateParent(self, parent_id, dist2start, control):
        self.parent_id = parent_id
        self.dist2start = dist2start
        self.control = control

class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()

    def plotEdge(self, src_id, dst_id):
        src_coord = self.planning_env.discrete_env.NodeIdToConfiguration(src_id)
        dst_coord = self.planning_env.discrete_env.NodeIdToConfiguration(dst_id)
        if self.visualize:
            self.planning_env.PlotEdge(src_coord, dst_coord)

    def Plan(self, start_config, goal_config):

        start = datetime.datetime.now()        

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)

        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        print(('A* planning ... \n Start State ... %s \n Start ID ... %s \n Goal State ... %s \n Goal ID ... %s \n') % (start_config, start_id, goal_config, goal_id))

        open_set = [(self.planning_env.ComputeDistance(start_id, goal_id), start_id)]
        closed_set = set([])

        closest_dist2goal = self.planning_env.ComputeDistance(start_id, goal_id)
        closest_node = start_id

        node_info = {start_id: NodeInfo(0, None, 0, goal_id, self.planning_env, None)}
        heapq.heapify(open_set)

        while (len(open_set) > 0):

            (t, node_id) = heapq.heappop(open_set)
            if node_id in closed_set:
                continue

            dist2goal = self.planning_env.ComputeDistance(node_id, goal_id)
            if dist2goal < closest_dist2goal:
                closest_dist2goal, closest_node = dist2goal, node_id

            if (node_id != start_id):
                closed_set.add((node_id, orient))
                self.plotEdge(node_info[node_id][2], node_id)

            if (len(node_info) % 5 ==0):
                print('Closest dist to goal : ', closest_dist2goal)

            if (node_id == goal_id):
                print('Goal found')
                break

            successors = self.planning_env.GetSuccessors(node_id)

            if len(successors) != 0:          
                for action in successors:
                    print('Footprint  ', action.footprint[-1])
                    (succ_id, orient_succ) = self.planning_env.ConfigurationToNodeId(action.footprint[-1])
                    if succ_id not in closed_set:
                        if succ_id in node_info:  
                            if node_info[succ_id].dist2start > node_info[node_id].dist2start+1:
                                node_info[succ_id].updateParent(nodeid, node_info[node_id].dist2start+1, action)
                        else: 
                            # Successor seen for the first time.
                            node_info[(succ_id, orient_succ)] = NodeInfo(succ_id, nodeid, start_id, goal_id, self.planning_env, action)
                            heapq.heappush(open_set, (node_info[succ_id].dist2start + node_info[succ_id].dist2goal, succ_id))
            else:
                print('No successors for ', node_id)
           

        plan = []
        if (goal_id not in node_info):
            print ('Goal not reached ! Cannot plan path')
        else:
            path = [goal_id]
            while path[-1] != start_id:
                path.append(node_info[path[-1]].control)
        
            plan =  path[::-1]
            elapsed = (datetime.datetime.now() - start).seconds
            print('Plan length :', len(plan))
            print('Nodes visited:', len(node_info))
            print('Elapsed time:', elapsed)

            if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
                self.planning_env.InitializePlot(goal_config)
                [self.planning_env.PlotEdge(plan[i-1].action.footprint[-1], plan[i].action.footprint[-1]) for i in range(1,len(plan))]

        return np.array(plan)
