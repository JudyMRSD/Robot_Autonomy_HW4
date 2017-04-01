import numpy as np
import datetime
import heapq

class NodeInfo:

    def __init__(self, node_id, parent_id, start_id, goal_id, planning_env, action):
        self.node_id = node_id
        self.parent_id = parent_id
        self.start_id = start_id
        self.goal_id = goal_id
        self.action = action
        self.planning_env = planning_env
        self.node_config = planning_env.discrete_env.NodeIdToConfiguration(node_id)

        self.hops2start = 0
        self.dist2goal = self.computeDist(node_id, goal_id)
        
    def computeDist(self, node1, node2):
        return self.planning_env.ComputeDistance(node1, node2)
    
    def computeHeuristic(self):
        return self.hops2start + 3*self.dist2goal

    def updateParent(self, parent_id, hops2start, action):
        self.parent_id = parent_id
        self.hops2start = hops2start
        self.action = action

class AStarPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()
        self.log_flag = True

    def log(self, txt, flag=True):
        if self.log_flag and flag:
            print(txt)

    def plotEdge(self, src_id, dst_id):
        src_coord = self.planning_env.discrete_env.NodeIdToConfiguration(src_id)
        dst_coord = self.planning_env.discrete_env.NodeIdToConfiguration(dst_id)
        if self.visualize:
            self.planning_env.PlotEdge(src_coord, dst_coord)

    def dumpHeap(self, lst):
        self.log(lst)

    def Plan(self, start_config, goal_config):

        start = datetime.datetime.now()        

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)

        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        self.log(('A* planning ... \n Start State ... %s \n Start ID ... %s \n Goal State ... %s \n Goal ID ... %s \n') % (start_config, start_id, goal_config, goal_id))
        open_set = [(self.planning_env.ComputeDistance(start_id, goal_id), start_id)]
        closed_set = set([])

        closest_dist2goal = self.planning_env.ComputeDistance(start_id, goal_id)
        closest_node = start_id
        self.log('Closest dist to goal : ' + str(closest_dist2goal))

        node_info = {start_id: NodeInfo(0, None, 0, goal_id, self.planning_env, None)}
        heapq.heapify(open_set)
       
        while (len(open_set) > 0):
            self.log('\nCurr queue ' + str(open_set), False)

            (t, node_id) = heapq.heappop(open_set)
            if node_id in closed_set:
                continue

            dist2goal = self.planning_env.ComputeDistance(node_id, goal_id)
            if dist2goal < closest_dist2goal:
                closest_dist2goal, closest_node = dist2goal, node_id

            if (node_id != start_id):
                closed_set.add(node_id)
                node_config = self.planning_env.discrete_env.NodeIdToConfiguration(node_id)
                self.log('Node ID ' + str(node_id) + ' Dist to goal : '+ str(node_info[node_id].dist2goal))
                self.plotEdge(node_info[node_id].parent_id, node_id)

            if (node_id == goal_id):
                self.log('Goal found')
                break

            successors = self.planning_env.GetSuccessors(node_id)

            if len(successors) != 0:          
                for [succ_id, action] in successors:
                    self.log('Successor ' + str(succ_id), False)
                    if succ_id not in closed_set:
                        if succ_id in node_info:  
                            self.log('Successor visited ' + str(succ_id), False)
                            if node_info[succ_id].hops2start > node_info[node_id].hops2start+1:
                                node_info[succ_id].updateParent(node_id, node_info[node_id].hops2start+1, action)
                        else: 
                            # Successor seen for the first time.
                            node_info[succ_id] = NodeInfo(succ_id, node_id, start_id, goal_id, self.planning_env, action)
                            self.log('Adding successor %s .. dist2goal : %s config: %s' %(succ_id, node_info[succ_id].dist2goal, node_info[succ_id].node_config), False)
                            heapq.heappush(open_set, (node_info[succ_id].computeHeuristic(), succ_id))
                    else:
                        self.log('Successor in closed set : %s' % (succ_id), False)
            else:
                self.log(('No successors for %s'% (node_id)), False)
           

        plan = []
        if (goal_id not in node_info):
            self.log ('Goal not reached ! Cannot plan path')
        else:
            path = [node_info[goal_id]]
            while path[-1].parent_id != start_id:
                path.append(node_info[path[-1].parent_id])
        
            plan =  path[::-1]
            elapsed = (datetime.datetime.now() - start).seconds
            self.log(('Plan length : %s') % (len(plan)))
            self.log(('Nodes visited: %s') % (len(node_info)))
            self.log(('Elapsed time: %s') % elapsed)

            if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
                self.planning_env.InitializePlot(goal_config)
                [self.planning_env.PlotEdge(plan[i-1].action.footprint[-1], plan[i].action.footprint[-1]) for i in range(1,len(plan))]

        actions = [item.action for item in plan]
        return np.array(actions)
