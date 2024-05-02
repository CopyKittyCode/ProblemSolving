import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path0, path1):
    #for t in range(max(len(path0), len(path1))):
    for t in range(0, len(path0)):
        # Check for vertex collision
        if get_location(path0, t) == get_location(path1, t):
            print("     vertex collision detected: ", get_location(path0, t), "timestep", t)
            return {'loc': get_location(path0, t), 'timestep': t}

        # Check for edge collision (swap)
        if t >= 0:
            loc0_prev = get_location(path0, t - 1)
            #print("loc0 prev", loc0_prev, t)
            loc1_prev = get_location(path1, t - 1)
            #print("loc1 prev", loc1_prev, t)
            loc0_curr = get_location(path0, t)
           # print("loc0 prev", loc0_curr, t)
            loc1_curr = get_location(path1, t)
            #print("loc1 prev", loc1_curr, t)
            if loc0_curr == loc1_prev and loc1_curr == loc0_prev:
                print("     edge collision detected path 0: loc", loc0_curr, "path 1", loc1_curr, "beginning time", t -1)
                return {'begin_loc0': loc0_prev, 'end_loc0':loc0_curr, 'begin_loc1': loc1_prev, 'end_loc1':loc1_curr, 'timestep': t-1}

    # No collision found
    return None

def detect_collisions(paths):
    collisions = []

    # Iterate over all pairs of agents
    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            # Detect collision between the current pair of agents
            collision = detect_collision(paths[i], paths[j])
            if collision:
                # Add collision information to the list
                collisions.append({'agents': (i, j), 'col': collision})
    
    return collisions

def standard_splitting(collision):
    print("standard splitting")
    constraints = []

    # Extract collision information
    agent0, agent1 = collision['agents']

    if 'loc' in collision['col']:  # For vertex collision
        location = collision['col']['loc']
        timestep = collision['col']['timestep']

        # Constraint to prevent the first agent at the specified location at the specified timestep
        constraints.append({'agent': agent0, 'loc': [location], 'timestep': timestep, 'positive':False})
        
        # Constraint to prevent the second agent at the specified location at the specified timestep
        constraints.append({'agent': agent1, 'loc': [location], 'timestep': timestep, 'positive':False})
        
    else: # For edge collision
        location1 = collision['col']['begin_loc0']
        location2 = collision['col']['end_loc0']
        location1 = collision['col']['begin_loc1']
        location2 = collision['col']['end_loc1']
        timestep = collision['col']['timestep']

        # Constraint to prevent the first agent from traversing the specified edge at the specified timestep
        constraints.append({'agent': agent0, 'loc': [location1, location2], 'timestep': timestep,'positive':False})
        
        # Constraint to prevent the second agent from traversing the specified edge at the specified timestep
        constraints.append({'agent': agent1, 'loc': [location2, location1], 'timestep': timestep, 'positive':False})
    
    return constraints


def disjoint_splitting(collision):
    constraints = []
    # Extract collision information
    agent = 0
    agent = 1
    ### both work, just randomly not. 
    #agent = random.choice(collision['agents'])  # Choose agent randomly
    if 'loc' in collision['col']:  # For vertex collision
        location = collision['col']['loc']
        timestep = collision['col']['timestep']

        # First constraint: Enforces the chosen agent to be at the specified location at the specified timestep (to be continued in planner)
        constraints.append({'agent': agent, 'loc': [location], 'timestep': timestep, 'positive':True})

        # Second constraint: Prevents the  same agent from being at the same location at the same timestep
        constraints.append({'agent': agent, 'loc': [location], 'timestep': timestep, 'positive': False})

    else:# For edge collision
        if(agent ==0):
            location1 = collision['col']['begin_loc0']
            location2 = collision['col']['end_loc0']
        else:
            location1 = collision['col']['begin_loc1']
            location2 = collision['col']['end_loc1']
        timestep = collision['col']['timestep']

        # First constraint: Enforces the chosen agent to traverse the specified edge at the specified timestep
        constraints.append({'agent': agent, 'loc': [location1, location2], 'timestep': timestep, 'positive': True})

        # Second constraint: Prevents the same agent from traversing the specified edge at the same timestep
        constraints.append({'agent': agent, 'loc': [location1, location2], 'timestep': timestep, 'positive': False})
    return constraints

def paths_violate_constraint(constraints, paths):
    violating_agents = []
    for constraint in constraints:
        for agent in range(0, len(paths)):
            loc = constraint['loc']
            timestep = constraint['timestep']
            p=paths[agent]
            if not(constraint['positive']) and constraint['agent']==agent:
                # Check if the agent's path violates its own negative constraint 
                if(len(loc)==1):
                    if(p[timestep] == loc[0]):
                        print("     agent ", agent, "is violating negative vertex constraint")
                        violating_agents.append(agent)
                if(len(loc)==2):
                    if  (p[timestep] == loc[0] and p[timestep+1] == loc[1]):
                        print("     agent ", agent, "is violating negative edge constraint")
                        violating_agents.append(agent)
            elif constraint['positive'] and constraint['agent']!=agent:
                    # Check if the other agent's paths violate the positive constraint 
                    if(len(loc)==1):
                        if(p[timestep] == loc[0]):
                            print("     agent ", agent, "is violating other agent's positive vertex constraint")
                            violating_agents.append(agent)
                    if(len(loc)==2):
                        if (p[timestep] == loc[1] and p[timestep+1] == loc[0]):
                            print("     agent", agent, "is violating other agent's positive edge constraint")
                            violating_agents.append(agent)
    return violating_agents


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("     Generate node {}".format(self.num_of_generated), "with cost", node['cost'])
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint):
        #to get disjoint back to false, modify standard in run_experiments parser
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()
        if timer.time() - self.start_time >0.3:
             raise BaseException('took too long')

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        #old_paths=root['paths']
        self.push_node(root)
        

        # High-Level Search
        while len(self.open_list) > 0:
            # Get the next node from the open list
            node = self.pop_node()
            old_paths=node['paths']
            print("parent node has constraints", node['constraints'], "and paths", node['paths'])
            # If this node has no collision, return solution
            if not node['collisions']:
                self.print_results(node)
                return node['paths']

            # Choose the first collision and convert to a list of constraints
            collision = node['collisions'][0]
            if  disjoint:
                constraints = disjoint_splitting(collision)
            else:
                constraints = standard_splitting(collision)

        

            # Add a new child node to the open list for each constraint
            child_no = 0
            #a list of the two children
            children = []
            p_e = [True, True]
            new_paths = []

            for constraint in constraints:
                child = {
                    'number': child_no,
                    'cost': 0,  # Placeholder for cost calculation
                    'constraints': node['constraints'] + [constraint],  
                    'paths':[None, None],  # Placeholder for paths calculation
                    'collisions': []  # Placeholder for collisions calculation
                }
 
                children.append(child)

        
                print("child", child_no,  "constraints", child['constraints'], "paths to compare", old_paths )
                violating_agents = paths_violate_constraint(child['constraints'], old_paths)
                #print(" violating agents", violating_agents)
                
                #gets me new paths for every violating agent for this child
                for agent in violating_agents:
                    #make a new path which respects the positive constraints
                    #print("recalculating path of agent", agent, "with the new constraint", child['constraints'])
                    path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                                agent, child['constraints'])
                    if path is None:
                        p_e[child_no]=False
                        print("no paths, i want to prune!")
                        #break
                    else:
                        new_paths.append({'child_no':child_no, 'agent':agent, 'path':path})

                #child['paths']=old_paths
                
                #print ("working on child no", child['number'], "with current paths ", child['paths'])   #if all violating agents have a new path, in this child, each violating agent gets its path updated.
                #if p_e[child['number']]:
                    #print("path exists")
                #give all the violating agents their new path
                for p in new_paths:
                    if p['child_no']==child['number']:
                        child['paths'][p['agent']]=p['path']
                #else: print("prune this somehow")                
                #print("work done, child", child['number'], "paths", child['paths'])
                #print("old paths here", old_paths)   

                child_no +=1             
                
                
            for child in children:
                #print("old paths mai jos", old_paths)   
                for i in range(0, len(child['paths'])):
                    if child['paths'][i]  is None:
                        child['paths'][i]=old_paths[i]
                        ##delete the for and it still works
            for child in children:
                if p_e[child['number']]:
                    print ("")
                    print("     pushed child", child)
                    child['cost'] = get_sum_of_cost(child['paths'])
                    child['collisions'] = detect_collisions(child['paths'])
                    self.push_node(child)
                else:print("pruning!")
            
        self.print_results(root)
        return root['paths']


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
        print("paths", node['paths'])
     
    """ if not disjoint:
        # Re-plan paths for each agent using the updated constraints
        for i in range(self.num_of_agents):
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                        i, child['constraints'])
            if path is None:
                raise BaseException('No solutions')
            child['paths'].append(path)

    # Update cost and collisions for the child node
    child['cost'] = get_sum_of_cost(child['paths'])
    child['collisions'] = detect_collisions(child['paths'])
    # Push the child node to the open list
    self.push_node(child)


    else:"""