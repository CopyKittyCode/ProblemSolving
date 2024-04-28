import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    for t in range(min(len(path1), len(path2))):
        # Check for vertex collision
        if get_location(path1, t) == get_location(path2, t):
            print("vertex collision detected: ", get_location(path1, t), "timestep", t)
            return {'loc': get_location(path1, t), 'timestep': t}

        # Check for edge collision (swap)
        if t > 0:
            loc1_prev = get_location(path1, t - 1)
            loc2_prev = get_location(path2, t - 1)
            loc1_curr = get_location(path1, t)
            loc2_curr = get_location(path2, t)
            if loc1_curr == loc2_prev and loc2_curr == loc1_prev:
                print("edge collision detected")
                return {'loc1': loc1_curr, 'loc2': loc2_curr, 'timestep': t, }

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

        #Who has prio? Assume agent with lower number has prio. sheet says to constrain both
        
        # Constraint to prevent the first agent at the specified location at the specified timestep
        constraints.append({'agent': agent0, 'loc': [location], 'timestep': timestep, 'positive':False})
        
        
        # Constraint to prevent the second agent at the specified location at the specified timestep
        constraints.append({'agent': agent1, 'loc': [location], 'timestep': timestep, 'positive':False})
        
    elif 'loc1' in collision and 'loc2' in collision['col']:  # For edge collision
        print("edge collision")
        location1 = collision['loc1']
        location2 = collision['loc2']
        timestep = collision['col']['timestep']
        
        # Constraint to prevent the first agent from traversing the specified edge at the specified timestep
        #agent with lower number has prio
        constraints.append({'agent': agent0, 'loc': [location1, location2], 'timestep': timestep,'positive':False})
        
        # Constraint to prevent the second agent from traversing the specified edge at the specified timestep
        constraints.append({'agent': agent1, 'loc': [location2, location1], 'timestep': timestep, 'positive':False})
    
    return constraints


def disjoint_splitting(collision):
    constraints = []
    #dummy
    #constraints.append({'agent': 1, 'loc': [(2,4), (2,3)], 'timestep': 5, 'positive': True})
    # Extract collision information
    if 'loc' in collision['col']:  # For vertex collision
        location = collision['col']['loc']
        timestep = collision['col']['timestep']
        agent=0
        #agent = random.choice(collision['agents'])  # Choose agent randomly
    

        # First constraint: Enforces the chosen agent to be at the specified location at the specified timestep (to be continued in planner)
        constraints.append({'agent': agent, 'loc': [location], 'timestep': timestep, 'positive':True})

        # Second constraint: Prevents the  same agent from being at the same location at the same timestep
        constraints.append({'agent': agent, 'loc': [location], 'timestep': timestep, 'positive': False})

    elif 'loc1' in collision and 'loc2' in collision['col']:  # For edge collision
        location1 = collision['col']['loc1']
        location2 = collision['col']['loc2']
        timestep = collision['col']['timestep']
        agent = random.choice(collision['agents'])  # Choose agent randomly
        
        # First constraint: Enforces the chosen agent to traverse the specified edge at the specified timestep
        constraints.append({'agent': agent, 'loc': [location1, location2], 'timestep': timestep, 'positive': True})

        # Second constraint: Prevents the same agent from traversing the specified edge at the same timestep
        constraints.append({'agent': agent, 'loc': [location2, location1], 'timestep': timestep, 'positive': False})
    #print("disjoint constraints", constraints)
    return constraints

def paths_violate_constraint(constraints, paths):
    violating_agents = []
    for constraint in constraints:
        for agent in range(0, len(paths)):
            loc = constraint['loc']
            timestep = constraint['timestep']
            p=paths[agent]
            """if constraint['positive'] and constraint['agent']==agent:  # Check if the constraint is positive, check if agent respects its own positive constraint: it should always.
                # Check if the agent's path violates the positive constraint 
                print("agent's", agent, "path ", p, " checked for timestep", timestep, "loc", loc, "positive")
                if(len(loc)==1):
                    if(p[timestep] != loc[0]):
                        print("agent ", agent, "is violating positive vertex constraint")
                        violating_agents.append(agent)
                if(len(loc)==2):
                    if not (p[timestep] == loc[0] and p[timestep+1] == loc[1]):
                        print("agent ", agent, "is violating positive edge constraint")
                        violating_agents.append(agent)
            elif""" 
            if constraint['positive'] and constraint['agent']!=agent:
                # Check if the other agent's paths violate the positive constraint 
                print("agent's", agent, "path", p, " checked for timestep", timestep, "loc", loc, "positive")
                if(len(loc)==1):
                    if(p[timestep] == loc[0]):
                        print("agent ", agent, "is violating other agent's positive vertex constraint")
                        violating_agents.append(agent)
                if(len(loc)==2):
                    if (p[timestep] == loc[0] and p[timestep+1] == loc[1]):
                        print("agent ", agent, "is violating other agent's positive vertex constraint")
                        violating_agents.append(agent)
            
    print(violating_agents)    
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
        print("Generate node {}".format(self.num_of_generated), "with cost", node['cost'])
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint):
        #to get disjoint back to false, modify standard in run_experiments parser
        print("cbs called", "disjoint", disjoint)
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

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
        self.push_node(root)

        # High-Level Search
        while len(self.open_list) > 0:
            # Get the next node from the open list
            node = self.pop_node()

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

            print("parent node has constraints", node['constraints'], "and paths", node['paths'])

            # Add a new child node to the open list for each constraint
           ###better: for every
            for constraint in constraints:
                child = {
                    'cost': 0,  # Placeholder for cost calculation
                    'constraints': node['constraints'] + [constraint],  
                    'paths': [],  # Placeholder for paths calculation
                    'collisions': []  # Placeholder for collisions calculation
                }
                print("child node has constraints", child['constraints'], "and paths", child['paths'])

                if not disjoint:
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


                else:
                    violating_agents = paths_violate_constraint(child['constraints'], node['paths'])
                    print("violating agents", violating_agents)
                    """find new shortest paths not only for the agent with a newly imposed positive constraint but for other agents as well 
                        You can compute a list of agent ids of agents that violate a given positive constraint with the helper function paths_violate_constraint.
                        CBS with disjoint splitting should not add a child node if no path exists for one or more of these agents. 
                    #this is weird"""
                    child['paths']=node['paths']
                  
                    paths_exist = True
                    """if len(violating_agents)==0:
                        #no violating agents, all positive constraints are kept
                        #recalculate paths for agents without positive constraints
                        for agent in range(0, len(node['paths'])):
                            path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                                    agent, child['constraints'])
                            if path is None:
                                paths_exist = False
                                print("no paths")
                                break
                        if(paths_exist):
                            child['paths'][agent]=path
                    else:"""
                    for agent in violating_agents:
                        #make a new path which respects the positive constraints
                        print("recalculating path of agent", agent, "with the new constraint", child['constraints'])
                        path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                                    agent, child['constraints'])
                        if path is None:
                            paths_exist = False
                            print("no paths")
                            break
                    if(paths_exist):
                        child['paths'][agent]=path

                    # If paths exist for all agents that violate the positive constraint, update cost and collisions
                    if paths_exist:
                        child['cost'] = get_sum_of_cost(child['paths'])
                        child['collisions'] = detect_collisions(child['paths'])
                        self.push_node(child)


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
