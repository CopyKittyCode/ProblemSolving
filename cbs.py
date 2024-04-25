import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    for t in range(min(len(path1), len(path2))):
        # Check for vertex collision
        if get_location(path1, t) == get_location(path2, t):
            return {'loc': get_location(path1, t), 'time': t}

        # Check for edge collision (swap)
        if t > 0:
            loc1_prev = get_location(path1, t - 1)
            loc2_prev = get_location(path2, t - 1)
            loc1_curr = get_location(path1, t)
            loc2_curr = get_location(path2, t)
            if loc1_curr == loc2_prev and loc2_curr == loc1_prev:
                return {'loc1': loc1_curr, 'loc2': loc2_curr, 'time': t, }

    # No collision found
    return None
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.



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
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.

    pass

def standard_splitting(collision):
    print("splitting")
    constraints = []

    # Extract collision information
    agent1, agent2 = collision['agents']
    print("collision", collision)

    if 'loc' in collision['col']:  # For vertex collision
        print("vertex collision")
        location = collision['col']['loc'] # got to here
        timestep = 0  # As no 'time' key is found, assuming timestep 0
        
        # Constraint to prevent the first agent at the specified location at the specified timestep
        constraints.append({'agent': agent1, 'loc': [location], 'timestep': timestep})
        
        # Constraint to prevent the second agent at the specified location at the specified timestep
        constraints.append({'agent': agent2, 'loc': [location], 'timestep': timestep})
        
    elif 'loc1' in collision and 'loc2' in collision['col']:  # For edge collision
        print("edge collition")
        location1 = collision['loc1']
        location2 = collision['loc2']
        timestep = 0  # As no 'time' key is found, assuming timestep 0
        
        # Constraint to prevent the first agent from traversing the specified edge at the specified timestep
        constraints.append({'agent': agent1, 'loc': [location1, location2], 'timestep': timestep})
        
        # Constraint to prevent the second agent from traversing the specified edge at the specified timestep
        constraints.append({'agent': agent2, 'loc': [location2, location1], 'timestep': timestep})
    
    return constraints
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    pass


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    pass


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
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        print("cbs called")
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

        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        self.print_results(root)
        return root['paths']


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
