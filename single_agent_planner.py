import heapq

#takes location and direction and returns a new location. 
def move(loc, dir):
    #loc is a pair (x,y)
    #direction is a touple to be added to loc
    # 
    directions = [(0, -1),(1, 0), (0, 1), (-1, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        unique_locations = set(path)
        num_unique_locations = len(unique_locations)
        rst += num_unique_locations -1
        #rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    #hvalues for each agent are computed
    #contains a touple for the edge, and the value h
    return h_values
    

def build_constraint_table(constraints, agent, goal_loc, max_time):
    constraint_table = {}
    #constraint[agent]=0
    #current agent=1
    for constraint in constraints:
        ###check how edge constrains are implemented - better turn to 2 node constraints 
        cpy={}
        timestep = constraint['timestep']
        if constraint['agent'] != agent:
            cpy['agent']=agent
            cpy['loc'] = constraint['loc']
            cpy['timestep']=constraint['timestep']
            cpy['positive']=False
            #print("constraints at the end of the if", constraint)
            #print("copy at the end of the if", cpy)
            
            if timestep not in constraint_table:
                constraint_table[timestep] = {'pos_vertex': [], 'pos_edge': [], 'neg_vertex':[], 'neg_edge':[]}
            if  len(cpy['loc']) == 2:
                a=constraint['loc'][0]
                b=constraint['loc'][1]
                cpy['loc'][0]=b
                cpy['loc'][1]=a
                #print("copy if we have edge constraint", cpy)
                if cpy['positive']:
                    constraint_table[timestep]['pos_edge'].append(cpy)
                else:
                    constraint_table[timestep]['neg_edge'].append(cpy)
            elif len(cpy['loc']) == 1:
                if cpy['positive']:
                    #print("added", cpy)
                    constraint_table[timestep]['pos_vertex'].append(cpy)
                else:
                    #print("added", cpy)
                    constraint_table[timestep]['neg_vertex'].append(cpy)
        else:
            if timestep not in constraint_table:
                constraint_table[timestep] = {'pos_vertex': [], 'pos_edge': [], 'neg_vertex':[], 'neg_edge':[]}
            if  len(constraint['loc']) == 2:
                if constraint['positive']:
                    constraint_table[timestep]['pos_edge'].append(constraint)
                else:
                    constraint_table[timestep]['neg_edge'].append(constraint)
            elif len(constraint['loc']) == 1:
                if constraint['positive']:
                    constraint_table[timestep]['pos_vertex'].append(constraint)
                else:
                    constraint_table[timestep]['neg_vertex'].append(constraint)

    #print("constraints at the end of the table", constraints)  

    #do not leave your goal spot
    for k in range(0, max_time+1):

        if k not in constraint_table:
                constraint_table[k] = {'pos_vertex': [], 'pos_edge': [], 'neg_vertex':[], 'neg_edge':[]}

        constraint_table[k]['neg_edge'].append({'agent': agent, 'loc': [goal_loc, (goal_loc[0], goal_loc[1]+1)], 'timestep': k, 'positive':False})
        
        constraint_table[k]['neg_edge'].append({'agent': agent, 'loc': [goal_loc, (goal_loc[0], goal_loc[1]-1)], 'timestep': k, 'positive':False})
        
        constraint_table[k]['neg_edge'].append({'agent': agent, 'loc': [goal_loc, (goal_loc[0]+1, goal_loc[1])], 'timestep': k, 'positive':False})
       
        constraint_table[k]['neg_edge'].append({'agent': agent, 'loc': [goal_loc, (goal_loc[0]-1, goal_loc[1])], 'timestep': k, 'positive':False})
         
    return constraint_table

def print_table(table):
    for key, value in table.items():
        print(f"{key}: {value}")

def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained_positive(next_time, constraint_table):
    if next_time in constraint_table and (len(constraint_table[next_time]['pos_vertex'])>0 or len(constraint_table[next_time]['pos_edge'])>0 ):
       #print("pos_node at next time" ,constraint_table[next_time]['pos_vertex'], "pos_edge at next time", constraint_table[next_time]['pos_edge'])
       return True
    return False

#vertex and edge 
def is_constrained_negative(curr_loc, next_loc, next_time, constraint_table):
    if next_time in constraint_table:
        for vertex_constraint in constraint_table[next_time]['neg_vertex']:
            if next_loc in vertex_constraint['loc']:
                return True
        for edge_constraint in constraint_table[next_time-1]['neg_edge']:
            if curr_loc == edge_constraint['loc'][0] and next_loc == edge_constraint['loc'][1]:
                #print("neg edge constraint", edge_constraint, "timestep", edge_constraint['timestep'])
                return True
    return False



def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']

def is_within_bounds(loc, my_map):
    return 0 <= loc[0] < len(my_map) and 0 <= loc[1] < len(my_map[0])


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    #print("a star called with args", "start_loc", start_loc, "goal_loc", goal_loc, "agent", agent, "constraints", constraints)
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """
    open_list = []
    closed_list = dict()
    h_value = h_values[start_loc]
    max_time = 5
    c_table = build_constraint_table(constraints, agent, goal_loc, max_time)
    #print_table(c_table)
    earliest_goal =-1
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'time_step':0}
    push_node(open_list, root)
    closed_list[(root['loc'], root['time_step'])] = root

    while len(open_list) > 0:
        curr = pop_node(open_list)
        #print("agent ", agent, "at position ", curr['loc'], " at time ", curr['time_step'])
        # remember when agent first reached its goal
        if(earliest_goal==-1 and curr['loc'] == goal_loc):
                earliest_goal=curr['time_step']

        #### dealing with goal constraints
        ###if after arriving at goal I encounter any constraints, discard whole path since im not allowed to move from goal
        #if(earliest_goal<curr['time_step']) and is_constrained_negative(curr['loc'], curr['loc'], curr['time_step']+1, c_table):
            #print("agent", agent, "at goal", curr['loc'], "at time", curr['time_step'], "is not allowed to be here, unviable path")
      
        
        #if i reach the max steps and im at goal
        if curr['time_step']==max_time and curr['loc'] == goal_loc:
                print("     agent", agent, "reached goal", goal_loc, "by path",get_path(curr))
                #print("earliest_goal for agent", agent, " is ", earliest_goal)
                return(get_path(curr))
        # if i reach the goal loc before the time is up and im allowed here for the next step, wait
        elif curr['loc'] == goal_loc and not is_constrained_negative(curr['loc'], curr['loc'], curr['time_step'] + 1, c_table):
            child = {'loc': curr['loc'],
            'g_val': curr['g_val'] + 1,
            'h_val': h_values[curr['loc']],
            'parent': curr,
            'time_step': curr['time_step'] + 1}
        
        if is_constrained_positive(curr['time_step'] + 1, c_table):
            
            if len(c_table[curr['time_step']+1]['pos_vertex'])>0:
                use_node=c_table[curr['time_step']+1]['pos_vertex'][0]['loc'][0]
                print("agent", agent, "with positive constraint must be at", use_node, "at time", curr['time_step']+1)
                child = {'loc': use_node,
                'g_val': curr['g_val'] + 1,
                'h_val': h_values[use_node],
                'parent': curr,
                'time_step': curr['time_step'] + 1}
            if len(c_table[curr['time_step']+1]['pos_edge'])>0:
                use_edge = c_table[curr['time_step']+1]['pos_edge'][0]['loc']
                print("use edge", use_edge)
                child = {'loc': use_edge[1],
                'g_val': curr['g_val'] + 1,
                'h_val': h_values[use_edge[1]],
                'parent': curr,
                'time_step': curr['time_step'] + 1}
            

        #moving:
        for dir in range(4):
          
            child_loc = move(curr['loc'], dir)
            if child_loc is None or not is_within_bounds(child_loc, my_map):
                continue

            if (my_map[child_loc[0]][child_loc[1]]):
                 # wall, do not add to possibilities
                continue 
            if is_constrained_negative(curr['loc'], child_loc, curr['time_step'] + 1, c_table) and not is_constrained_negative(curr['loc'], curr['loc'], curr['time_step'] + 1, c_table):
                # If the transition is constrained, and im allowed to stay where i am, wait in place
                #print("agent", agent, "with next location", child_loc, " neg constrained at time", curr['time_step']+1)
                #print_table(c_table)
                child = {'loc': curr['loc'],
                'g_val': curr['g_val'] + 1,
                'h_val': h_values[curr['loc']],
                'parent': curr,
                'time_step': curr['time_step'] + 1}
            
            elif not is_constrained_negative(curr['loc'], child_loc, curr['time_step'] + 1, c_table):
                # If the target location is not blocked, proceed with creating a child node for the move
                child = {'loc': child_loc,
                 'g_val': curr['g_val'] + 1,
                 'h_val': h_values[child_loc],
                 'parent': curr,
                 'time_step': curr['time_step'] + 1}
            
            

            # Check if the child node is already in the closed list
            if (child['loc'], child['time_step']) in closed_list:
                existing_node = closed_list[(child['loc'], child['time_step'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['time_step'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['time_step'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
