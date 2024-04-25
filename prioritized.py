import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
        print("prio called")

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []
        """""
        ##################### Issue with this approach: map specific constraints, need to first run it and see where each agent is at which time, what time we stop at
        #node constraint
        #wait a step behind agent 1
        constraints.append({'agent': 0,'loc': [(1,2)],'timestep': 2})
        #edge constraint
        #go down at appropriate time
        constraints.append({'agent': 1,'loc': [(1,3),(1,4)],'timestep': 2})
        constraints.append({'agent': 1,'loc': [(1,3),(1,2)],'timestep': 2})
        #constraints to make them wait at their goal position
        ##CPU time (s):    0.02, Sum of costs:    18
        constraints.append({'agent': 0,'loc': [(1,5),(1,4)],'timestep': 5})
        constraints.append({'agent': 0,'loc': [(1,5),(1,4)],'timestep': 6})
        constraints.append({'agent': 0,'loc': [(1,5),(1,4)],'timestep': 7})
        constraints.append({'agent': 0,'loc': [(1,5),(1,4)],'timestep': 8})
        constraints.append({'agent': 0,'loc': [(1,5),(1,4)],'timestep': 9})

        constraints.append({'agent': 1,'loc': [(1,4),(1,5)],'timestep': 5})
        constraints.append({'agent': 1,'loc': [(1,4),(1,3)],'timestep': 5})

        constraints.append({'agent': 1,'loc': [(1,4),(1,5)],'timestep': 6})
        constraints.append({'agent': 1,'loc': [(1,4),(1,3)],'timestep': 6})

        constraints.append({'agent': 1,'loc': [(1,4),(1,5)],'timestep': 6})
        constraints.append({'agent': 1,'loc': [(1,4),(1,3)],'timestep': 6})

        constraints.append({'agent': 1,'loc': [(1,4),(1,5)],'timestep': 7})
        constraints.append({'agent': 1,'loc': [(1,4),(1,3)],'timestep': 7})

        constraints.append({'agent': 1,'loc': [(1,4),(1,5)],'timestep': 8})
        constraints.append({'agent': 1,'loc': [(1,4),(1,3)],'timestep': 8})

        constraints.append({'agent': 1,'loc': [(1,4),(1,5)],'timestep': 9})
        constraints.append({'agent': 1,'loc': [(1,4),(1,3)],'timestep': 9})

        """

        for i in range(self.num_of_agents):  # Find path for each agent
            #i=0
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            print("path", path)
            if path is None:
                raise BaseException('No solutions')
            
            # Add constraints for future agents
            for j in range(i + 1, self.num_of_agents):
                #vertex constraints
                for k in range(len(path)):
                        constraints.append({'agent': j,'loc': [path[k]],'timestep': k})

                #edge constraints
                for k in range(len(path)-1):
                    constraints.append({'agent': j, 'loc': [path[k], path[k+1]], 'timestep': k+1}) 


            result.append(path)

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
