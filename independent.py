import time as timer
#needed for path finding
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class IndependentSolver(object):
    """A planner that plans for each robot independently."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.CPU_time = 0

        # compute heuristics for the low-level search
        # this case for astar
        self.heuristics = []

        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))
            print("finding heuristics")
            print(self.heuristics) #size of this?

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        #record start time
        start_time = timer.time()
        #start an array for the result, a path for each agent
        result = []

        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, []) # call a star on the problem for each agent
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

        ##############################

        #end time - start time = time it took
        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))

        return result
