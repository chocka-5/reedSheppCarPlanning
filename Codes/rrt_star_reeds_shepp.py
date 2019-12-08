import xlrd
import xlwt
import time
import copy
import math
import random
import sys
import os
from xlwt import Workbook

wb = xlrd.open_workbook(r'''/home/chocka/Excel.xlsx''')
wb1 = Workbook()


sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../ReedsSheppPath/")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../RRTStar/")

try:
    import reeds_shepp_path_planning
    from rrt_star import RRTStar
except ImportError:
    raise




class RRTStarReedsShepp(RRTStar):
    """
    Class for RRT star planning with Reeds Shepp path
    """

    class Node(RRTStar.Node):
        """
        RRT Node
        """

        def __init__(self, x, y, yaw):
            super().__init__(x, y)
            self.yaw = yaw
            self.path_yaw = []

    def __init__(self, start, goal, obstacle_list, rand_area,
                 max_iter=200,
                 connect_circle_dist=50.0
                 ):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        """
        self.start = self.Node(start[0], start[1], start[2])
        self.end = self.Node(goal[0], goal[1], goal[2])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.connect_circle_dist = connect_circle_dist

        self.curvature = 1.0
        self.goal_yaw_th = np.deg2rad(1.0)
        self.goal_xy_th = 0.5

    def planning(self, search_until_max_iter=True):
        """
        planning
        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd)

            if self.check_collision(new_node, self.obstacle_list):
                near_indexes = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_indexes)
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_indexes)
                    self.try_goal_path(new_node)



            if (not search_until_max_iter) and new_node:  # check reaching the goal
                last_index = self.search_best_goal_node()
                if last_index:
                    return self.generate_final_course(last_index)

        print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index:
            return self.generate_final_course(last_index)
        else:
            print("Cannot find path")

        return None

    def try_goal_path(self, node):

        goal = self.Node(self.end.x, self.end.y, self.end.yaw)

        new_node = self.steer(node, goal)
        if new_node is None:
            return

        if self.check_collision(new_node, self.obstacle_list):
            self.node_list.append(new_node)


    def steer(self, from_node, to_node):

        px, py, pyaw, mode, course_lengths = reeds_shepp_path_planning.reeds_shepp_path_planning(
            from_node.x, from_node.y, from_node.yaw,
            to_node.x, to_node.y, to_node.yaw, self.curvature)

        if not px:
            return None

        new_node = copy.deepcopy(from_node)
        new_node.x = px[-1]
        new_node.y = py[-1]
        new_node.yaw = pyaw[-1]

        new_node.path_x = px
        new_node.path_y = py
        new_node.path_yaw = pyaw
        new_node.cost += sum([abs(l) for l in course_lengths])
        new_node.parent = from_node

        return new_node

    def calc_new_cost(self, from_node, to_node):

        _, _, _, _, course_lengths = reeds_shepp_path_planning.reeds_shepp_path_planning(
            from_node.x, from_node.y, from_node.yaw,
            to_node.x, to_node.y, to_node.yaw, self.curvature)
        if not course_lengths:
            return float("inf")

        return from_node.cost + sum([abs(l) for l in course_lengths])

    def get_random_node(self):

        rnd = self.Node(random.uniform(self.min_rand, self.max_rand),
                        random.uniform(self.min_rand, self.max_rand),
                        random.uniform(-math.pi, math.pi)
                        )

        return rnd

    def search_best_goal_node(self):

        goal_indexes = []
        for (i, node) in enumerate(self.node_list):
            if self.calc_dist_to_goal(node.x, node.y) <= self.goal_xy_th:
                goal_indexes.append(i)
        print("goal_indexes:", len(goal_indexes))

        # angle check
        final_goal_indexes = []
        for i in goal_indexes:
            if abs(self.node_list[i].yaw - self.end.yaw) <= self.goal_yaw_th:
                final_goal_indexes.append(i)

        print("final_goal_indexes:", len(final_goal_indexes))

        if not final_goal_indexes:
            return None

        min_cost = min([self.node_list[i].cost for i in final_goal_indexes])
        print("min_cost:", min_cost)
        for i in final_goal_indexes:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def generate_final_course(self, goal_index):
        path = [[self.end.x, self.end.y, self.end.yaw]]
        node = self.node_list[goal_index]
        while node.parent:
            for (ix, iy, iyaw) in zip(reversed(node.path_x), reversed(node.path_y), reversed(node.path_yaw)):
                path.append([ix, iy, iyaw])
            node = node.parent
        path.append([self.start.x, self.start.y, self.start.yaw])
        return path


def main():
    sheet = wb.sheet_by_index(0)
    sheet1 = wb.sheet_by_index(1)
    sheet2 = wb1.add_sheet('cost')
    sheet3 = wb1.add_sheet('time')
    L=int((sheet1.nrows)/100)
    for j in range(1):
        print(j)
        L1=int((j-1)*L)
        L2=int((j*L))
        obstacleList=[[] for i in range(L)]
        for k in range(L1,L2):
            obstacleList[k-(j-1)*L]=(sheet1.cell_value(k, 0)-0.5,sheet1.cell_value(k, 1)-0.5,0.6)
        start = [sheet.cell_value(2*(j-1), 0)-0.5,sheet.cell_value(2*(j-1), 1)-0.5, 0]
        goal = [sheet.cell_value((2*j-1), 0)-0.5,sheet.cell_value((2*j-1), 1)-0.5, 1.57]
        print(start)
        print(goal)
    #print(L2)
        #print(obstacleList)
        flag=0
        f=0
        while(flag==0 and f<3):
            f=f+1
            start_time = time.time()
            rrt = RRT(start, goal, randArea=[0.5,int(500/20)-0.5],obstacleList=obstacleList)
            path = rrt_star_reeds_shepp.planning
            #print(path)
            if path is not None:
                flag=flag+1
        cost=0
        if(flag>0):
            length=len(path)
            for i in path:
                i[0]=(round(i[0],4))
                i[1]=(round(i[1],4))
            for i in range(length-1):
                c=((path[i+1][0]-path[i][0])**2)
                d=((path[i+1][1]-path[i][1])**2)
                cost=cost+math.sqrt(c+d)
        co=cost*20
        print(co)
        t=time.time() - start_time
        print(t)
        #sheet2.write(j-1, 0,co)
        #sheet3.write(j-1, 0,t)
        #wb1.save('final1.xls')
if __name__ == '__main__':
    main()
