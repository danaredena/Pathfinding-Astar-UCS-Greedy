import cv2
import heapq
import copy

# Get picture
picture = input("Input a picture filename: ")
input_img = cv2.imread(picture, 1)

#output file
out_text = open('solutions.txt', 'w')
out_line = "Solutions for " + picture + "\n"
out_text.write(out_line)

dimen = input_img.shape
width = dimen[1] #col
height = dimen[0] #row

# Generate Grid
class Node:
    def __init__(self, cost, i, j, g_val, parent, path_cost):
        self.cost = cost
        self.parent = parent
        self.i = i
        self.j = j
        self.g_val = g_val
        self.path_cost = path_cost
    def get_parent(self):
        return self.parent
    def get_cost(self):
        return self.cost
    def get_g_val(self):
        return self.g_val
    def get_i(self):
        return self.i
    def get_j(self):
        return self.j
    def get_path_cost(self):
        return self.path_cost

def gen_grid(img, width, height):
    g_grid = []
    for i in range(0, height, 50):
        g_row = []
        for j in range(0, width, 50):
            g_val = input_img[i, j, 1]
            if (input_img[i, j, 0] == 0 and input_img[i, j, 1] == 0 and input_img[i, j, 2] == 255):
                red = Node(0, int(i/50), int(j/50), 0, None, 0)
            elif (input_img[i, j, 0] == 255 and input_img[i, j, 1] == 0 and input_img[i, j, 2] == 0):
                blue = Node(None, int(i/50), int(j/50), 0, None, None)
            elif (input_img[i, j, 0] == 0 and input_img[i, j, 1] == 0 and input_img[i, j, 2] == 0):
                g_val = -1
            g_row.append(g_val)
        g_grid.append(g_row)
    return g_grid, red, blue, int(i/50), int(j/50)

g_grid, start, goal, grid_row, grid_col = gen_grid(input_img, width, height)

# Generate Nodes
priorq = []
best_soln = [None, None, None]

def is_cycle(x, y, from_node):
    ances_node = from_node.get_parent()
    while(True):
        if (ances_node == None):
            return False
        if (ances_node.get_i() == x and ances_node.get_j() == y):
            return True
        ances_node = ances_node.get_parent()

# Check for valid path
def is_path(to_i, to_j, from_node, cond):
    global grid_row, grid_col, priorq, g_grid, goal, best_soln, start
    #in bounds
    if (to_i < 0 or to_i > grid_row):
        return
    elif (to_j < 0 or to_j > grid_col):
        return
    else:
        #Black cell
        to_g = g_grid[to_i][to_j]
        if (to_g == -1):
            return

        #exclude cycles
        if (is_cycle(to_i, to_j, from_node)):
            return

        if (cond == 0):
            cost = abs(int(from_node.get_g_val()) - int(to_g))+from_node.get_cost()
            path_cost = abs(int(from_node.get_g_val()) - int(to_g))+from_node.get_path_cost()
        elif (cond == 1):
            cost = (abs(to_i - goal.get_i()) + abs(to_j - goal.get_j()))
            path_cost = abs(int(from_node.get_g_val()) - int(to_g))+from_node.get_path_cost()
        elif (cond == 2):
            man_d = (abs(to_i - goal.get_i()) + abs(to_j - goal.get_j()))
            h = to_g+man_d
            cost = abs(int(from_node.get_g_val()) - int(to_g))+from_node.get_path_cost()
            path_cost = abs(int(from_node.get_g_val()) - int(to_g))+from_node.get_path_cost()

        #if goal state
        if (goal.get_i() == to_i and goal.get_j() == to_j):
            if (best_soln[cond] == None):
                best_soln[cond] = Node(cost, to_i, to_j, to_g, from_node, path_cost)
            elif (best_soln[cond].get_cost() > cost):
                best_soln[cond] = Node(cost, to_i, to_j, to_g, from_node, path_cost)
            return

        #for duplicates
        for i in range(len(priorq)):
            if (priorq[i][3].get_i() == to_i and priorq[i][3].get_j() == to_j):
                if (priorq[i][0] <= cost): #check
                    return
                else:
                    priorq.pop(i)
                    heapq.heapify(priorq)
                    break

        if (best_soln[cond] != None and best_soln[cond].get_cost() < cost):
            return
        else:
            new_node = Node(cost, to_i, to_j, to_g, from_node, path_cost)
            heapq.heappush(priorq, (cost, to_i, to_j, new_node))
            return

# Get Path
def px_path(i, j, from_node, cond):
    global g_grid
    curr_g = g_grid[i][j]

    #up path
    up = is_path(i-1, j, from_node, cond)
    #down path
    down = is_path(i+1, j, from_node, cond)
    #left path
    left = is_path(i, j-1, from_node, cond)
    #right path
    right = is_path(i, j+1, from_node, cond)


def pathfind(cond):
    global start, goal, priorq, best_soln
    px_path(start.get_i(), start.get_j(), start, cond)

    while (len(priorq) > 0):
        next_px = heapq.heappop(priorq)
        if (best_soln[cond] != None):
            if (cond == 1):
                return best_soln[cond]
            if (best_soln[cond].get_cost() <= next_px[0] ):
                return best_soln[cond]

        px_path(next_px[3].get_i(),next_px[3].get_j(), next_px[3], cond)
    return best_soln[cond]

def back_draw(goal, cond, img):
    global picture
    if (goal != None):
        node = goal.get_parent()
        from_x = (goal.get_i()*50)+25
        from_y = (goal.get_j()*50)+25
        to_x = (node.get_i()*50)+25
        to_y = (node.get_j()*50)+25
        img = cv2.line(img,(from_y,from_x),(to_y,to_x),(255,255,255),1)

        while (node.get_i() != start.get_i() or node.get_j() != start.get_j()):
            from_x = to_x
            from_y = to_y
            node = node.get_parent()
            to_x = (node.get_i()*50)+25
            to_y = (node.get_j()*50)+25
            img = cv2.line(img,(from_y,from_x),(to_y,to_x),(255,255,255),1)

    in_name = picture[:-4]
    if (cond == 0):
        out_name = in_name + "-ucs.png"
    elif (cond == 1):
        out_name = in_name + "-gbfs.png"
    elif (cond == 2):
        out_name = in_name + "-a*.png"
    output_img = cv2.imwrite(out_name, img)

best_soln[0] = pathfind(0)
best_soln[1] = pathfind(1)
best_soln[2] = pathfind(2)
if (best_soln[0] == None):
    out_text.write("NO SOLUTION/PATH FOUND")

else:
    out_line = "Uniform-Cost: " + str(best_soln[0].get_path_cost()) + "\n"
    out_text.write(out_line)

    out_line = "Greedy-BFS: " + str(best_soln[1].get_path_cost()) + "\n"
    out_text.write(out_line)

    out_line = "A*: " + str(best_soln[2].get_path_cost()) + "\n"
    out_text.write(out_line)

img0 = copy.copy(input_img)
img1 = copy.copy(input_img)
img2 = copy.copy(input_img)

back_draw(best_soln[0], 0, img0)
back_draw(best_soln[1], 1, img1)
back_draw(best_soln[2], 2, img2)
