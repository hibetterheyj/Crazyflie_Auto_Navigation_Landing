"""
crazyfile_path_planning.py
compute waypoints and plan for the crazyfile
"""
import numpy as np

def compute_orient(yaw):
    if yaw >=-45 and yaw < 45:
        return 'e'
    elif yaw >= 45 and yaw < 135:
        return 'n'
    elif (yaw >=135 and yaw < 180) or (yaw >= -180 and yaw < -135):
        return 'w'
    elif yaw >= -135 and yaw < -45:
        return 's'

def compute_ang(orient):
    if orient == 'e':
        return 0
    elif orient == 'n':
        return 90
    elif orient == 'w':
        return 180
    elif orient == 's':
        return -90

def orient_rotate90(origin_orient):
    if origin_orient == 'e':
        return 'n'
    elif origin_orient == 'n':
        return 'w'
    elif origin_orient == 'w':
        return 's'
    elif origin_orient == 's':
        return 'e'

# compare the current detected z with past logs of z
def detect_box_edge(z_log, curr_z, box_threshold):
    prev_z = sum(z_log)/len(z_log)
    if abs(curr_z-prev_z) > box_threshold:
        # print("box edge detected!")
        return True
    return False

# save current z into log with length of `win_len`
def save_z_log(z_log, curr_z, win_len=2):
    """save past sensor data"""
    z_log.append(curr_z)
    if len(z_log) >= win_len:
        z_log.pop(0)
    return z_log

# compute box center according detect points, current moving orientation, and in or out from the box
def compute_box_center(x_global, y_global, orient, io):
    if orient == 'e':
        box_x_computed = True
        box_y_computed = False
        if io == 'in':
            cal_pos = x_global + 0.15
        elif io == 'out':
            cal_pos = x_global - 0.15
    elif orient == 'n':
        box_x_computed = False
        box_y_computed = True
        if io == 'in':
            cal_pos = y_global + 0.15
        elif io == 'out':
            cal_pos = y_global - 0.15
    elif orient == 'w':
        box_x_computed = True
        box_y_computed = False
        if io == 'in':
            cal_pos = x_global - 0.15
        elif io == 'out':
            cal_pos = x_global + 0.15
    elif orient == 's':
        box_x_computed = False
        box_y_computed = True
        if io == 'in':
            cal_pos = y_global - 0.15
        elif io == 'out':
            cal_pos = y_global + 0.15
    return box_x_computed, box_y_computed, cal_pos

# rotate 180 degrees for the change of search_direction in coverage path planning
def rotate180(angle):
    if angle == 90:
        return -90
    else:
        return 90

## functions for coverage path planning
# with numpy implementation
def compute_coverage_wpts_np(offset_boundary, sweeping_spacing, search_direction, region_x, region_y, ratio=100, region_origin = [0,0]):
    start_x = offset_boundary + region_origin[0]
    end_x = region_x - offset_boundary + 1 + region_origin[0]
    globalx_list = np.arange(start_x, end_x, sweeping_spacing)
    start_y = offset_boundary + region_origin[1]
    end_y = region_y - offset_boundary + 1 + region_origin[1]
    globaly_list = np.arange(start_y, end_y, sweeping_spacing)
    for idx, i in enumerate(globalx_list):
        if search_direction == 90:
            for idy, j in enumerate(globaly_list):
                if idx==0 and idy==0:
                    waypoints = np.array([[i/ratio, j/ratio]])
                else:
                    waypoints = np.append(waypoints, [[i/ratio, j/ratio]], axis=0)
        else:
            for idy, j in enumerate(globaly_list[::-1]):
                if idx==0 and idy==0:
                    waypoints = np.array([[i/ratio, j/ratio]])
                else:
                    waypoints = np.append(waypoints, [[i/ratio, j/ratio]], axis=0)
        search_direction = rotate180(search_direction)
    return waypoints

# compute a real waypoint to a single grid waypoint
def compute_grid_idx(ref_x, ref_y, sweeping_spacing, wpt, region_origin, ratio=100):
    #print("Current ref: ({}, {}); Waypoint: ({}, {})".format(ref_x, ref_y, wpt[0], wpt[1]))
    x = ((ref_y - wpt[1])*ratio) / (sweeping_spacing)
    y = ((wpt[0] - ref_x)*ratio) / (sweeping_spacing)
    return np.array([[round(x), round(y)]], dtype=int)

# compute real waypoints to grid waypoints
def compute_wpts2grid_np(ref_x, ref_y, sweeping_spacing, wpts, region_origin = [0,0], ratio=100):
    grid_waypoints = compute_grid_idx(ref_x, ref_y, sweeping_spacing, wpts[0], region_origin, ratio)
    for wpt in wpts[1:]:
        grid_waypoints = np.append(grid_waypoints, compute_grid_idx(ref_x, ref_y, sweeping_spacing, wpt, region_origin, ratio), axis=0)
    return grid_waypoints

# compute a grid waypoint to a single real waypoint
def compute_grid_wpt(grid_idx, ref_x, ref_y, sweeping_spacing, ratio=100):
    wpt_x = grid_idx[1] * sweeping_spacing/ratio + ref_x
    wpt_y = ref_y - grid_idx[0] * sweeping_spacing/ratio
    return np.array([[wpt_x, wpt_y]])

# compute grid waypoints to real waypoints
def compute_grid2wpts_np(sub_grid_idx_list, ref_x, ref_y, sweeping_spacing, ratio=100):
    waypoints = compute_grid_wpt(sub_grid_idx_list[0], ref_x, ref_y, sweeping_spacing, ratio)
    for grid_idx in sub_grid_idx_list[1:]:
        waypoints = np.append(waypoints, compute_grid_wpt(grid_idx, ref_x, ref_y, sweeping_spacing, ratio), axis=0)
    return waypoints

## Path planning
# class Node for the "A star" algorithm
# code partially referenced from https://github.com/Antho1426/thymio-slam
class Node():
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

# use A* to replan the path if encountering the obstacle
def a_star(grid_map, start, end):
    """
    Input:
        grid_map: matrix representing the map
        start: start waypoint position, like (ys, xs)
        end: start waypoint position, like (ye, xe)
    Output:
        list of available grid
    """
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0
    open_list = []
    closed_list = []
    open_list.append(start_node)
    while len(open_list) > 0:
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
        open_list.pop(current_index)
        closed_list.append(current_node)
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            node_position = (current_node.position[0] + new_position[0],
                             current_node.position[1] + new_position[1])
            if node_position[0] > (len(grid_map) - 1) or node_position[0] < 0 or \
                    node_position[1] > (len(grid_map[len(grid_map) - 1]) - 1) or \
                    node_position[1] < 0:
                continue
            if grid_map[node_position[0]][node_position[1]] != 0:
                continue
            new_node = Node(current_node, node_position)
            children.append(new_node)
        for child in children:
            for closed_child in closed_list:
                if child == closed_child:
                    continue
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + (
                        (child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue
            open_list.append(child)

# Computes the required sensor direction for obstacle detection and moving orientation
# from computed grid waypoints path and heading orient
def compute_sensor_move(path, orient):
    """
    Input:
        path: (ye, xe)
        orient: start orientation
    Output:
        sensor_direction_list: list of sensor direction, such as front, right, left, back
        move_orient_list: list of moving orientations, such as 'n' (north), 's' (south), 'e' (east), 'w' (west)
    """
    # path = a_star(grid_map, start, end)
    move_orient_list = []
    sensor_direction_list = []
    for i in range(0, len(path) - 1):
        if orient == 'e':
            # [2, 0] means row2 and column0
            if (path[i][0] == path[i+1][0]) and (path[i][1] < path[i+1][1]):
                sensor_direction_list.append("front")
                move_orient_list.append("e")
            elif (path[i][0] == path[i+1][0]) and (path[i][1] > path[i+1][1]):
                sensor_direction_list.append("back")
                move_orient_list.append("w")
            elif (path[i][0] < path[i+1][0]) and (path[i][1] == path[i+1][1]):
                sensor_direction_list.append("right")
                move_orient_list.append("s")
            elif (path[i][0] > path[i+1][0]) and (path[i][1] == path[i+1][1]):
                sensor_direction_list.append("left")
                move_orient_list.append("n")
        elif orient == 'n':
            # [2, 0] -> [2,1]
            if (path[i][0] == path[i+1][0]) and (path[i][1] < path[i+1][1]):
                sensor_direction_list.append("right")
                move_orient_list.append("e")
            elif (path[i][0] == path[i+1][0]) and (path[i][1] > path[i+1][1]):
                sensor_direction_list.append("left")
                move_orient_list.append("w")
            elif (path[i][0] < path[i+1][0]) and (path[i][1] == path[i+1][1]):
                sensor_direction_list.append("back")
                move_orient_list.append("s")
            elif (path[i][0] > path[i+1][0]) and (path[i][1] == path[i+1][1]):
                sensor_direction_list.append("front")
                move_orient_list.append("n")
        elif orient == 'w':
            # [2, 0] -> [2,1]
            if (path[i][0] == path[i+1][0]) and (path[i][1] < path[i+1][1]):
                sensor_direction_list.append("back")
                move_orient_list.append("e")
            elif (path[i][0] == path[i+1][0]) and (path[i][1] > path[i+1][1]):
                sensor_direction_list.append("front")
                move_orient_list.append("w")
            elif (path[i][0] < path[i+1][0]) and (path[i][1] == path[i+1][1]):
                sensor_direction_list.append("left")
                move_orient_list.append("s")
            elif (path[i][0] > path[i+1][0]) and (path[i][1] == path[i+1][1]):
                sensor_direction_list.append("right")
                move_orient_list.append("n")
        elif orient == 's':
            # [2, 0] -> [2,1]
            if (path[i][0] == path[i+1][0]) and (path[i][1] < path[i+1][1]):
                sensor_direction_list.append("left")
                move_orient_list.append("e")
            elif (path[i][0] == path[i+1][0]) and (path[i][1] > path[i+1][1]):
                sensor_direction_list.append("right")
                move_orient_list.append("w")
            elif (path[i][0] < path[i+1][0]) and (path[i][1] == path[i+1][1]):
                sensor_direction_list.append("front")
                move_orient_list.append("s")
            elif (path[i][0] > path[i+1][0]) and (path[i][1] == path[i+1][1]):
                sensor_direction_list.append("back")
                move_orient_list.append("n")
        else:
            print("ERROR: unexpected orientation")
    return sensor_direction_list, move_orient_list

# computed idx for corresponding sensor_direction
# in `_four_dist` from `multiranger_update.py`
def cal_sensor_direction(sensor_direction):
    """
    _four_dist = [self._front_distance, self._back_distance, self._left_distance, self._right_distance]
    """
    if sensor_direction == "front":
        return 0
    elif sensor_direction == "back":
        return 1
    elif sensor_direction == "left":
        return 2
    elif sensor_direction == "right":
        return 3

#############################################
## Deprecated!
#############################################
# compute coverage waypoints for the task
def compute_coverage_wpts(offset_boundary, sweeping_spacing, search_direction, region_x, region_y, ratio=100, region_origin = [0,0]):
    # origin in cm (use int for list feature)
    waypoints = []
    region_start_x = region_origin[0]
    region_start_y = region_origin[1]
    start_y = offset_boundary + region_start_y
    end_y = region_y - offset_boundary + 1 + region_start_y
    globaly_list = list(range(start_y, end_y, sweeping_spacing))
    for i in range(offset_boundary+region_start_x, region_x-offset_boundary+1+region_start_x, sweeping_spacing):
        if search_direction == 90:
            for idx, j in enumerate(globaly_list):
                if idx == len(globaly_list)-1:
                    waypoints.append([i/ratio, j/ratio, 0])
                else:
                    waypoints.append([i/ratio, j/ratio, search_direction])
        else:
            for idx, j in enumerate(globaly_list[::-1]):
                if idx == len(globaly_list)-1:
                    waypoints.append([i/ratio, j/ratio, 0])
                else:
                    waypoints.append([i/ratio, j/ratio, search_direction])
        search_direction = rotate180(search_direction)
    return waypoints

def compute_astar_path(ys, xs, ye, xe, orient, grid_map):
    """
    Computes the path (using the "astar" function) from a start to an end
    position on the grid_map and the corresponding required list of commands
    :param ys: start y position
    :param xs: start x position
    :param ye: end y position
    :param xe: end x position
    :param orient: start orientation
    :param grid_map: matrix representing the map of the grid_map
    :return: list of commands to execute for travelling along the computed path
    """
    path = a_star(grid_map, (ys, xs), (ye, xe))
    list_of_commands = []
    list_orient = [orient]
    for i in range(0, len(path) - 1):

        if orient == 'n':
            if path[i][0] < path[i + 1][0]:
                list_of_commands.append('turn_back')
                orient = 's'
                list_orient.append(orient)
                list_of_commands.append('go_forward')
                list_orient.append(orient)
            elif path[i][0] > path[i + 1][0]:
                list_of_commands.append('go_forward')
                list_orient.append(orient)
            elif path[i][1] < path[i + 1][1]:
                list_of_commands.append('turn_right')
                orient = 'e'
                list_orient.append(orient)
                list_of_commands.append('go_forward')
                list_orient.append(orient)
            else:
                list_of_commands.append('turn_left')
                orient = 'w'
                list_orient.append(orient)
                list_of_commands.append('go_forward')
                list_orient.append(orient)

        elif orient == 's':
            if path[i][0] < path[i + 1][0]:
                list_of_commands.append('go_forward')
                list_orient.append(orient)
            elif path[i][0] > path[i + 1][0]:
                list_of_commands.append('turn_back')
                orient = 'n'
                list_orient.append(orient)
                list_of_commands.append('go_forward')
                list_orient.append(orient)
            elif path[i][1] < path[i + 1][1]:
                list_of_commands.append('turn_left')
                orient = 'e'
                list_orient.append(orient)
                list_of_commands.append('go_forward')
                list_orient.append(orient)
            else:
                list_of_commands.append('turn_right')
                orient = 'w'
                list_orient.append(orient)
                list_of_commands.append('go_forward')
                list_orient.append(orient)

        elif orient == 'e':
            if path[i][0] < path[i + 1][0]:
                list_of_commands.append('turn_right')
                orient = 's'
                list_orient.append(orient)
                list_of_commands.append('go_forward')
                list_orient.append(orient)
            elif path[i][0] > path[i + 1][0]:
                list_of_commands.append('turn_left')
                orient = 'n'
                list_orient.append(orient)
                list_of_commands.append('go_forward')
                list_orient.append(orient)
            elif path[i][1] < path[i + 1][1]:
                list_of_commands.append('go_forward')
                list_orient.append(orient)
            else:
                list_of_commands.append('turn_back')
                orient = 'w'
                list_orient.append(orient)
                list_of_commands.append('go_forward')
                list_orient.append(orient)

        else:  # i.e. orient == 'w'
            if path[i][0] < path[i + 1][0]:
                list_of_commands.append('turn_left')
                orient = 's'
                list_orient.append(orient)
                list_of_commands.append('go_forward')
                list_orient.append(orient)
            elif path[i][0] > path[i + 1][0]:
                list_of_commands.append('turn_right')
                orient = 'n'
                list_orient.append(orient)
                list_of_commands.append('go_forward')
                list_orient.append(orient)
            elif path[i][1] < path[i + 1][1]:
                list_of_commands.append('turn_back')
                orient = 'e'
                list_orient.append(orient)
                list_of_commands.append('go_forward')
                list_orient.append(orient)
            else:
                list_of_commands.append('go_forward')
                list_orient.append(orient)

    return list_of_commands, path
