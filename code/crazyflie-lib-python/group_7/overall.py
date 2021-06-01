# -*- coding: utf-8 -*-
"""
    # Overall pipeline
    # Parsing arguments
    # set (x, y) as follows
        python waypoint_following.py -x 0.6 -y 0.6
"""
import sys
import os
from os import write
from os import close
import logging
import time
import math
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
# use ggplot style for more sophisticated visuals
plt.style.use('ggplot')

# cflib
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
# multiranger_update: add `four_dist` property to retrieval four distance sensor at the same time
# from cflib.utils.multiranger import Multiranger
from cflib.utils.multiranger_update import Multiranger
# output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# self-developed library
from cf_state_class import Estimate
from cf_utilis import live_plotter, mkdir
from cf_load_params import set_task_options, set_other_options
from cf_search import orient_rotate90, detect_box_edge, save_z_log, compute_box_center, cal_sensor_direction
from cf_search import compute_coverage_wpts_np, compute_wpts2grid_np, compute_grid2wpts_np, compute_sensor_move, a_star

#%% variables
URI = 'radio://0/80/2M/E7E7E7E7E7'
# """
# our experiment2: 120x480 (30)
# start: mid: land = H120xW120, H120xW240 H120xW120
overall_region_w = 520.0 # cm
overall_region_h = 120.0 # cm
offset_boundary = 15.0 # cm
search_direction = -90 # degrees
sweeping_spacing = 30.0 # cm
region_offset_w = 30.0 # cm
land_region_w = 120.0 + region_offset_w # cm
land_region_h = 120.0 # cm
ratio = 100
# start_x, start_y = 345.0, 105 # cm
land_region_origin = [400.0, 0.0] # cm
des_x = (land_region_origin[0] + offset_boundary) / 100.0
heading_orient = 'e'
# """

"""
# final experiment: 300x500
# start: mid: land = H300xW150, H300xW200, H300xW150
overall_region_w = 500.0 # cm
overall_region_h = 300.0 # cm
offset_boundary = 7.5 # cm
search_direction = -90 # degrees
sweeping_spacing = 15.0 # cm
land_region_w = 60.0 # cm
land_region_h = 120.0 # cm
ratio = 100
start_x, start_y = 7.5, 112.5 # cm
land_region_origin = [180.0, 0.0] # cm
heading_orient = 'e'
"""

# grid map
grid_sz = sweeping_spacing
x_grid = math.ceil((land_region_w)/grid_sz)
y_grid = math.ceil((land_region_h)/grid_sz)

# logs
z_log = []
detected_pts = []
x_data = []
y_data = []
pos = []
edge1_detected = False
edge2_detected = False
pause_time = 0.1

#%% functions
# check whether Crazyfile is close to the obstacle
def is_close(range, min_dist=0.2):
    # min_dist: minimal allowed distance, default 0.2 m
    if range is None:
        return False
    else:
        return range < min_dist

# transfer the velocity from global to local coordinates
def glo_2_loc(yaw,vel_x,vel_y):
    loc_x = np.cos(yaw*np.pi/180)*vel_x - np.sin(yaw*np.pi/180)*vel_y
    loc_y = np.sin(yaw*np.pi/180)*vel_x + np.cos(yaw*np.pi/180)*vel_y
    return loc_x, loc_y

# regulate yaw to init angle
def regulate_yaw(mc, init_yaw, curr_yaw):
    if init_yaw - curr_yaw >= 0:
        mc.turn_left(init_yaw - curr_yaw)
    if init_yaw - curr_yaw < 0:
        mc.turn_right(curr_yaw - init_yaw)

# print and save as log file
def write_log(*args):
    line = ' '.join([str(a) for a in args])
    log_file.write(line+'\n')
    print(line)

# scan safe region for auto navigation to landing region
def scan_safe_region(mc,ranger,est,is_scan_y = True):
    if is_scan_y:
        print("scan y")
        left_d = ranger.left
        right_d = ranger.right
        x_t = est.est_x

        mc.move_distance(0.07, 0, 0, 0.1)
        time.sleep(0.5)
        left_d = min(ranger.left,left_d)
        right_d = min(ranger.right,right_d)

        mc.move_distance( -0.14, 0, 0, 0.1)
        time.sleep(0.5)
        left_d = min(ranger.left,left_d)
        right_d = min(ranger.right,right_d)

        while abs(est.est_x - x_t) > 0.02:
            mc.move_distance( x_t - est.est_x, 0, 0,0.05)

        mc.stop()

        return left_d - 0.15, right_d - 0.15
    else:
        print("scan x")
        front_d = ranger.front
        y_t = est.est_y

        mc.move_distance(0, 0.07, 0, 0.1)
        time.sleep(0.5)
        front_d = min(front_d,ranger.front)

        mc.move_distance(0, -0.14, 0, 0.1)
        time.sleep(0.5)
        front_d = min(front_d,ranger.front)

        while abs(est.est_y - y_t) > 0.02:
            mc.move_distance( 0, y_t - est.est_y, 0,0.05)

        mc.stop()

        return front_d

# print and save as log file
def write_log(*args):
    line = ' '.join([str(a) for a in args])
    log_file.write(line+'\n')
    print(line)

## __main__ function
if __name__ == '__main__':
    ## Load parameters
    parser = argparse.ArgumentParser(description='auto_nav')
    # set main parameters for self-developed algorithms
    set_task_options(parser)
    # set other parameters
    set_other_options(parser)
    args = parser.parse_args()

    map_x = overall_region_w/100.0
    map_y = overall_region_h/100.0

    # create a log file
    base_path = os.getcwd() + args.log_path
    mkdir(base_path)
    experiment_name = base_path+"overall-" + time.strftime("%Y%m%d_%H%M")
    log_file = open(experiment_name+".log","w")

    ## Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    lmeas = LogConfig(name='Meas', period_in_ms=100)
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # We take off when the commander is created
        with MotionCommander(scf) as mc:
            with Multiranger(scf) as multiranger:
                with Estimate(scf) as est:
                    mc.stop()
                    time.sleep(1)
                    mc._reset_position_estimator()
                    mc.stop()
                    time.sleep(1)
                    init_x = est.est_x
                    init_y = est.est_y
                    init_yaw = est.est_yaw
                    print("Start pos:",args.x+est.est_x-init_x,args.y+est.est_y-init_y)
                    # plotting handle
                    if args.with_visualization:
                        x_data.append(args.x*100)
                        y_data.append(args.y*100)
                        pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)

                    #############################################
                    # Navigate to the landing region with obstacle avoidance
                    #############################################
                    # Initial parameters
                    no_right = False
                    no_left = False
                    obstacle_front = False
                    while des_x - (est.est_x - init_x + args.x) > 0.15:
                        print("Now position is:", est.est_x - init_x + args.x,est.est_y - init_y + args.y)
                        print("Desired x of region is:", des_x)
                        time.sleep(0.5)
                        if is_close(multiranger.up):
                            print("force quit")
                            break

                        if multiranger.front < 0.25:
                            obstacle_front = True
                        if obstacle_front or no_left or no_right:
                            print("front unsafe")

                            obstacle_front = False
                            front_safe = False

                            start_y = est.est_y
                            start_x = est.est_x

                            # if est.est_y - init_y + args.y <  map_y/2:
                            if est.est_y - init_y + args.y < map_y/2:
                                while est.est_y - init_y + args.y + 0.15 < map_y:
                                    if no_left:
                                        print("no left")
                                        no_left = False
                                        break
                                    print("left search")
                                    if is_close(multiranger.up) or is_close(multiranger.left):
                                        print("left unsafe")
                                        break
                                    mc.start_linear_motion(0,0.2,0)
                                    time.sleep(0.1)
                                    if args.with_visualization:
                                        x_data.append((est.est_x - init_x+args.x)*100)
                                        y_data.append((est.est_y - init_y+args.y)*100)
                                        pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                                    if multiranger.front > 0.35:
                                        front_safe_d = multiranger.front
                                        front_safe = True
                                        mc.move_distance(0,0.05,0,0.05)
                                        print("left search found safe path")
                                        break

                                if not front_safe:
                                    if start_y != est.est_y:
                                        mc.move_distance(0,start_y - est.est_y,0,0.2)
                                        mc.move_distance(start_x - est.est_x,start_y - est.est_y,0,0.05)
                                    while est.est_y - init_y + args.y - 0.15 > 0:
                                        if no_right:
                                            print("no right")
                                            no_right = False
                                            break
                                        print("right search")
                                        if is_close(multiranger.up) or is_close(multiranger.right):
                                            print("right unsafe")
                                            break
                                        mc.start_linear_motion(0,-0.2,0)
                                        time.sleep(0.1)
                                        if args.with_visualization:
                                            x_data.append((est.est_x - init_x+args.x)*100)
                                            y_data.append((est.est_y - init_y+args.y)*100)
                                            pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                                        if multiranger.front > 0.35:
                                            front_safe_d = multiranger.front
                                            front_safe = True
                                            mc.move_distance(0,-0.05,0,0.05)
                                            print("right search found safe path")
                                            break
                            else:
                                while est.est_y - init_y + args.y - 0.15 > 0:
                                    if no_right:
                                        print("no right")
                                        no_right = False
                                        break
                                    print("right search")
                                    if is_close(multiranger.up) or is_close(multiranger.right):
                                        print("right unsafe")
                                        break
                                    mc.start_linear_motion(0,-0.2,0)
                                    time.sleep(0.1)
                                    if args.with_visualization:
                                        x_data.append((est.est_x - init_x+args.x)*100)
                                        y_data.append((est.est_y - init_y+args.y)*100)
                                        pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                                    if multiranger.front > 0.35:
                                        front_safe_d = multiranger.front
                                        front_safe = True
                                        mc.move_distance(0,-0.05,0,0.05)
                                        print("right search found safe path")
                                        break

                                if not front_safe:
                                    if start_y != est.est_y:
                                        mc.move_distance(0,start_y - est.est_y,0,0.2)
                                        mc.move_distance(start_x - est.est_x,start_y - est.est_y,0,0.05)
                                    while est.est_y - init_y + args.y + 0.15 < map_y:
                                        if no_left:
                                            no_left = False
                                            break
                                        print("left search")
                                        if is_close(multiranger.up) or is_close(multiranger.left):
                                            print("left unsafe")
                                            break
                                        mc.start_linear_motion(0,0.2,0)
                                        time.sleep(0.1)
                                        if args.with_visualization:
                                            x_data.append((est.est_x - init_x+args.x)*100)
                                            y_data.append((est.est_y - init_y+args.y)*100)
                                            pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                                        if multiranger.front > 0.35:
                                            front_safe_d = multiranger.front
                                            front_safe = True
                                            mc.move_distance(0,0.05,0,0.05)
                                            print("right search found safe path")
                                            break
                            if not front_safe:
                                print("I am trapped")
                                break
                        else:
                            front_safe_d = multiranger.front

                        if multiranger.left < 0.1 and multiranger.right > 0.15:
                            mc.stop()
                            mc.move_distance(0,-min(0.08,multiranger.right-0.1),0,0.2)
                            if args.with_visualization:
                                x_data.append((est.est_x - init_x+args.x)*100)
                                y_data.append((est.est_y - init_y+args.y)*100)
                                pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                        elif multiranger.right < 0.1 and multiranger.left > 0.15:
                            mc.stop()
                            mc.move_distance(0,min(0.08,multiranger.left-0.1),0,0.2)
                            if args.with_visualization:
                                x_data.append((est.est_x - init_x+args.x)*100)
                                y_data.append((est.est_y - init_y+args.y)*100)
                                pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)

                        mc.turn_right(15)
                        mc.stop()
                        time.sleep(0.5)
                        if multiranger.front < 0.2/np.cos(15*np.pi/180)+0.15:
                            regulate_yaw(mc, init_yaw, est.est_yaw)
                            # mc._reset_position_estimator
                            time.sleep(0.5)
                            if is_close(multiranger.left,0.25) or map_y - (est.est_y - init_y+args.y) < 0.25:
                                print("left is not available")
                                no_left = True
                                continue
                            else:
                                mc.move_distance(0,0.2,0,0.2)
                                time.sleep(0.5)
                                if args.with_visualization:
                                    x_data.append((est.est_x - init_x+args.x)*100)
                                    y_data.append((est.est_y - init_y+args.y)*100)
                                    pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                                front_safe_d = min(front_safe_d,multiranger.front)
                                mc.stop()
                        else:
                            mc.turn_left(30)
                            # mc._reset_position_estimator
                            mc.stop()
                            time.sleep(0.5)
                            if multiranger.front < 0.2/np.cos(15*np.pi/180)+0.15:
                                regulate_yaw(mc, init_yaw, est.est_yaw)
                                time.sleep(0.5)
                                if is_close(multiranger.right,0.25) or est.est_y - init_y+args.y < 0.25:
                                    print("right is not available")
                                    no_right = True
                                    continue
                                else:
                                    mc.move_distance(0,-0.2,0,0.2)
                                    time.sleep(0.5)
                                    if args.with_visualization:
                                        x_data.append((est.est_x - init_x+args.x)*100)
                                        y_data.append((est.est_y - init_y+args.y)*100)
                                        pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                                    front_safe_d = min(front_safe_d,multiranger.front)
                                    mc.stop()
                            regulate_yaw(mc, init_yaw, est.est_yaw)
                            time.sleep(0.5)
                            # mc._reset_position_estimator

                        go_d = min(des_x - (est.est_x - init_x + args.x),front_safe_d-0.25,0.5)
                        if front_safe_d - 0.25 == go_d:
                            obstacle_front = True
                        print("front safe, go:", go_d)
                        nav_des_x = est.est_x + go_d
                        nav_des_y = est.est_y
                        err_nav = 100
                        while err_nav > 0.12:
                            if multiranger.left < 0.1 and multiranger.right > 0.15:
                                mc.stop()
                                mc.move_distance(0,-min(0.2,multiranger.right-0.1),0,0.2)
                                if args.with_visualization:
                                    x_data.append((est.est_x - init_x+args.x)*100)
                                    y_data.append((est.est_y - init_y+args.y)*100)
                                    pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                            elif multiranger.right < 0.1 and multiranger.left > 0.15:
                                mc.stop()
                                mc.move_distance(0,min(0.2,multiranger.left-0.1),0,0.2)
                                if args.with_visualization:
                                    x_data.append((est.est_x - init_x+args.x)*100)
                                    y_data.append((est.est_y - init_y+args.y)*100)
                                    pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                            mc.start_linear_motion(0.4,0,0)
                            time.sleep(0.05)
                            if args.with_visualization:
                                x_data.append((est.est_x - init_x+args.x)*100)
                                y_data.append((est.est_y - init_y+args.y)*100)
                                pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                            err_nav = abs(nav_des_x-est.est_x)
                        if multiranger.left < 0.1 and multiranger.right > 0.15:
                            mc.stop()
                            mc.move_distance(0,-min(0.2,multiranger.right-0.1),0,0.2)
                            if args.with_visualization:
                                x_data.append((est.est_x - init_x+args.x)*100)
                                y_data.append((est.est_y - init_y+args.y)*100)
                                pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                        elif multiranger.right < 0.1 and multiranger.left > 0.15:
                            mc.stop()
                            mc.move_distance(0,min(0.2,multiranger.left-0.1),0,0.2)
                            if args.with_visualization:
                                x_data.append((est.est_x - init_x+args.x)*100)
                                y_data.append((est.est_y - init_y+args.y)*100)
                                pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                        mc.move_distance(nav_des_x-est.est_x,0,0,0.1)
                        if args.with_visualization:
                            x_data.append((est.est_x - init_x+args.x)*100)
                            y_data.append((est.est_y - init_y+args.y)*100)
                            pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                        mc.stop()

                    while abs(des_x - (est.est_x - init_x + args.x)) > 0.01:
                        if is_close(multiranger.up):
                            break
                        mc.move_distance(des_x - (est.est_x - init_x + args.x), 0, 0, 0.1)
                        if args.with_visualization:
                            x_data.append((est.est_x - init_x+args.x)*100)
                            y_data.append((est.est_y - init_y+args.y)*100)
                            pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)

                    regulate_yaw(mc, init_yaw, est.est_yaw)
                    mc.stop()
                    time.sleep(1)
                    print("Arrive desired x")
                    print("Now pos:",args.x+est.est_x-init_x,args.y+est.est_y-init_y)

                    z_log = save_z_log(z_log, est.est_z, win_len=args.avg_window)
                    print("Navigate end at x = ", est.est_x - init_x + args.x)
                    navi_end_y = est.est_y - init_y + args.y

                    #############################################
                    # Integrate current point into coverage path planning
                    #############################################
                    # basic model 2: 120x120
                    write_log("Initializing waypoint list")
                    des_list = compute_coverage_wpts_np(offset_boundary, sweeping_spacing,
                                                                search_direction, land_region_w, land_region_h, ratio, land_region_origin)
                    ref_x, ref_y = des_list[0][0], des_list[0][1]
                    grid_idx_list = compute_wpts2grid_np(ref_x, ref_y, sweeping_spacing,
                                                        des_list, land_region_origin, ratio)
                    sensor_direction_list, move_orient_list = compute_sensor_move(grid_idx_list, heading_orient)
                    # assume start position is north to the first waypoint
                    sensor_direction_list.insert(0,sensor_direction_list[0])
                    move_orient_list.insert(0,move_orient_list[0])

                    write_log("Initializing grid map")
                    land_zone_grid_np = np.zeros((y_grid, x_grid), dtype=int)
                    land_zone_grid = land_zone_grid_np.tolist()

                    write_log("Waypoint list", des_list)
                    write_log("Grid waypoint list", grid_idx_list)
                    write_log("Sensor direction list", sensor_direction_list)
                    write_log("Moving orient list", move_orient_list)
                    write_log("land zone grid size:", land_zone_grid_np.shape)

                    # Compute `navi_end_y` with y in des_list to discard useless waypoints
                    path_counter = -1
                    while (des_list[0][1] > navi_end_y):
                        path_counter += 1
                        if path_counter > 0:
                            # prepare for obstacle avoidance and replanning
                            prev_grid_idx = grid_idx
                        des, des_list = des_list[0], des_list[1:]
                        des[0] -= args.x
                        des[1] -= args.y
                        grid_idx, grid_idx_list = grid_idx_list[0], grid_idx_list[1:]
                        move_orient, move_orient_list = move_orient_list[0], move_orient_list[1:]
                        sensor_direction, sensor_direction_list = sensor_direction_list[0], sensor_direction_list[1:]
                        if grid_idx_list[0][1] == 1:
                            write_log("Discard the first column waypoints at most!")
                            break

                    reached_all_wpts = False
                    #############################################
                    # start waypoint following
                    #############################################
                    mc.stop()
                    # mc._reset_position_estimator()
                    time.sleep(1)
                    while (des_list.size != 0):
                        mc.stop()
                        time.sleep(0.1)
                        path_counter += 1
                        if path_counter > 0:
                            # prepare for obstacle avoidance and replanning
                            prev_grid_idx = grid_idx
                        des, des_list = des_list[0], des_list[1:]
                        des[0] -= args.x
                        des[1] -= args.y
                        grid_idx, grid_idx_list = grid_idx_list[0], grid_idx_list[1:]
                        move_orient, move_orient_list = move_orient_list[0], move_orient_list[1:]
                        sensor_direction, sensor_direction_list = sensor_direction_list[0], sensor_direction_list[1:]

                        #############################################
                        # detect whether the obstacle is blocked in next place
                        #############################################
                        write_log("Use {} sensor to detect currently".format(sensor_direction))
                        # if is_close(multiranger._right_distance, min_dist=0.15):
                        write_log("Obstacle in moving direction", multiranger._four_dist[cal_sensor_direction(sensor_direction)])
                        # adding additional 0.1 as delay distance
                        next_blocked = False
                        if is_close(multiranger._four_dist[cal_sensor_direction(sensor_direction)], min_dist=sweeping_spacing/ratio+0.1):
                            next_blocked = True
                        else:
                            mc.turn_left(10)
                            time.sleep(0.5)
                            if is_close(multiranger._four_dist[cal_sensor_direction(sensor_direction)], min_dist=(sweeping_spacing/ratio)/np.cos(10*np.pi/180)+0.12):
                                regulate_yaw(mc, init_yaw, est.est_yaw)
                                time.sleep(0.5)
                                next_blocked = True
                            else:
                                mc.turn_right(20)
                                time.sleep(0.5)
                                if is_close(multiranger._four_dist[cal_sensor_direction(sensor_direction)], min_dist=(sweeping_spacing/ratio)/np.cos(10*np.pi/180)+0.12):
                                    regulate_yaw(mc, init_yaw, est.est_yaw)
                                    time.sleep(0.2)
                                    next_blocked = True
                                else:
                                    regulate_yaw(mc, init_yaw, est.est_yaw)
                                    time.sleep(0.2)


                        if next_blocked:
                            write_log("Next step {} (grid: {}) is blocked, and start replanning ...".format(des, grid_idx))
                            # update idx in land_zone_grid as zero
                            land_zone_grid[grid_idx[0]][grid_idx[1]] = 1
                            # replan from previous point
                            start_np = prev_grid_idx
                            end_np = grid_idx_list[0]
                            start = start_np.tolist()
                            end = end_np.tolist()
                            write_log("A* replanning from {} to {}".format(start, end))

                            # compute `sub_grid_idx_list` and `sub_des_list` to end point
                            sub_grid_idx_tuple_list = a_star(land_zone_grid, tuple(start), tuple(end))
                            sub_grid_idx_list = [list(ele) for ele in sub_grid_idx_tuple_list]
                            sub_des_list = compute_grid2wpts_np(sub_grid_idx_list, ref_x, ref_y, sweeping_spacing, ratio)

                            # update new `des_list` and `grid_idx_list`; already plan towards the end point
                            des_list = np.append(sub_des_list, des_list[1:], axis=0)
                            grid_idx_list = np.append(sub_grid_idx_list, grid_idx_list[1:], axis=0)

                            # update sensor_direction_list, move_orient_list
                            sensor_direction_list, move_orient_list = compute_sensor_move(grid_idx_list, heading_orient)
                            # already reach the start point
                            des_list, grid_idx_list = des_list[1:], grid_idx_list[1:]
                            continue
                        else:
                            write_log("Next step is unblocked, and continue to fly ...")

                        write_log("Next destination ({}, {})".format(des[0]+args.x, des[1]+args.y))
                        time.sleep(0.5)

                        #############################################
                        # check overall error is smaller than reaching threshold (0.02) during searching
                        #############################################
                        curr_x = est.est_x - init_x
                        curr_y = est.est_y - init_y
                        overall_err = np.sqrt(np.square(des[0]-curr_x)+np.square(des[1]-curr_y))
                        while (overall_err >= 0.02):
                            ## waypoints following
                            # if overall_err < 0.02:
                            #     # if len(des_list) == 0:
                            #     mc.start_linear_motion(0, 0, 0)
                            #     write_log("Waypoint ({}, {}) reached!".format(des[0]+args.x, des[1]+args.y))
                            # else:
                            write_log("\nSearching 1st edge ...")
                            write_log("* Current estimated z", est.est_z)
                            edge1_detected = detect_box_edge(z_log, est.est_z, args.box_threshold)
                            write_log("edge1_detected:", edge1_detected)
                            ptx, pty = est.est_x+args.x-init_x, est.est_y+args.y-init_y
                            if (not edge1_detected):
                                z_log = save_z_log(z_log, est.est_z, win_len=args.avg_window)
                                # continue to fly towards current waypoint
                                if overall_err > 0.1:
                                    v_mag = 0.2
                                else:
                                    v_mag = 0.1
                                v_x = v_mag*(des[0]-curr_x)/overall_err
                                v_y = v_mag*(des[1]-curr_y)/overall_err
                                # write_log("current is ",curr_x+args.x,curr_y+args.y)
                                # write_log("des is ",des[0]+args.x,des[1]+args.y)
                                # write_log("global vel is:",v_x,v_y)
                                loc_v_x, loc_v_y = glo_2_loc(est.est_yaw, v_x, v_y)
                                # write_log("SPEED ({}, {})".format(loc_v_x, loc_v_y))
                                mc.start_linear_motion(loc_v_x, loc_v_y, 0)
                                time.sleep(0.2)
                                mc.start_linear_motion(0, 0, 0)
                                if overall_err < 0.1:
                                    time.sleep(0.1)
                                curr_x = est.est_x - init_x
                                curr_y = est.est_y - init_y
                                write_log("position:", curr_x+args.x ,curr_y+args.y)
                                # plotting handle
                                if args.with_visualization:
                                    x_data.append((curr_x+args.x)*100)
                                    y_data.append((curr_y+args.y)*100)
                                    pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)

                                overall_err = np.sqrt(np.square(des[0]-curr_x)+np.square(des[1]-curr_y))
                                if (len(des_list) == 0) and  (overall_err < 0.02):
                                    write_log("No landing pad found")
                                    reached_all_wpts = True
                                    mc.start_linear_motion(0, 0, 0)
                            else:
                                detected_pts.append([ptx, pty])
                                mc.stop()
                                time.sleep(1)
                                write_log("\nFirst edge detected in ({}, {})!\n".format(ptx, pty))
                                des_list = np.array([[]]) # clear all the waypoints
                                break

                    if (not reached_all_wpts):
                        #############################################
                        # move 0.15m after finding the 1st edge
                        #############################################
                        box_x_computed, box_y_computed, cal_pos = compute_box_center(detected_pts[0][0],
                                                                                                                                    detected_pts[0][1],
                                                                                                                                    move_orient,
                                                                                                                                    'in')
                        write_log("Computed center element", cal_pos)

                        write_log("\nbox_x_computed: {}\nbox_y_computed: {}".format(box_x_computed, box_y_computed))
                        regulate_yaw(mc, init_yaw, est.est_yaw)
                        move_dist = 0.15 # 0.15
                        write_log("Start moving in {} direction".format(move_orient))
                        write_log("position:", curr_x+args.x ,curr_y+args.y)
                        # plotting handle
                        if args.with_visualization:
                            x_data.append((est.est_x - init_x + args.x)*100)
                            y_data.append((est.est_y - init_y + args.y)*100)
                            pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)

                        if move_orient == 'e':
                            mc.move_distance(move_dist, 0, 0)
                        elif move_orient == 'n':
                            mc.move_distance(0, move_dist, 0)
                        elif move_orient == 's':
                            mc.move_distance(0, -move_dist, 0)
                        else:
                            mc.move_distance(-move_dist, 0, 0)

                        #############################################
                        # Rotate 90 degrees and start search second edge
                        #############################################
                        write_log("position:", est.est_x - init_x+args.x ,est.est_y - init_y+args.y)
                        # plotting handle
                        if args.with_visualization:
                            x_data.append((est.est_x - init_x + args.x)*100)
                            y_data.append((est.est_y - init_y + args.y)*100)
                            pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)

                        move_orient = orient_rotate90(move_orient)
                        write_log("move towards:", move_orient)
                        z_log = []
                        mc.stop()
                        for _ in range(5):
                            time.sleep(0.2)
                            z_log = save_z_log(z_log, est.est_z, win_len=args.avg_window)

                        # edge2_detected = detect_box_edge(z_log, est.est_z, args.box_threshold)
                        edge2_detected = False
                        regulate_yaw(mc, init_yaw, est.est_yaw)
                        cnt = 0
                        pstartx, pstarty = est.est_x+args.x-init_x, est.est_y+args.y-init_y
                        if move_orient == 'e' and multiranger.front < 0.4:
                            move_orient == 'w'
                        elif move_orient == 'n' and multiranger.left < 0.4:
                            move_orient == 's'
                        elif move_orient == 'w' and multiranger.back < 0.4:
                            move_orient == 'e'
                        elif move_orient == 's' and multiranger.right < 0.4:
                            move_orient == 'n'

                        # considering too close to the boundary!!!
                        while (not edge2_detected) and (cnt < 21):
                            cnt+=1
                            write_log("\nSearching 2nd edge ...")
                            if move_orient == 'e':
                                velocity_x = 0.1
                                velocity_y = 0
                            elif move_orient == 'n':
                                velocity_x = 0
                                velocity_y = 0.1
                            elif move_orient == 'w':
                                velocity_x = -0.1
                                velocity_y = 0
                            elif move_orient == 's':
                                velocity_x = 0
                                velocity_y = -0.1
                            mc.start_linear_motion(velocity_x, velocity_y, 0)
                            time.sleep(0.2)
                            # to verify second edge!
                            curr_x = est.est_x - init_x
                            curr_y = est.est_y - init_y
                            write_log("* Past estimated z", sum(z_log)/len(z_log))
                            write_log("* Current estimated z", est.est_z)
                            write_log("position:", curr_x+args.x ,curr_y+args.y)
                            # plotting handle
                            if args.with_visualization:
                                x_data.append((est.est_x - init_x + args.x)*100)
                                y_data.append((est.est_y - init_y + args.y)*100)
                                pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)

                            # (0.025) is not a parameter, should be args.box_threshold
                            edge2_detected = detect_box_edge(z_log, est.est_z, 0.025)
                            write_log("edge2_detected", edge2_detected)
                            ptx, pty = est.est_x+args.x-init_x, est.est_y+args.y-init_y
                            if (not edge2_detected):
                                z_log = save_z_log(z_log, est.est_z, win_len=args.avg_window)
                                print("not detected")
                            else:
                                mc.stop()
                                detected_pts.append([ptx, pty])
                                write_log("\nSecond edge detected in ({}, {})!\n".format(ptx, pty))
                                break
                        # Another termination condition: the starting point is too close to 2nd edge
                        if (not edge2_detected):
                            mc.stop()
                            detected_pts.append([pstartx, pstarty])
                            write_log("\nedge2 not detected!\n")
                            # move_orient = orient_rotate90(orient_rotate90(move_orient))
                            write_log("\nUse ({}, {}) as edge point!\n".format(pstartx, pstarty))

                        #############################################
                        # Calculate the center and fly towards landing waypoint
                        #############################################
                        write_log("Current moving orient:", move_orient)
                        if (not box_x_computed):
                            land_y = cal_pos
                            _, _, land_x = compute_box_center(detected_pts[1][0],
                                                            detected_pts[1][1],
                                                            move_orient,
                                                            'out')
                            if move_orient == 'e':
                                land_x -= 0.06
                            elif move_orient == 'w':
                                land_x += 0.06
                        elif (not box_y_computed):
                            land_x = cal_pos
                            _, _, land_y = compute_box_center(detected_pts[1][0],
                                                            detected_pts[1][1],
                                                            move_orient,
                                                            'out')
                            if move_orient == 'n':
                                land_y -= 0.06
                            elif move_orient == 's':
                                land_y += 0.06
                        write_log("Landing pad center @ ({}, {})".format(land_x, land_y))
                        des_list = np.array([[land_x, land_y]])

                        # Finally fly towards the goal
                        reached_goal = False
                        des, des_list = des_list[0], des_list[1:]
                        des[0] -= args.x
                        des[1] -= args.y
                        while (not reached_goal):
                            curr_x = est.est_x - init_x
                            curr_y = est.est_y - init_y
                            write_log("position:", curr_x+args.x ,curr_y+args.y)
                            # plotting handle
                            if args.with_visualization:
                                x_data.append((est.est_x - init_x + args.x)*100)
                                y_data.append((est.est_y - init_y + args.y)*100)
                                pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)

                            overall_err = np.sqrt(np.square(des[0]-curr_x)+np.square(des[1]-curr_y))
                            if overall_err < 0.02:
                                reached_goal = False
                                mc.start_linear_motion(0, 0, 0)
                                write_log("Goal reached!")
                                break
                            else:
                                write_log("Not reached!")
                                if overall_err > 0.1:
                                    v_mag = 0.2
                                else:
                                    v_mag = 0.05
                                v_x = v_mag*(des[0]-curr_x)/overall_err
                                v_y = v_mag*(des[1]-curr_y)/overall_err
                                loc_v_x, loc_v_y = glo_2_loc(est.est_yaw, v_x, v_y)
                                mc.start_linear_motion(loc_v_x, loc_v_y, 0)
                                time.sleep(0.25)

                    #############################################
                    # Take off again and fly back to the starting region
                    #############################################
                    # save x & y data for analysis
                    np.savetxt(experiment_name+'_x_half.csv', np.array(x_data), delimiter=',')
                    np.savetxt(experiment_name+'_y_half.csv', np.array(y_data), delimiter=',')

                    time.sleep(0.5)
                    land_final_x = est.est_x - init_x + args.x
                    land_final_y = est.est_y - init_y + args.y
                    mc.land(0.1)
                    time.sleep(1)

                    mc.take_off(0.3,0.2)
                    time.sleep(1)
                    mc._reset_position_estimator()
                    time.sleep(1)
                    init_x = est.est_x
                    init_y = est.est_y
                    init_yaw = est.est_yaw
                    des_x = args.x; des_y = args.y
                    args.x = land_final_x
                    args.y = land_final_y

                    if args.y > map_y/2:
                        mc.move_distance(0,-min(args.y-map_y/2,multiranger.right),0,0.4)
                    elif args.y < map_y/2:
                        mc.move_distance(0,min(map_y/2-args.y,multiranger.left),0,0.4)
                    mc.stop()
                    #############################################
                    # Navigate to the start region with obstacle avoidance
                    #############################################
                    # Initial parameters
                    no_right = False
                    no_left = False
                    obstacle_back = False

                    while des_x - (est.est_x - init_x + args.x) < -0.15:
                        print("Now position is:", est.est_x - init_x + args.x,est.est_y - init_y + args.y)
                        print("Desired x of region is:", des_x)
                        time.sleep(0.5)
                        if is_close(multiranger.up):
                            print("force quit")
                            break

                        if multiranger.back < 0.25:
                            obstacle_back = True

                        if obstacle_back or no_left or no_right:
                            print("back unsafe")

                            obstacle_back = False
                            back_safe = False

                            start_y = est.est_y
                            start_x = est.est_x

                            # if est.est_y - init_y + args.y <  map_y/2:
                            if est.est_y - init_y + args.y < map_y/2:
                                while est.est_y - init_y + args.y + 0.15 < map_y:
                                    if no_left:
                                        no_left = False
                                        break
                                    print("left search")
                                    if is_close(multiranger.up) or is_close(multiranger.left):
                                        print("left unsafe")
                                        break
                                    mc.start_linear_motion(0,0.2,0)
                                    time.sleep(0.1)
                                    if args.with_visualization:
                                        x_data.append((est.est_x - init_x+args.x)*100)
                                        y_data.append((est.est_y - init_y+args.y)*100)
                                        pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                                    if multiranger.back > 0.35:
                                        back_safe_d = multiranger.back
                                        back_safe = True
                                        mc.move_distance(0,0.05,0,0.05)
                                        print("left search found safe path")
                                        break

                                if not back_safe:
                                    if start_y != est.est_y:
                                        mc.move_distance(0,start_y - est.est_y,0,0.2)
                                        mc.move_distance(start_x - est.est_x,start_y - est.est_y,0,0.05)
                                    while est.est_y - init_y + args.y - 0.15 > 0:
                                        if no_right:
                                            no_right = False
                                            break
                                        print("right search")
                                        if is_close(multiranger.up) or is_close(multiranger.right):
                                            print("right unsafe")
                                            break
                                        mc.start_linear_motion(0,-0.2,0)
                                        time.sleep(0.1)
                                        if args.with_visualization:
                                            x_data.append((est.est_x - init_x+args.x)*100)
                                            y_data.append((est.est_y - init_y+args.y)*100)
                                            pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                                        if multiranger.back > 0.35:
                                            back_safe_d = multiranger.back
                                            back_safe = True
                                            mc.move_distance(0,-0.05,0,0.05)
                                            print("right search found safe path")
                                            break
                            else:
                                while est.est_y - init_y + args.y - 0.15 > 0:
                                    if no_right:
                                        print("no right")
                                        no_right = False
                                        break
                                    print("right search")
                                    if is_close(multiranger.up) or is_close(multiranger.right):
                                        print("right unsafe")
                                        break
                                    mc.start_linear_motion(0,-0.2,0)
                                    time.sleep(0.1)
                                    if args.with_visualization:
                                        x_data.append((est.est_x - init_x+args.x)*100)
                                        y_data.append((est.est_y - init_y+args.y)*100)
                                        pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                                    if multiranger.back > 0.35:
                                        back_safe_d = multiranger.back
                                        back_safe = True
                                        mc.move_distance(0,-0.05,0,0.05)
                                        print("right search found safe path")
                                        break

                                if not back_safe:
                                    if start_y != est.est_y:
                                        mc.move_distance(0,start_y - est.est_y,0,0.2)
                                        mc.move_distance(start_x - est.est_x,start_y - est.est_y,0,0.05)
                                    while est.est_y - init_y + args.y + 0.15 < map_y:
                                        if no_left:
                                            print("no left")
                                            no_left = False
                                            break
                                        print("left search")
                                        if is_close(multiranger.up) or is_close(multiranger.left):
                                            print("left unsafe")
                                            break
                                        mc.start_linear_motion(0,0.2,0)
                                        time.sleep(0.1)
                                        if args.with_visualization:
                                            x_data.append((est.est_x - init_x+args.x)*100)
                                            y_data.append((est.est_y - init_y+args.y)*100)
                                            pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                                        if multiranger.back > 0.35:
                                            back_safe_d = multiranger.back
                                            back_safe = True
                                            mc.move_distance(0,0.05,0,0.05)
                                            print("right search found safe path")
                                            break
                            if not back_safe:
                                print("I am trapped")
                                break
                        else:
                            back_safe_d = multiranger.back

                        if multiranger.left < 0.1 and multiranger.right > 0.15:
                            mc.stop()
                            mc.move_distance(0,-min(0.08,multiranger.right-0.1),0,0.2)
                            if args.with_visualization:
                                x_data.append((est.est_x - init_x+args.x)*100)
                                y_data.append((est.est_y - init_y+args.y)*100)
                                pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                        elif multiranger.right < 0.1 and multiranger.left > 0.15:
                            mc.stop()
                            mc.move_distance(0,min(0.08,multiranger.left-0.1),0,0.2)
                            if args.with_visualization:
                                x_data.append((est.est_x - init_x+args.x)*100)
                                y_data.append((est.est_y - init_y+args.y)*100)
                                pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)

                        mc.turn_right(15)
                        mc.stop()
                        time.sleep(0.5)
                        if multiranger.back < 0.2/np.cos(15*np.pi/180)+0.15:
                            regulate_yaw(mc, init_yaw, est.est_yaw)
                            # mc._reset_position_estimator
                            time.sleep(0.5)
                            if is_close(multiranger.right,0.25) or est.est_y - init_y+args.y < 0.25:
                                print("right is not available")
                                no_right = True
                                continue
                            else:
                                mc.move_distance(0,-0.2,0,0.2)
                                time.sleep(0.5)
                                if args.with_visualization:
                                    x_data.append((est.est_x - init_x+args.x)*100)
                                    y_data.append((est.est_y - init_y+args.y)*100)
                                    pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                                back_safe_d = min(back_safe_d,multiranger.back)
                                mc.stop()
                        else:
                            mc.turn_left(30)
                            # mc._reset_position_estimator
                            mc.stop()
                            time.sleep(0.5)
                            if multiranger.back < 0.2/np.cos(15*np.pi/180)+0.15:
                                regulate_yaw(mc, init_yaw, est.est_yaw)
                                time.sleep(0.5)
                                if is_close(multiranger.left,0.25) or map_y - (est.est_y - init_y+args.y) < 0.25:
                                    print("left is not available")
                                    no_left = True
                                    continue
                                else:
                                    mc.move_distance(0,0.2,0,0.2)
                                    time.sleep(0.5)
                                    if args.with_visualization:
                                        x_data.append((est.est_x - init_x+args.x)*100)
                                        y_data.append((est.est_y - init_y+args.y)*100)
                                        pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                                    back_safe_d = min(back_safe_d,multiranger.back)
                                    mc.stop()
                            regulate_yaw(mc, init_yaw, est.est_yaw)
                            time.sleep(0.5)
                            # mc._reset_position_estimator


                        go_d = min(-(des_x - (est.est_x - init_x + args.x)),back_safe_d-0.25,0.5)
                        if back_safe_d-0.25 == go_d:
                            obstacle_back = True
                        print("back safe, go:", go_d)
                        nav_des_x = est.est_x - go_d
                        nav_des_y = est.est_y
                        err_nav = 100
                        while err_nav > 0.12:
                            if multiranger.left < 0.1 and multiranger.right > 0.15:
                                mc.stop()
                                mc.move_distance(0,-min(0.2,multiranger.right-0.1),0,0.2)
                                if args.with_visualization:
                                    x_data.append((est.est_x - init_x+args.x)*100)
                                    y_data.append((est.est_y - init_y+args.y)*100)
                                    pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                            elif multiranger.right < 0.1 and multiranger.left > 0.15:
                                mc.stop()
                                mc.move_distance(0,min(0.2,multiranger.left-0.1),0,0.2)
                                if args.with_visualization:
                                    x_data.append((est.est_x - init_x+args.x)*100)
                                    y_data.append((est.est_y - init_y+args.y)*100)
                                    pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                            mc.start_linear_motion(-0.4,0,0)
                            time.sleep(0.05)
                            if args.with_visualization:
                                x_data.append((est.est_x - init_x+args.x)*100)
                                y_data.append((est.est_y - init_y+args.y)*100)
                                pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                            err_nav = abs(nav_des_x-est.est_x)
                        if multiranger.left < 0.1 and multiranger.right > 0.15:
                            mc.stop()
                            mc.move_distance(0,-min(0.2,multiranger.right-0.1),0,0.2)
                            if args.with_visualization:
                                x_data.append((est.est_x - init_x+args.x)*100)
                                y_data.append((est.est_y - init_y+args.y)*100)
                                pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                        elif multiranger.right < 0.1 and multiranger.left > 0.15:
                            mc.stop()
                            mc.move_distance(0,min(0.2,multiranger.left-0.1),0,0.2)
                            if args.with_visualization:
                                x_data.append((est.est_x - init_x+args.x)*100)
                                y_data.append((est.est_y - init_y+args.y)*100)
                                pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                        mc.move_distance(nav_des_x-est.est_x,0,0,0.1)
                        if args.with_visualization:
                            x_data.append((est.est_x - init_x+args.x)*100)
                            y_data.append((est.est_y - init_y+args.y)*100)
                            pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                        mc.stop()

                    while abs(des_x - (est.est_x - init_x + args.x)) > 0.01:
                        if is_close(multiranger.up):
                            break
                        mc.move_distance(des_x - (est.est_x - init_x + args.x), 0, 0, 0.05)
                        if args.with_visualization:
                            x_data.append((est.est_x - init_x+args.x)*100)
                            y_data.append((est.est_y - init_y+args.y)*100)
                            pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)

                    print("Arrive desired x")
                    print("Now pos:",args.x+est.est_x-init_x,args.y+est.est_y-init_y)

                    z_log = []
                    mc.stop()
                    for _ in range(4):
                        time.sleep(0.2)
                        z_log = save_z_log(z_log, est.est_z, win_len=args.avg_window)
                    final_edge_detected = False
                    search_time = 0
                    if des_y > est.est_y-init_y+args.y:
                        while search_time < 3:
                            search_time += 1
                            while est.est_y-init_y+args.y < map_y - 0.3:
                                if multiranger.left < 0.15:
                                    break
                                mc.start_linear_motion(0,0.1,0)
                                time.sleep(0.2)
                                if args.with_visualization:
                                    x_data.append((est.est_x - init_x+args.x)*100)
                                    y_data.append((est.est_y - init_y+args.y)*100)
                                    pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                                final_edge_detected = detect_box_edge(z_log, est.est_z, args.box_threshold)
                                if final_edge_detected:
                                    print("find final edge")
                                    mc.stop()
                                    mc.move_distance(0,0.1,0,0.1)
                                    search_time = 100
                                    break
                                else:
                                    z_log = save_z_log(z_log, est.est_z, win_len=args.avg_window)
                            if not final_edge_detected:
                                while est.est_y-init_y+args.y > 0.3:
                                    if multiranger.right < 0.15:
                                        break
                                    mc.start_linear_motion(0,-0.1,0)
                                    time.sleep(0.2)
                                    if args.with_visualization:
                                        x_data.append((est.est_x - init_x+args.x)*100)
                                        y_data.append((est.est_y - init_y+args.y)*100)
                                        pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                                    final_edge_detected = detect_box_edge(z_log, est.est_z, args.box_threshold)
                                    if final_edge_detected:
                                        print("find final edge")
                                        mc.stop()
                                        mc.move_distance(0,-0.1,0,0.1)
                                        search_time = 100
                                        break
                                    else:
                                        z_log = save_z_log(z_log, est.est_z, win_len=args.avg_window)

                            time.sleep(0.5)
                            while multiranger.back < 0.3 and est.est_y-init_y+args.y < map_y - 0.3:
                                mc.start_linear_motion(0,0.1,0)
                                time.sleep(0.05)

                            while est.est_x - init_x + args.x > des_x - 0.2*search_time:
                                mc.start_linear_motion(-0.1,0,0)
                                time.sleep(0.2)
                                if args.with_visualization:
                                    x_data.append((est.est_x - init_x+args.x)*100)
                                    y_data.append((est.est_y - init_y+args.y)*100)
                                    pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                                final_edge_detected = detect_box_edge(z_log, est.est_z, args.box_threshold)
                                if final_edge_detected:
                                    print("find final edge")
                                    mc.stop()
                                    mc.move_distance(0.1,0,0,0.1)
                                    search_time = 100
                                    break
                                else:
                                    z_log = save_z_log(z_log, est.est_z, win_len=args.avg_window)
                    else:
                        while search_time < 3:
                            search_time += 1
                            while est.est_y-init_y+args.y > 0.3:
                                if multiranger.right < 0.15:
                                    break
                                mc.start_linear_motion(0,-0.1,0)
                                time.sleep(0.2)
                                if args.with_visualization:
                                    x_data.append((est.est_x - init_x+args.x)*100)
                                    y_data.append((est.est_y - init_y+args.y)*100)
                                    pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                                final_edge_detected = detect_box_edge(z_log, est.est_z, args.box_threshold)
                                if final_edge_detected:
                                    print("find final edge")
                                    mc.stop()
                                    mc.move_distance(0,-0.1,0,0.1)
                                    search_time = 100
                                    break
                                else:
                                    z_log = save_z_log(z_log, est.est_z, win_len=args.avg_window)
                            if not final_edge_detected:
                                while est.est_y-init_y+args.y < map_y - 0.3:
                                    if multiranger.left < 0.15:
                                        break
                                    mc.start_linear_motion(0,0.1,0)
                                    time.sleep(0.2)
                                    if args.with_visualization:
                                        x_data.append((est.est_x - init_x+args.x)*100)
                                        y_data.append((est.est_y - init_y+args.y)*100)
                                        pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                                    final_edge_detected = detect_box_edge(z_log, est.est_z, args.box_threshold)
                                    if final_edge_detected:
                                        print("find final edge")
                                        mc.stop()
                                        mc.move_distance(0,0.1,0,0.1)
                                        search_time = 100
                                        break
                                    else:
                                        z_log = save_z_log(z_log, est.est_z, win_len=args.avg_window)

                            time.sleep(0.5)
                            while multiranger.back < 0.3 and est.est_y-init_y+args.y > 0.3:
                                mc.start_linear_motion(0,-0.1,0)
                                time.sleep(0.05)

                            while est.est_x - init_x + args.x > des_x - 0.2*search_time:
                                mc.start_linear_motion(-0.1,0,0)
                                time.sleep(0.2)
                                if args.with_visualization:
                                    x_data.append((est.est_x - init_x+args.x)*100)
                                    y_data.append((est.est_y - init_y+args.y)*100)
                                    pos = live_plotter(x_data, y_data, pos, pause_time, overall_region_w, overall_region_h)
                                final_edge_detected = detect_box_edge(z_log, est.est_z, args.box_threshold)
                                if final_edge_detected:
                                    print("find final edge")
                                    mc.stop()
                                    mc.move_distance(0.1,0,0,0.1)
                                    search_time = 100
                                    break
                                else:
                                    z_log = save_z_log(z_log, est.est_z, win_len=args.avg_window)

                    mc.stop()

            # save x & y data for analysis
            np.savetxt(experiment_name+'_x.csv', np.array(x_data), delimiter=',')
            np.savetxt(experiment_name+'_y.csv', np.array(y_data), delimiter=',')

            print('Task finished!')
    log_file.close()
