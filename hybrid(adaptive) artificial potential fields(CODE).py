# -*- coding: utf-8 -*-
"""
Created on Mon Nov 23 00:13:50 2020

@author: Nikhil
@BINFORD ROBOTICS

Potential Field based planner

"""

from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
import serial 
import argparse 
from dronekit import connect, VehicleMode, InternalGlobalRelative, LocationGlobalRelative 
from pymavlink import mavutil
import time

parser=argparse.ArgumentParser()
parser.add_argument('--connect',default='127.0.0.1:14550')
args=parser.parse_args()
vehicle=connect(args.connect, baud=57600,wait_ready=True )
ser=serial.Serial("/dev/ttyACM0",9600,timeout=1)

# first let's arm the vehicle with the RC controller and lift it to a required height"
"""
Drone takes off and reaches a height 
"""

# Parameters
KP = 5.0  # attractive potential gain
ETA = 100.0  # repulsive potential gain
AREA_WIDTH = 30.0  # potential area width [m]
# the number of previous positions used to check oscillations
OSCILLATIONS_DETECTION_LENGTH = 3

show_animation = True


oX = [-5]
oY = [0]


def calc_potential_field(gx, gy, ox, oy, reso, rr, sx, sy):
    minx = min(min(ox), sx, gx) - AREA_WIDTH / 2.0
    miny = min(min(oy), sy, gy) - AREA_WIDTH / 2.0
    maxx = max(max(ox), sx, gx) + AREA_WIDTH / 2.0
    maxy = max(max(oy), sy, gy) + AREA_WIDTH / 2.0
    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))

    # calc each potential
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]

    for ix in range(xw):
        x = ix * reso + minx

        for iy in range(yw):
            y = iy * reso + miny
            ug = calc_attractive_potential(x, y, gx, gy)
            uo = calc_repulsive_potential(x, y, ox, oy, rr)
            uf = ug + uo
            pmap[ix][iy] = uf

    return pmap, minx, miny


def calc_attractive_potential(x, y, gx, gy):
    return 0.5 * KP * np.hypot(x - gx, y - gy)


def calc_repulsive_potential(x, y, ox, oy, rr):
    # search nearest obstacle
    minid = -1
    dmin = float("inf")
    for i, _ in enumerate(ox):
        d = np.hypot(x - ox[i], y - oy[i])
        if dmin >= d:
            dmin = d
            minid = i

    # calc repulsive potential
    dq = np.hypot(x - ox[minid], y - oy[minid])

    if dq <= rr:
        if dq <= 0.1:
            dq = 0.1

        return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2
    else:
        return 0.0


def get_motion_model():
    # dx, dy
    motion = [[1, 0],
              [0, 1],
              [-1, 0],
              [0, -1],
              [-1, -1],
              [-1, 1],
              [1, -1],
              [1, 1]]

    return motion


def oscillations_detection(previous_ids, ix, iy):
    previous_ids.append((ix, iy))

    if (len(previous_ids) > OSCILLATIONS_DETECTION_LENGTH):
        previous_ids.popleft()

    # check if contains any duplicates by copying into a set
    previous_ids_set = set()
    for index in previous_ids:
        if index in previous_ids_set:
            return True
        else:
            previous_ids_set.add(index)
    return False

def potential_field_planning(sx, sy, gx, gy, ox, oy, reso, rr):
    
    # calc potential field
    pmap, minx, miny = calc_potential_field(gx, gy, ox, oy, reso, rr, sx, sy)
    initi_distX = 0
    initi_distY = 0
     
    # search path
    d = np.hypot(sx - gx, sy - gy)
    ix = round((sx - minx) / reso)
    iy = round((sy - miny) / reso)
    gix = round((gx - minx) / reso)
    giy = round((gy - miny) / reso)

    if show_animation:
        draw_heatmap(pmap)
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(ix, iy, "*k")
        plt.plot(gix, giy, "*m")
        
    rx, ry = [sx], [sy]
    motion = get_motion_model()
    previous_ids = deque()


    while d >= reso:
        #check if there is an obstacle
        i = int(ser.readline().decode('utf-8').rstrip())
        if i < 90:
            oX.append(sx + i)
            oY.append(sy)
        else:
            dummy = 1
        # calc potential field
        pmap, minx, miny = calc_potential_field(gx, gy, ox, oy, reso, rr, sx, sy)
        # search path
        d = np.hypot(sx - gx, sy - gy)
        ix = round((sx - minx) / reso)
        iy = round((sy - miny) / reso)
        gix = round((gx - minx) / reso)
        giy = round((gy - miny) / reso)
        if show_animation:
            draw_heatmap(pmap)
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                   lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(ix, iy, "*k")
            plt.plot(gix, giy, "*m")
        minp = float("inf")
        minix, miniy = -1, -1
        for i, _ in enumerate(motion):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])
            if inx >= len(pmap) or iny >= len(pmap[0]) or inx < 0 or iny < 0:
                p = float("inf")  # outside area
                print("outside potential!")
            else:
                p = pmap[inx][iny]
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
        ix = minix
        iy = miniy
        xp = ix * reso + minx
        yp = iy * reso + miny
        sx = xp
        sy = yp
        V_x = sx - initi_distX #speed in M/s
        V_y = sy - initi_distY #speed in M/s
        """
        Over ride the RC using, dronekit channel overrides
        Channel 2 = pitch , which controls the x velocity
        Channel 1 = roll, which controls the y velocity
        Pass the velocities in pwm valued, that are equivalent to the 0.5 m/s velocity by overriding the channels
        """
        time_duration = 1
        vehicle.channels.overrides = {'1':'pwm values', '2':'pwm value'}
        print("velocity in x :", V_x)
        print("velocity in y :", V_y)
        print(sx)
        print(sy)
        initi_distX = sx
        initi_distY = sy
        d = np.hypot(gx - xp, gy - yp)
        rx.append(xp)
        ry.append(yp)

        if (oscillations_detection(previous_ids, ix, iy)):
            print("Oscillation detected at ({},{})!".format(ix, iy))
            

        if show_animation:
            plt.plot(ix, iy, ".r")
            plt.pause(0.01)

    print("Goal!!")

    return rx, ry


def draw_heatmap(data):
    data = np.array(data).T
    plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)

def main():
    print("potential_field_planning start")

    sx = 0.0  # start x position [m]
    sy = 0.0  # start y positon [m]
    gx = 4.0  # goal x position [m]
    gy = 0.0  # goal y position [m]
    grid_size = 0.5  # potential grid size [m]
    robot_radius = 0.8  # robot radius [m] = obstacle radius[m] + extra gap[m] + drone radius[m]

    ox = oX #[14,15,16,14,15,16,14,15,16]  # obstacle x position list [m]
    oy = oY #[0,0,0,-0.5,-0.5,-0.5,0.5,0.5,0.5]  # obstacle y position list [m]
    
    if show_animation:
        plt.grid(True)
        plt.axis("equal")
    
    # path generation
    arm_and_takeoff(1)
    
    _, _ = potential_field_planning(
        sx, sy, gx, gy, ox, oy, grid_size, robot_radius)
    
    vehicle.mode = VehicleMode("LAND")
    
    if show_animation:
        plt.show()


if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")

