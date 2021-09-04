# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2018 Bitcraze AB
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
This script shows the basic use of the PositionHlCommander class.

Simple example that connects to the crazyflie at `URI` and runs a
sequence. This script requires some kind of location system.

The PositionHlCommander uses position setpoints.

Change the URI variable to your Crazyflie configuration.
"""
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils.multiranger import Multiranger
import logging
import time
import cflib.crtp
import math
from cflib.crazyflie.log import LogConfig

# Global variables
MIN_DISTANCE = 0.3  # m
MIN_DISTANCE_FRONT = 0.3  # m
MIN_DISTANCE_BACK = 0.3  # m
MIN_DISTANCE_LEFT = 0.3  # m
MIN_DISTANCE_RIGHT = 0.3  # m

EPSILON = 0.1
EPSILON_GOAL = 0.1
STEP = 0.03  # m
STEP_AVOIDANCE = 0.2  # m
HEIGHT = 0.5  # m
X_START = 0.75  # m
Y_START = 1.5  # m
VELOCITY = 0.4
ANGLE_CHECK = 5*math.pi/180
TURN_TIME = 1
UP = 1
DOWN = 0
STEP_X_LIST = 0.3
STEP_Y_LIST = 0.3
THRESHOLD_Z = 0.03
THRESHOLD_Z_REF = 0.03
THRESHOLD_Z_CHANGE = 0.005
NB_Z_VALUES = 40
NB_MIN_VALUES_IN_LIST = 20
VEL_GOTO = 0.3


# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7E7'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
position_estimate = [0, 0, 0]


def comeback_de_folie(x, y, z, multiranger, velocity, pc, thresh_z):
    """
        Go to a given position and detect the starting pad to land.
        Used to go back to the goal and detect it.

        :param x: X coordinate
        :param y: Y coordinate
        :param z: Z coordinate
        :param multiranger: Quadcopter multiranger
        :param velocity: the velocity (meters/second)
        :param thresh_z: Threshold between two peak of down multiranger to detect the landing pad
        :return obs: Boolean to say if the landing pad has been detected
        :return last: Direction of arriving when pad is detected
        :return step_x: Estimation of the step coordinate according to X
        :return step_y: Estimation of the step coordinate according to Y
        """

    z = pc._height(z)
    dx = x - pc._x
    dy = y - pc._y
    dz = z - pc._z
    last = 0
    distance = math.sqrt(dx * dx + dy * dy + dz * dz)
    obs = False
    step_x = 0
    step_y = 0
    if distance > 0.0:
        speed = pc._velocity(velocity)
        duration_s = distance / speed
        pc._hl_commander.go_to(x, y, z, 0, duration_s)
        start_timer = time.time()
        end_timer = start_timer
        print('dx,dy,dz', dx, dy, dz)
        z_value = []
        delta_z = 0
        while (end_timer-start_timer) < duration_s:

            # Fill up a list with the multiranger.down value
            if (len(z_value) < NB_Z_VALUES) and (multiranger.down is not None):
                z_value.append(multiranger.down)

            elif (len(z_value) >= NB_Z_VALUES) and (multiranger.down is not None):
                z_value.pop(0)
                z_value.append(multiranger.down)

            if len(z_value) >= NB_MIN_VALUES_IN_LIST:
                min_z = min(z_value)
                max_z = max(z_value)
                delta_z = max_z - min_z

            # Check if the min/max difference is over a certain threshold hence indicating a step detection
            print('thresh z',thresh_z)
            if delta_z > thresh_z and (len(z_value) >= NB_MIN_VALUES_IN_LIST):
                print('obs')
                # Keep position estimate of step coordinate
                step_x = position_estimate[0]
                step_y = position_estimate[1]
                obs = True
                end_timer = time.time()
                timer = (end_timer - start_timer)
                # Sleep the remaining time to reach the desired position in order to keep good drone tracking
                time.sleep(duration_s - timer)
                break
            end_timer = time.time()

        pc._x = x
        pc._y = y
        pc._z = z

        # Check the different incoming direction when detecting the landing pad
        if dy == 0 and obs and dx > 0:
            print('dy = 0 and dx pos')
            pc.go_to(step_x+0.08, y, 0.5, 0.2)
            last = 1

        elif dx == 0 and obs and dy < 0:
            print('dx =0 dy neg')
            pc.go_to(x, step_y+0.05, 0.5, 0.2)
            time.sleep(1)
            last = 2

        elif dx == 0 and obs and dy > 0:
            print('dx =0 y pos')
            pc.go_to(x, step_y-0.04, 0.5, 0.2)
            last = 3

    return obs, last, step_x, step_y


def go_to_de_folie2(x, y, z, multiranger, velocity, pc, thresh_z):
    """
        Go to a given position and detect the goal pad to land.
        Used to go from starting pad to goal pad.

        :param x: X coordinate
        :param y: Y coordinate
        :param z: Z coordinate
        :param multiranger: Quadcopter multiranger
        :param velocity: the velocity (meters/second)
        :param thresh_z: Threshold between two peak of down multiranger to detect the landing pad
        :return obs: Boolean to say if the landing pad has been detected
        :return last: Direction of arriving when pad is detected
        :return step_x: Estimation of the step coordinate according to X
        :return step_y: Estimation of the step coordinate according to Y
        """

    z = pc._height(z)
    dx = x - pc._x
    dy = y - pc._y
    dz = z - pc._z
    last = 0
    distance = math.sqrt(dx * dx + dy * dy + dz * dz)
    obs = False
    step_x = 0
    step_y = 0
    if distance > 0.0:
        speed = pc._velocity(velocity)
        duration_s = distance / speed
        pc._hl_commander.go_to(x, y, z, 0, duration_s)
        start_timer = time.time()
        end_timer = start_timer
        print('dx,dy,dz', dx, dy, dz)
        z_value = []
        delta_z = 0
        while (end_timer-start_timer) < duration_s:

            # Fill up a list with the multiranger.down value
            if (len(z_value) < NB_Z_VALUES) and (multiranger.down is not None):
                z_value.append(multiranger.down)

            elif (len(z_value) >= NB_Z_VALUES) and (multiranger.down is not None):
                z_value.pop(0)
                z_value.append(multiranger.down)

            if len(z_value) >= NB_MIN_VALUES_IN_LIST:
                min_z = min(z_value)
                max_z = max(z_value)
                delta_z = max_z - min_z

            # Check if the min/max difference is over a certain threshold hence indicating a step detection
            print('thresh z',thresh_z)
            if delta_z > thresh_z and (len(z_value) >= NB_MIN_VALUES_IN_LIST):
                print('obs')
                # Keep position estimate of step coordinate
                step_x = position_estimate[0]
                step_y = position_estimate[1]
                obs = True
                end_timer = time.time()
                timer = (end_timer - start_timer)
                # Sleep the remaining time to reach the desired position in order to keep good drone tracking
                time.sleep(duration_s - timer)

                break
            end_timer = time.time()

        pc._x = x
        pc._y = y
        pc._z = z

        # Check the different incoming direction when detecting the landing pad
        if dy == 0 and obs and dx > 0:
            print('dy = 0 and dx pos')
            pc.go_to(step_x+0.05, y, 0.5, 0.2)
            last = 1

        if dx == 0 and obs and dy < 0:
            print('dx =0 dy neg')
            pc.go_to(x, step_y-0.05, 0.5, 0.2)
            time.sleep(1)
            last = 2

        if dx == 0 and obs and dy > 0:
            print('dx =0 y pos')
            pc.go_to(x, step_y-0.04, 0.5, 0.2)
            last = 3

    return obs, last, step_x, step_y


def landing_pad(last, pc, mlt, step_1_x, step_1_y):
    """
    When first edge of the pad is detected, comes here to detect the edges according to the other axis.
    :param last: Direction when reaching the first step
    :param pc: Position commander class
    :param mlt: multiranger info
    :param step_1_x: Estimation of the step coordinate according to X when first reaching the pad
    :param step_1_y: Estimation of the step coordinate according to Y when first reaching the pad
    :return: True as the landing is detected
    """

    global THRESHOLD_Z
    print('last')

    # Determine the next axis to search the step
    if last == 1:
        x, y, z = pc.get_position()
        y += 0.7
        time.sleep(0.1)
        obs_y = False
        thresh_z = THRESHOLD_Z
        print('before while')

        # Search for the second edge
        while not obs_y:
            pc.go_to(x, y,z,0.2)
            time.sleep(0.5)
            obs_y, _, _, _ = go_to_de_folie2(x, y - 1.5, z, mlt, 0.4, pc, thresh_z)

            # Change threshold if the edge has not been detected
            thresh_z -= THRESHOLD_Z_CHANGE
            print('thresh 1', thresh_z)

    elif last == 2:
        x, y, z = pc.get_position()
        x -= 0.7
        thresh_z = THRESHOLD_Z
        obs_y = False

        # Search for the second edge
        while not obs_y:
            pc.go_to(x, y,z,0.2)
            time.sleep(0.5)
            obs_y, _, step_1_x, step_1_y = go_to_de_folie2(x+1.5, y, z, mlt, 0.4, pc, thresh_z)

            # Change threshold if the edge has not been detected
            thresh_z -= THRESHOLD_Z_CHANGE
            print('thresh 2', thresh_z)
    elif last == 3:
        x, y, z = pc.get_position()
        x -= 0.7
        thresh_z = THRESHOLD_Z
        obs_y = False

        # Search for the second edge
        while not obs_y:
            pc.go_to(x, y,z,0.2)
            time.sleep(0.5)
            obs_y, _, step_1_x, step_1_y = go_to_de_folie2(x + 1.5, y, z, mlt, 0.4, pc, thresh_z)

            # Change threshold if the edge has not been detected
            thresh_z -= THRESHOLD_Z_CHANGE
            print('thresh 3', thresh_z)

    THRESHOLD_Z = THRESHOLD_Z_REF
    return True


def get_estimate():
    """
    Return the position estimate of the drone
    :return: position x,y,z
    """
    return position_estimate[0], position_estimate[1], position_estimate[2]


def is_close(range):
    """
    Return True if the value of range (multiranger info) are smaller than a Threshold
    :param range: Value to check
    :return: Return True if the value is smaller, False otherwise
    """
    if range is None:
        return False
    else:
        return range < MIN_DISTANCE


def is_close_front(range):
    """
    Return True if the value of range (multiranger front sensor) is smaller than a Threshold
    :param range: Value to check
    :return: Return True if the value is smaller, False otherwise
    """
    if range is None:
        return False
    else:
        return range < MIN_DISTANCE_FRONT


def is_close_back(range):
    """
    Return True if the value of range (multiranger back sensor) is smaller than a Threshold
    :param range: Value to check
    :return: Return True if the value is smaller, False otherwise
    """
    if range is None:
        return False
    else:
        return range < MIN_DISTANCE_BACK


def is_close_right(range):
    """
    Return True if the value of range (multiranger right sensor) is smaller than a Threshold
    :param range: Value to check
    :return: Return True if the value is smaller, False otherwise
    """
    if range is None:
        return False
    else:
        return range < MIN_DISTANCE_RIGHT


def is_close_left(range):
    """
    Return True if the value of range (multiranger left sensor) is smaller than a Threshold
    :param range: Value to check
    :return: Return True if the value is smaller, False otherwise
    """
    if range is None:
        return False
    else:
        return range < MIN_DISTANCE_LEFT


def avoidance_move_y(delta_y, mlt, pc):
    """
    Function to handle the quadcopter displacement along Y when no obstacle has been detected
    and the drone is going from start to goal

    :param delta_y: Difference between drone position and goal coordinate according to Y axis
    :param mlt: Multiranger info
    :param pc: position commander class
    :return: True if a landing pad has been detected, False otherwise
    """

    # If the distance to go is smaller than a certain STEP, do a movement of delta y.
    # Otherwise do a step on Right or Left depending on delta_y sign
    # Each step search for the landing pad, if detected land and return the stop signal

    if abs(delta_y) < STEP:
        print('Move delta_y')
        x, y, z = pc.get_position()
        obs, last, s_x, s_y = go_to_de_folie2(x, y+delta_y, z, mlt, VEL_GOTO, pc, THRESHOLD_Z)
        if obs:
            landing = landing_pad(last, pc, multiranger, s_x, s_y)
            if landing:
                print('landing 1')
                pc.land(0.1)
                return True

    else:
        if delta_y > 0:
            print('Move left STEP ')
            x, y, z = pc.get_position()
            obs, last, s_x, s_y = go_to_de_folie2(x, y+STEP, z, mlt, VEL_GOTO, pc, THRESHOLD_Z)
            if obs:
                landing = landing_pad(last, pc, multiranger, s_x, s_y)
                if landing:
                    print('landing 2')
                    pc.land(0.1)
                    return True

        else:
            print('Move right STEP ')
            x, y, z = pc.get_position()
            obs, last, s_x, s_y = go_to_de_folie2(x, y - STEP, z, mlt, VEL_GOTO, pc,THRESHOLD_Z)
            if obs:
                landing = landing_pad(last, pc, multiranger, s_x, s_y)
                if landing:
                    print('landing 3')
                    pc.land(0.1)
                    return True
    return False


def avoidance_move_y_avoid(delta_y, mlt, pc):
    """
    Function to handle the quadcopter displacement along Y when an obstacle has been detected
    and the drone is going from start to goal

    :param delta_y: Difference between drone position and goal coordinate according to Y axis
    :param mlt: Multiranger info
    :param pc: position commander class
    :return: Always return False
    """

    # If the distance to go is smaller than a certain STEP_AVOIDANCE, do a movement of delta y.
    # Otherwise do a step on Right or Left depending on delta_y sign

    if abs(delta_y) < STEP_AVOIDANCE:  # move by delta_y
        print('Move delta_y = {}'.format(delta_y))
        pc.left(delta_y)

    else:
        if delta_y > 0:  # Move from step to left
            print('Move left STEP_AVOIDANCE ')
            pc.left(STEP_AVOIDANCE)

        else:  # Move from step to right
            print('Move right STEP_AVOIDANCE ')
            pc.right(STEP_AVOIDANCE)
    return False


def avoidance_move_x(delta_x, mlt, pc):
    """
    Function to handle the quadcopter displacement along X when no obstacle has been detected
    and the drone is going from start to goal

    :param delta_x: Difference between drone position and goal coordinate according to X axis
    :param mlt: Multiranger info
    :param pc: position commander class
    :return: True if a landing pad has been detected, False otherwise
    """
    # If the distance to go is smaller than a certain STEP, do a movement of delta_x.
    # Otherwise do a step forward or backward depending on delta_x sign
    # Each step search for the landing pad, if detected land and return the stop signal

    if abs(delta_x) < STEP:
        print('Move delta_x')
        x, y, z = pc.get_position()
        obs, last, s_x, s_y = go_to_de_folie2(x+delta_x, y, z, mlt, VEL_GOTO+0.2, pc,THRESHOLD_Z)
        if obs:
            landing = landing_pad(last, pc, multiranger, s_x, s_y)
            if landing:
                print('landing 4')
                pc.land(0.1)
                return True

    else:
        if delta_x > 0:
            print('Move forward')
            x, y, z = pc.get_position()
            obs, last, s_x, s_y = go_to_de_folie2(x + STEP, y, z, mlt, VEL_GOTO+0.2, pc, THRESHOLD_Z)
            if obs:
                landing = landing_pad(last, pc, multiranger,s_x,s_y)
                if landing:
                    print('landing 5')
                    pc.land(0.1)
                    return True


        else:
            print('Move back **************************************')
            x, y, z = pc.get_position()
            obs, last, s_x, s_y = go_to_de_folie2(x - STEP, y, z,mlt, VEL_GOTO+0.2, pc,THRESHOLD_Z)
            if obs:
                landing = landing_pad(last, pc, multiranger, s_x, s_y)
                if landing:
                    print('landing 6')
                    pc.land(0.1)
                    return True
    return False


def avoidance_move_x_avoid(delta_x ,mlt, pc):
    """
    Function to handle the quadcopter displacement along X when an obstacle has been detected
    and the drone is going from start to goal

    :param delta_x: Difference between drone position and goal coordinate according to X axis
    :param mlt: Multiranger info
    :param pc: position commander class
    :return: Always return False
    """
    # If the distance to go is smaller than a certain STEP_AVOIDANCE, do a movement of delta_x.
    # Otherwise do a step forward or backward depending on delta_x sign

    if abs(delta_x) < STEP_AVOIDANCE:
        print('Move delta_x')
        pc.forward(delta_x)

    else:
        if delta_x > 0:
            print('Move forward avoid')
            pc.forward(STEP_AVOIDANCE)

        else:
            print('Move back *******************************')
            pc.back(STEP_AVOIDANCE)
    return False


def avoidance_move_x_comeback(delta_x, mlt, pc):
    """
    Function to handle the quadcopter displacement along X when an obstacle has been detected
    and the drone is going from goal to start

    :param delta_x: Difference between drone position and goal coordinate according to X axis
    :param mlt: Multiranger info
    :param pc: position commander class
    :return:  True if a landing pad has been detected, False otherwise
    """
    # If the distance to go is smaller than a certain STEP, do a movement of delta_x.
    # Otherwise do a step forward or backward depending on delta_x sign
    # Each step search for the landing pad, if detected land and return the stop signal

    if abs(delta_x) < STEP:
        print('Move delta_x')
        x, y, z = pc.get_position()
        obs, last, s_x, s_y = go_to_de_folie2(x+delta_x, y, z, mlt, VEL_GOTO, pc, THRESHOLD_Z)
        if obs:
            landing = landing_pad(last, pc, multiranger, s_x, s_y)
            if landing:
                print('landing 4')
                pc.land(0.1)
                return True

    else:
        if delta_x > 0:
            print('Move forward')
            x, y, z = pc.get_position()
            obs, last, s_x, s_y = go_to_de_folie2(x + STEP, y, z, mlt, VEL_GOTO, pc, THRESHOLD_Z)
            if obs:
                landing = landing_pad(last, pc, multiranger, s_x, s_y)
                if landing:
                    print('landing 5')
                    pc.land(0.1)
                    return True


        else:
            print('Move back **************************************')
            x, y, z = pc.get_position()
            obs, last, s_x, s_y = go_to_de_folie2(x - STEP, y, z, mlt, VEL_GOTO, pc, THRESHOLD_Z)
            if obs:
                landing = landing_pad(last, pc, multiranger, s_x, s_y)
                if landing:
                    print('landing 6')
                    pc.land(0.1)
                    return True
    return False


def avoidance_move_y_comeback(delta_y, mlt, pc):
    """
    Function to handle the quadcopter displacement along Y when an obstacle has been detected
    and the drone is going from start to goal

    :param delta_y: Difference between drone position and goal coordinate according to Y axis
    :param mlt: Multiranger info
    :param pc: position commander class
    :return:  True if a landing pad has been detected, False otherwise
    """
    # If the distance to go is smaller than a certain STEP, do a movement of delta_y.
    # Otherwise do a step on Right or Left depending on delta_y sign
    # Each step search for the landing pad, if detected land and return the stop signal

    if abs(delta_y) < STEP:
        print('Move delta_y')
        x, y, z = pc.get_position()
        obs, last, s_x, s_y = go_to_de_folie2(x, y+delta_y, z, mlt, VEL_GOTO, pc, THRESHOLD_Z)
        if obs:
            landing = landing_pad(last, pc, multiranger, s_x, s_y)
            if landing:
                print('landing 1')
                pc.land(0.1)
                return True

    else:
        if delta_y > 0:
            print('Move left STEP ')
            x, y, z = pc.get_position()
            obs, last, s_x, s_y = go_to_de_folie2(x, y+STEP, z, mlt, VEL_GOTO, pc, THRESHOLD_Z)
            if obs:
                landing = landing_pad(last, pc, multiranger, s_x, s_y)
                if landing:
                    print('landing 2')
                    pc.land(0.1)
                    return True

        else:
            print('Move right STEP ')
            x, y, z = pc.get_position()
            obs, last, s_x, s_y = go_to_de_folie2(x, y - STEP, z, mlt, VEL_GOTO, pc, THRESHOLD_Z)
            if obs:
                landing = landing_pad(last, pc, multiranger, s_x, s_y)
                if landing:
                    print('landing 3')
                    pc.land(0.1)
                    return True
    return False


def go_A_to_B_saved(ultimate_goal, pc, multiranger, back_to_base):
    """
    Global function to link two checkpoints. Handle if an obstacles is detected or not and call the appropriate function
    It is the function called when going from start to goal.

    :param ultimate_goal: Checkpoint coordinate
    :param pc: Position commander info
    :param multiranger: multiranger info
    :param back_to_base: Say if the drone is going back or not
    :return: If the landing has been completed
    """

    # Control Y position
    y_complete = False
    avoid = False
    finish_landed = False
    while not y_complete and not finish_landed:
        _, y, _ = pc.get_position()
        delta_y = ultimate_goal[1] - y

        # Handle the detection of an obstacle in Y direction. First if handle if an obstacle has already been detected
        if avoid:
            while abs(delta_y) > EPSILON:
                _, y, _ = pc.get_position()
                delta_y = ultimate_goal[1] - y

                # Depending on the on going direction as well as multiranger values, choose which direction to go
                if (is_close(multiranger.right) and delta_y < 0) or (is_close(multiranger.left) and delta_y > 0):
                    if delta_y < 0:
                        pc.left(STEP_AVOIDANCE)
                        time.sleep(0.1)
                    else:
                        pc.right(STEP_AVOIDANCE)
                        time.sleep(0.1)
                    pc.forward(STEP_AVOIDANCE)
                    time.sleep(0.1)
                else:
                    if is_close(multiranger.back):
                        pc.forward(STEP_AVOIDANCE)

                    pc.forward(STEP_AVOIDANCE)
                    finish_landed = avoidance_move_y_avoid(delta_y, multiranger, pc)
                time.sleep(1)
            avoid = False
            y_complete = True

            # Adjust to X coordinate of the checkpoint
            print("Phase 2")
            x_now, _, _ = pc.get_position()
            delta_x = x_saved - x_now
            while abs(delta_x) > EPSILON and ((not is_close_front(multiranger.front) and delta_x>0) or (not is_close(multiranger.back) and delta_x < 0)):
                finish_landed = avoidance_move_x_avoid(delta_x,multiranger,pc)

                time.sleep(1)
                x_now, _, _ = pc.get_position()
                delta_x = x_saved - x_now

            print("End ---------------------")

        # Handle if it an obstacle was not seen before on Y axis but is seen now
        elif not avoid and ((delta_y < 0 and is_close(multiranger.right)) or (delta_y > 0 and is_close(multiranger.left))):  # condition to enter avoidance
            print('Phase 1 -----------------')
            x_saved, _, _ = pc.get_position()
            avoid = True
        # Handle Y movement when no obstacle has been/was detected in Y axis
        else:
            if abs(delta_y) > EPSILON:
                finish_landed = avoidance_move_y(delta_y, multiranger, pc)
            else:
                print('Arrived Y')
                y_complete = True

    print('Y complete, begin X -------------------------------------------')

    # Control X position
    x_complete = False
    avoid = False
    LEFT = 0
    RIGHT = 1
    Y_MIDDLE_MAP = absolute_to_relative_y(1.7)

    while not x_complete and not finish_landed:

        print("Front:{}".format(multiranger.front))
        x, y, _ = pc.get_position()
        delta_x = ultimate_goal[0] - x
        # Handle the detection of an obstacle in X direction. First if handle if an obstacle has already been detected
        if avoid:
            while abs(delta_x) > EPSILON:
                if (is_close(multiranger.back) and delta_x < 0) or (is_close_front(multiranger.front) and delta_x > 0):
                    if delta_x < 0:
                        pc.forward(STEP_AVOIDANCE)
                        time.sleep(0.1)
                    else:
                        pc.back(STEP_AVOIDANCE)
                        time.sleep(0.1)

                    if x_avoid_direction == RIGHT:
                        pc.right(STEP_AVOIDANCE)
                        time.sleep(0.1)
                    else:
                        pc.left(STEP_AVOIDANCE)
                        time.sleep(0.1)
                else:
                    if x_avoid_direction == RIGHT:
                        if is_close(multiranger.left):
                            print('wall too close on the left')
                            pc.right(STEP_AVOIDANCE)
                            time.sleep(0.1)

                    if x_avoid_direction == LEFT:
                        if is_close(multiranger.right):
                            print('wall too close on the right')
                            pc.left(STEP_AVOIDANCE)
                            time.sleep(0.1)

                    print('Probel delta_x')
                    print(delta_x)
                    finish_landed = avoidance_move_x_avoid(delta_x, multiranger, pc)

                time.sleep(1)
                x, _, _ = pc.get_position()
                delta_x = ultimate_goal[0] - x

            # Adjust along Y axis of checkpoint coordinate
            print("Phase 2")
            _, y_now, _ = pc.get_position()
            delta_y = y_saved - y_now
            while abs(delta_y) > EPSILON and ((not is_close(multiranger.left) and delta_y > 0) or (not is_close(multiranger.right) and delta_y < 0)):
                finish_landed = avoidance_move_y_avoid(delta_y, multiranger, pc)
                time.sleep(1)
                _, y_now, _ = pc.get_position()
                delta_y = y_saved - y_now

            # Exit condition avoidance
            avoid = False
            x_complete = True

            print("End ---------------------")
        # Handle if it an obstacle was not seen before on X axis but is seen now
        elif not avoid and (is_close_front(multiranger.front) and delta_x > 0) or (is_close_front(multiranger.back) and delta_x < 0):  # condition to enter avoidance
            print('Phase 1 -----------------')
            if y <= Y_MIDDLE_MAP:
                print('Avoid direction : LEFT')
                x_avoid_direction = LEFT
            else:
                print('Avoid direction : RIGHT')
                x_avoid_direction = RIGHT
            _, y_saved, _ = pc.get_position()
            avoid = True
        # Handle X movement when no obstacle has been/was detected in X axis
        else:
            if abs(delta_x) > EPSILON:
                finish_landed=avoidance_move_x(delta_x, multiranger, pc)
            else:
                print('Arrived X')
                x_complete = True
    return finish_landed


def go_B_to_A_saved(ultimate_goal, pc, multiranger, back_to_base):
    """
    Global function to link two checkpoints. Handle if an obstacles is detected or not and call the appropriate function
    It is the function called when going from goal to start.

    :param ultimate_goal: Checkpoint coordinate
    :param pc: Position commander info
    :param multiranger: multiranger info
    :param back_to_base: Say if the drone is going back or not
    :return: If the landing has been completed
    """
    # Control Y position
    y_complete = False
    avoid = False
    finish_landed=False
    while not y_complete and not finish_landed:

        _, y, _ = pc.get_position()
        delta_y = ultimate_goal[1] - y
        # Handle the detection of an obstacle in Y direction. First if handle if an obstacle has already been detected
        if avoid:
            while abs(delta_y) > EPSILON:
                _, y, _ = pc.get_position()
                delta_y = ultimate_goal[1] - y
                if (is_close(multiranger.right) and delta_y < 0) or (is_close(multiranger.left) and delta_y > 0):
                    if delta_y < 0:
                        pc.left(STEP_AVOIDANCE)
                        time.sleep(0.1)
                    else:
                        pc.right(STEP_AVOIDANCE)
                        time.sleep(0.1)
                    pc.forward(STEP_AVOIDANCE)
                    time.sleep(0.1)
                else:
                    if is_close(multiranger.back):
                        pc.forward(STEP_AVOIDANCE)

                    pc.forward(STEP_AVOIDANCE)
                    finish_landed = avoidance_move_y_avoid(delta_y, multiranger, pc)
                time.sleep(1)
            avoid = False
            y_complete = True

            # Adjust
            print("Phase 2")
            x_now, _, _ = pc.get_position()
            delta_x = x_saved - x_now
            while abs(delta_x) > EPSILON and ((not is_close_front(multiranger.front) and delta_x>0) or (not is_close(multiranger.back) and delta_x < 0)):
                finish_landed = avoidance_move_x_avoid(delta_x, multiranger, pc)

                time.sleep(1)
                x_now, _, _ = pc.get_position()
                delta_x = x_saved - x_now

            print("End ---------------------")
        # Handle if it an obstacle was not seen before on Y axis but is seen now
        elif not avoid and ((delta_y < 0 and is_close(multiranger.right)) or (delta_y > 0 and is_close(multiranger.left))):  # condition to enter avoidance
            print('Phase 1 -----------------')
            x_saved, _, _ = pc.get_position()
            avoid = True
        # Handle Y movement when no obstacle has been/was detected in Y axis
        else:
            if abs(delta_y) > EPSILON:
                finish_landed = avoidance_move_y_comeback(delta_y, multiranger, pc)
            else:
                print('Arrived Y')
                y_complete = True

    print('Y complete, begin X -------------------------------------------')

    # Control X position
    x_complete = False
    avoid = False
    LEFT = 0
    RIGHT = 1
    Y_MIDDLE_MAP = absolute_to_relative_y(1.7)


    while not x_complete and not finish_landed:

        print("Front:{}".format(multiranger.front))
        x, y, _ = pc.get_position()
        delta_x = ultimate_goal[0] - x
        # Handle the detection of an obstacle in X direction. First if handle if an obstacle has already been detected
        if avoid:
            while abs(delta_x) > EPSILON:
                if (is_close(multiranger.back) and delta_x < 0) or (is_close_front(multiranger.front) and delta_x > 0):
                    if delta_x < 0:
                        pc.forward(STEP_AVOIDANCE)
                        time.sleep(0.1)
                    else:
                        pc.back(STEP_AVOIDANCE)
                        time.sleep(0.1)

                    if x_avoid_direction == RIGHT:
                        pc.right(STEP_AVOIDANCE)
                        time.sleep(0.1)
                    else:
                        pc.left(STEP_AVOIDANCE)
                        time.sleep(0.1)
                else:
                    if x_avoid_direction == RIGHT:
                        if is_close(multiranger.left):
                            print('wall too close on the left')
                            pc.right(STEP_AVOIDANCE)
                            time.sleep(0.1)

                    if x_avoid_direction == LEFT:
                        if is_close(multiranger.right):
                            print('wall too close on the right')
                            pc.left(STEP_AVOIDANCE)
                            time.sleep(0.1)

                    print('Probel delta_x')
                    print(delta_x)
                    finish_landed = avoidance_move_x_avoid(delta_x, multiranger, pc)

                time.sleep(1)
                x, _, _ = pc.get_position()
                delta_x = ultimate_goal[0] - x

            # Adjust
            print("Phase 2")
            _, y_now, _ = pc.get_position()
            delta_y = y_saved - y_now
            while abs(delta_y) > EPSILON and ((not is_close(multiranger.left) and delta_y > 0) or (not is_close(multiranger.right) and delta_y < 0)):
                finish_landed = avoidance_move_y_avoid(delta_y, multiranger, pc)
                time.sleep(1)
                _, y_now, _ = pc.get_position()
                delta_y = y_saved - y_now

            # Exit condition avoidance
            avoid = False
            x_complete = True

            print("End ---------------------")
        # Handle if it an obstacle was not seen before on X axis but is seen now
        elif not avoid and (is_close_front(multiranger.front) and delta_x > 0) or (is_close_front(multiranger.back) and delta_x < 0):  # condition to enter avoidance
            print('Phase 1 -----------------')
            if y <= Y_MIDDLE_MAP:
                print('Avoid direction : LEFT')
                x_avoid_direction = LEFT
            else:
                print('Avoid direction : RIGHT')
                x_avoid_direction = RIGHT
            _, y_saved, _ = pc.get_position()
            avoid = True
        # Handle X movement when no obstacle has been/was detected in X axis
        else:
            if abs(delta_x) > EPSILON:
                finish_landed = avoidance_move_x_comeback(delta_x, multiranger, pc)
            else:
                print('Arrived X')
                x_complete = True
    return finish_landed


def relative_to_absolute(x_r, y_r, x_ref, y_ref):
    """
    Change the coordinate from relative coordinate of the drone to absolute one of the map

    :param x_r: Relative coordinate along X
    :param y_r: Relative coordinate along Y
    :param x_ref: X coordinate of the takeoff pad in absolute coordinate
    :param y_ref: Y coordinate of the takeoff pad in absolute coordinate
    :return: Coordinates in absolute coordinate
    """
    x_abs = x_r + x_ref
    y_abs = - y_r + y_ref
    return x_abs, y_abs


def absolute_to_relative(x_change, y_change):
    """
    Change from absolute coordinate, map, to relative coordinate, drone

    :param x_change: Coordinate in X in absolute coordinate
    :param y_change: Coordinate in X in absolute coordinate
    :return: Coordinate in relative referential
    """
    x_r = x_change - X_START
    y_r = - y_change + Y_START
    return x_r, y_r


def absolute_to_relative_x(x_change):
    """
    Change from absolute coordinate, map, to relative coordinate, drone, of X axis

    :param x_change: Coordinate in X in absolute coordinate
    :return: X coordinate in relative referential
    """
    x_r = x_change - X_START
    return x_r


def absolute_to_relative_y(y_change):
    """
    Change from absolute coordinate, map, to relative coordinate, drone, of Y axis

    :param y_change: Coordinate in Y in absolute coordinate
    :return: Y coordinate in relative referential
    """
    y_r = - y_change + Y_START
    return y_r


def log_pos_callback(timestamp, data, logconf):
    """
    Callback to handle the position quadcopter estimate
    """
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    position_estimate[2] = data['stateEstimate.z']


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with Multiranger(scf) as multiranger:
            with PositionHlCommander(
                    scf,
                    x=0.0, y=0.0, z=0.0,  # initial position
                    default_velocity=0.2,
                    default_height=HEIGHT,
                    controller=PositionHlCommander.CONTROLLER_PID) as pc:
                logconf = LogConfig(name='Position', period_in_ms=10)
                logconf.add_variable('stateEstimate.x', 'float')
                logconf.add_variable('stateEstimate.y', 'float')
                logconf.add_variable('stateEstimate.z', 'float')
                scf.cf.log.add_config(logconf)
                logconf.data_received_cb.add_callback(log_pos_callback)

                # Start estimator callback and reset it
                logconf.start()
                pc._reset_position_estimator()
                pc._reset_position_estimator()
                time.sleep(0.5)
                print('Initialization complete')

                # Compute the checkpoint the drone will cross
                waypoints_x = [4.1]
                waypoints_x[0] = absolute_to_relative_x(waypoints_x[0])
                for i in range(0, 7):
                    waypoints_x.append(waypoints_x[i]+STEP_X_LIST)
                waypoints_y = [0.1]
                waypoints_y[0] = absolute_to_relative_y(waypoints_y[0])
                for i in range(0, 10):
                    waypoints_y.append(waypoints_y[i] - STEP_Y_LIST)

                # Choose take starting direction at take off
                if not is_close_front(multiranger.front):
                    pc.forward(0.4)
                elif not is_close_back(multiranger.back):
                    pc.back(0.4)
                elif not is_close_left(multiranger.left):
                    pc.left(0.4)
                else:
                    print("No free direction")

                # Start the actual program and the search pattern. Encoded in the checkpoint coordinate
                print("One step forward")
                time.sleep(1)
                print(waypoints_x)
                print(waypoints_y)
                finish_landed = False
                for x_loop in range(len(waypoints_x)):
                    for y_loop in range(len(waypoints_y)):
                        if finish_landed:
                            print("Hourra")

                        # Check which checkpoint is the next one
                        elif x_loop % 2:  # odd up
                            print('New waypoint ----------------')
                            goal = [waypoints_x[x_loop], waypoints_y[len(waypoints_y)-y_loop-1], HEIGHT]
                            print(goal)

                            finish_landed = go_A_to_B_saved(goal, pc, multiranger, False)
                            print('-----------------------------')
                        else:  # even down
                            print('New waypoint ----------------')
                            goal = [waypoints_x[x_loop], waypoints_y[y_loop]]
                            print(goal)

                            finish_landed = go_A_to_B_saved(goal, pc, multiranger, False)
                            print('-----------------------------')

                # Take off from landing pad and choose the starting direction
                pc.take_off(0.5)
                time.sleep(0.5)
                if not is_close_back(multiranger.back):
                    pc.back(0.6)
                elif not is_close_front(multiranger.front):
                    pc.forward(0.6)
                elif not is_close_left(multiranger.left):
                    pc.left(0.6)
                else:
                    print("Blocked")
                print('1 step back')

                # Final search zone :
                waypoints_x_back = [1.5]
                waypoints_x_back[0] = absolute_to_relative_x(waypoints_x_back[0])
                for i in range(0, 5):
                    waypoints_x_back.append(waypoints_x_back[i] - STEP_X_LIST)
                waypoints_y_back = [0.5]
                waypoints_y_back[0] = absolute_to_relative_y(waypoints_y_back[0])
                for i in range(0, 7):
                    waypoints_y_back.append(waypoints_y_back[i] - STEP_Y_LIST)
                print(waypoints_x_back)
                print(waypoints_y_back)

                # Algorithm to search for the starting pad
                finish_landed = False
                for x_loop in range(len(waypoints_x_back)):
                    for y_loop in range(len(waypoints_y_back)):
                        if finish_landed:
                            print("Hourra")
                        elif x_loop % 2:  # odd up
                            print('New waypoint ----------------')
                            goal = [waypoints_x_back[x_loop], waypoints_y_back[len(waypoints_y_back) - y_loop - 1], HEIGHT]
                            print(goal)

                            finish_landed = go_B_to_A_saved(goal, pc, multiranger, True)
                            print('-----------------------------')
                        else:  # even down
                            print('New waypoint ----------------')
                            goal = [waypoints_x_back[x_loop], waypoints_y_back[y_loop]]
                            print(goal)
                            
                            finish_landed = go_B_to_A_saved(goal, pc, multiranger, True)
                            print('-----------------------------')

                # Reaching the goal and stopping as the quadcopter is arrived
                pc.land()
                logconf.stop()
                # We land when the MotionCommander goes out of scope
                print('UAV landing complete')

                # Program completed
