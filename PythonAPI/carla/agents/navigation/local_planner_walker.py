# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" This module contains a local planner to perform low-level location following based on PID controllers. """

from collections import deque

import carla
from agents.tools.misc import get_speed

class WalkerLocalPlanner(object):
    """
    WalkerLocalPlanner implements the basic behavior of following a
    trajectory of locations that is generated on-the-fly.

    The low-level motion of the walker is computed by using two PID controllers,
    one is used for the lateral control and the other for the longitudinal control (cruise speed).

    When multiple paths are available (intersections) this local planner makes a random choice,
    unless a given global plan has already been specified.
    """

    def __init__(self, walker, opt_dict={}):
        """
        :param walker: actor to apply to local planner logic onto
        :param opt_dict: dictionary of arguments with different parameters:
            dt: time between simulation steps
            target_speed: desired cruise speed in Km/h
            lateral_control_dict: values of the lateral PID controller
            longitudinal_control_dict: values of the longitudinal PID controller
        """
        self._walker = walker
        self._world = self._walker.get_world()

        self._walker_controller = None
        self.target_location = None

        self._location_queue = deque(maxlen=100)

        # Base parameters
        self._dt = 0.05
        self._target_speed = 1.0  # Km/h
        self._args_lateral_dict = {'K_P': 1.95, 'K_I': 0.05, 'K_D': 0.2, 'dt': self._dt}
        self._args_longitudinal_dict = {'K_P': 1.0, 'K_I': 0.05, 'K_D': 0, 'dt': self._dt}
        self._base_min_distance = 1.0
        self._distance_ratio = 0.5

        # Overload parameters
        if opt_dict:
            if 'dt' in opt_dict:
                self._dt = opt_dict['dt']
            if 'target_speed' in opt_dict:
                self._target_speed = opt_dict['target_speed']
            if 'lateral_control_dict' in opt_dict:
                self._args_lateral_dict = opt_dict['lateral_control_dict']
            if 'longitudinal_control_dict' in opt_dict:
                self._args_longitudinal_dict = opt_dict['longitudinal_control_dict']
            if 'base_min_distance' in opt_dict:
                self._base_min_distance = opt_dict['base_min_distance']
            if 'distance_ratio' in opt_dict:
                self._distance_ratio = opt_dict['distance_ratio']

        self.target_location = self._walker.get_location()
        self._location_queue.append(self.target_location)

    def set_speed(self, speed):
        """
        Changes the target speed

        :param speed: new target speed in Km/h
        :return:
        """
        self._target_speed = speed

    def set_plan(self, current_plan, clean_queue=True):
        """
        Adds a new plan to the local planner. A plan must be a list of carla.Transform.
        The 'clean_queue` parameter erases the previous plan if True, otherwise, it adds it to the old one

        :param current_plan: list of carla.Transform
        :param clean_queue: bool
        :return:
        """
        if clean_queue:
            self._location_queue.clear()

        # Remake the locations queue if the new plan has a higher length than the queue
        new_plan_length = len(current_plan) + len(self._location_queue)
        if new_plan_length > self._location_queue.maxlen:
            new_location_queue = deque(maxlen=new_plan_length)
            for wp in self._location_queue:
                new_location_queue.append(wp)
            self._location_queue = new_location_queue

        for elem in current_plan:
            self._location_queue.append(elem)

    def run_step(self):
        """
        Execute one step of local planning which involves running the longitudinal and lateral PID controllers to
        follow the locations trajectory.

        :return: control to be applied
        """
        # Purge the queue of obsolete locations
        walker_location = self._walker.get_location()
        walker_speed = get_speed(self._walker) / 3.6
        self._min_distance = self._base_min_distance + self._distance_ratio * walker_speed

        num_location_removed = 0
        for target_location in self._location_queue:

            if len(self._location_queue) - num_location_removed == 1:
                min_distance = 1  # Don't remove the last locations until very close by
            else:
                min_distance = self._min_distance

            if walker_location.distance(target_location) < min_distance:
                num_location_removed += 1
            else:
                break

        if num_location_removed > 0:
            for _ in range(num_location_removed):
                self._location_queue.popleft()

        # Get the target location and move. Stop if no target location
        control = carla.WalkerControl()
        if len(self._location_queue) == 0:
            control.speed = 0.0
        else:
            self.target_location = self._location_queue[0]
            direction = self.target_location - self._walker.get_location()

            control.speed = self._target_speed
            control.direction = direction.make_unit_vector()

        return control

    def done(self):
        """
        Returns whether or not the planner has finished

        :return: boolean
        """
        return len(self._location_queue) == 0
