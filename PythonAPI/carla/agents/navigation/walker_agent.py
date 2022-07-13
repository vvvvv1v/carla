# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module implements an agent that automatically follow a given plan.
(Very simplified version of the AIWalkerController)
"""

from agents.navigation.local_planner_walker import WalkerLocalPlanner


class WalkerAgent(object):
    """
    WalkerAgent implements an agent that automatically follow a given plan
    """

    def __init__(self, walker, target_speed=1, opt_dict={}):
        """
        Initialization the agent parameters.

            :param walker: actor to apply to agent logic onto
            :param target_speed: speed (in m/s) at which the walker will move
            :param opt_dict: dictionary in case some of its parameters want to be changed.
                This also applies to parameters related to the WalkerLocalPlanner.
        """
        self._walker = walker
        self._world = self._walker.get_world()

        self._target_speed = target_speed
        self._local_planner = WalkerLocalPlanner(self._walker, opt_dict=opt_dict)

    def set_target_speed(self, speed):
        """
        Changes the target speed of the agent

            :param speed (float): target speed in Km/h
        """
        self._target_speed = speed
        self._local_planner.set_speed(speed)

    def set_plan(self, plan, clean_queue=True):
        """
        Adds a specific plan to the agent.

            :param plan: list of carla.Location representing the route to be followed
            :param clean_queue: resets the current agent's plan
        """
        self._local_planner.set_plan(plan, clean_queue=clean_queue)

    def run_step(self):
        """Execute one step of navigation."""
        return self._local_planner.run_step()

    def done(self):
        """Check whether the agent has reached its destination."""
        return self._local_planner.done()
