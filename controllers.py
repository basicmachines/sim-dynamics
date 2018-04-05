#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Dynamic Model Simulator.

A dynamic physical model simulator using pyBox2d within a
self-contained pygame environment for the purpose of
experimenting with dynamic models and dynamic model
controllers.

The systems can be influenced manually through keyboard
commands or controlled by a simple controller (PID).

Files contained in the module:

 1. simulator.py - main program file
 2. vars.py - Variable class
 3. data_output.py - file output operations
 4. models.py - example models
 5. controllers.py - contains a keyboard and a PID controller

The pyBox2d shape classes are extended with some drawing
code so that they render in pygame.  Adapted from the examples
in the pybox2d repository:
https://github.com/pybox2d/pybox2d/tree/master/examples/simple
"""

import pygame
from collections import deque
from future.utils import iteritems

class KeyboardInput(object):

    def __init__(self, model_inputs, key_actions, name="Keyboard"):

        self.model_inputs = model_inputs
        self.key_actions = key_actions
        self.name = name

    def update_actions(self):

        # Get status of all keys
        keys = pygame.key.get_pressed()

        # Update model inputs if there is a change
        # Note: keys only work on binary (True/False) model inputs
        for k, a in iteritems(self.key_actions):
            if keys[k]:
                self.model_inputs[a].value = True
            else:
                self.model_inputs[a].value = False


class PIDController(object):
    """Controller for use with physics simulation
    models. Uses the PID (proportional, integral,
    derivative) control algorithm.
    """

    def __init__(self, cv, mv, kp, ki, kd, set_point,
                 mv_max=None, mv_min=None, name="PID",
                 bool_outputs=None,
                 time_step=1.0, integral_length=30):

        self.cv = cv
        self.mv = mv
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.set_point = set_point

        self.mv_max = mv_max
        self.mv_min = mv_min
        self.name = name
        self.bool_outputs = bool_outputs

        if bool_outputs is not None:
            if not isinstance(mv, dict):
                raise TypeError("If using boolean outputs, mv "
                                "argument must be a dict of outputs.")

        self.time_step = time_step

        self.integral_length = integral_length
        self.errors = deque()
        self.error_sum = 0.0
        self.de_dt = 0.0

    def update_actions(self):

        error = self.cv.value - self.set_point

        if len(self.errors) > 0:
            self.de_dt = (error - self.errors[0])/self.time_step

        self.errors.appendleft(error)
        self.error_sum += error

        if len(self.errors) > self.integral_length:
            self.error_sum -= self.errors.pop()

        output_value = (self.kp * error +
                        self.ki * self.error_sum +
                        self.kd * self.de_dt)

        if output_value > self.mv_max:
            output_value = self.mv_max
        elif output_value < self.mv_min:
            output_value = self.mv_min

        if self.bool_outputs:
            output_value = int(output_value)
            bool_outputs = self.bool_outputs[output_value]

            for x in self.mv:
                if x in bool_outputs:
                    self.mv[x].value = True
                else:
                    self.mv[x].value = False
        else:
            self.mv.value = output_value
