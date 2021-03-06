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


class Variable(object):
    """Used to define variables for use as inputs to or outputs
    from a physics simulation model.

    Arguments:
    ntype (str): Either 'float', 'discrete', or 'bool'.  Floats are
                 Python float values.  'bool' means Boolean (True or
                 False).  'discrete' means the variable may only have
                 a finite list of values (e.g. [-1.0, 0.0, 1.0]).
                 Integers are not currently supported.  Use floats
                 or discrete instead.
    name (str): Give the variable a name as a string
    values (list): Provide a list of possible values (mandatory for
                 for discrete variables)
    min, max (float): Provide a lower and upper bound for the range of
                 possible values (use for float variables only)
    init_value (float or int): Give the variable an initial value (default is None).
    """

    # The following types are used to represent binary (bool),
    # discrete (int), and continuous (float) variables.
    ntypes = {
        'float': float,
        'discrete': (float, int, str, bool),
        'bool': bool
    }

    def __init__(self, ntype, name=None, values=None, lb=None, ub=None, init_value=None):

        if ntype not in self.ntypes:
            raise ValueError("Input type {} not supported".format(ntype))

        self.ntype = ntype
        self.name = name

        if ntype == int:
            if self.values is None:
                raise ValueError("Must provide a list of values for discrete input.")
            self.values = values
        elif ntype == float:
            self.lb = lb
            self.ub = ub

        if init_value:
            if not isinstance(init_value, self.ntypes[ntype]):
                raise ValueError("Initial value is not of the correct type.")

        self.value = init_value
