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

import Box2D
from Box2D import b2Vec2
from math import cos, pi
from vars import Variable

# TODO: Consider a superclass that models inherit from
# class Box2DModel(object):
#     """Dynamic simulation model class using Box2D.
#     """
#     pass


class Pendulum(object):
    """
    Dynamic simulation model of a pendulum using Box2D.

    Arguments:
        position (Box2D.b2Vec2): x, y co-ordinates of pendulum's
                                 fulcrum.
        arm_length (float)
        arm_width (float)
        ball_radius (float)
        start_angle (float): Initial angle of pendulum (radians)
        joint_friction_torque (float)
        min_motor_torque (float)
    """

    def __init__(self, position=(0.0, 0.0), arm_length=8.0, arm_width=0.4,
                 ball_radius=1.0, start_angle=pi,
                 joint_friction_torque=10.0, min_motor_torque=20.0):

        self.name = 'Pendulum'
        self.fulcrum_position = b2Vec2(position)

        # Pendulum parameters
        self.arm_length = arm_length
        self.arm_width = arm_width
        self.ball_radius = ball_radius
        self.start_angle = start_angle
        self.joint_friction_torque = joint_friction_torque
        self.min_motor_torque = min_motor_torque

        # Pendulum torque inputs (manipulated variables)
        self.inputs = {
            'TP1': Variable('bool', name='TP1'),
            'TP2': Variable('bool', name='TP2'),
            'TP3': Variable('bool', name='TP3'),
            'TN1': Variable('bool', name='TN1'),
            'TN2': Variable('bool', name='TN2'),
            'TN3': Variable('bool', name='TN3')
        }

        self.torque_settings = {
            'TP1': 1,
            'TP2': 2,
            'TP3': 4,
            'TN1': -1,
            'TN2': -2,
            'TN3': -4
        }

        self.torque = 0.0

        # Pendulum variables (outputs)
        self.outputs = {
            'T': Variable('float', name='T', init_value=0.0),
            'a': Variable('float', name='a', init_value=self.start_angle),
            'dadt': Variable('float', name='dadt', init_value=0.0)
        }

    def add_to_box2d(self, world):

        # Create a static body for the pendulum's anchor
        self.anchor_body = world.CreateStaticBody(
            position=self.fulcrum_position
        )

        # Create a dynamic body for the pendulum
        self.body = world.CreateDynamicBody(
            position=self.fulcrum_position,
            angle=0.5*pi - self.start_angle
        )

        self.arm_fixture = self.body.CreatePolygonFixture(
            box=(0.5*self.arm_length, 0.5*self.arm_width, (0.5*self.arm_length, 0), 0.0),
            density=1,
            friction=0.3
        )

        self.ball_fixture = self.body.CreateCircleFixture(
            pos=(self.arm_length, 0),
            radius=self.ball_radius,
            density=1,
            friction=0.3
        )

        # Join the two bodies together
        # Use a motor to simulate friction
        self.joint = world.CreateRevoluteJoint(
            bodyA=self.anchor_body,
            bodyB=self.body,
            anchor=self.anchor_body.worldCenter,
            maxMotorTorque=self.joint_friction_torque,
            motorSpeed=0.0,
            enableMotor=True)

    def update_inputs(self):
        """Applies the values of the input variables to
        the bodies in the physical model.  Run this method
        before each time step of the physics simulation."""

        # Check each input and adjust torque applied
        torque_setting = sum(
            [self.torque_settings[t] for t in self.inputs if self.inputs[t].value]
        )

        self.torque = torque_setting*self.min_motor_torque
        self.body.ApplyTorque(self.torque, wake=True)

    def update_outputs(self):
        """Updates the values of the output variables to
        reflect the current state of the model.  Run this
        method after each time step of the physics
        simulation."""

        # This is necessary to prevent the angle from
        # getting too large or small

        self.body.angle = ((self.body.angle + 0.5*pi) % (2*pi)) - 0.5*pi

        # For this model, the angle of the pendulum is measured
        # from the vertical position (clockwise = positive)
        # The range of the angle is limited to -pi to pi
        self.outputs['a'].value = 0.5*pi - self.body.angle

        # Alternative way to implement above without affecting
        # self.body.angle:
        # self.outputs['a'].value = ((1.5*pi - self.body.angle) % (2*pi)) - pi

        # Speed of rotation
        self.outputs['dadt'].value = -self.body.angularVelocity

        # Torque applied to pendulum
        self.outputs['T'].value = -self.torque

    def reset(self):
        """Resets the model variables to their initial
        state."""

        self.body.angularVelocity = 0.0
        self.body.linearVelocity = b2Vec2(0.0, 0.0)
        self.body.position = self.fulcrum_position
        self.body.angle = 0.5*pi - self.start_angle
        self.update_outputs()


class CartPole(object):
    """
    Dynamic simulation model of a cart-pole system using Box2D.

    Arguments:
        position (Box2D.b2Vec2): x, y co-ordinates of pendulum's
                                 fulcrum.
        pole_length (float)
        pole_width (float)
        cart_width (float)
        cart_height (float)
        start_angle (float)
        joint_friction_torque (float)
        sliding_friction (float)
        min_motor_force (float)
    """

    def __init__(self, position=(0, 0), pole_length=12.0, pole_width=0.4, cart_width=4.0,
                 cart_height=2.0, start_angle=0.5*pi, joint_friction_torque=10.0,
                 sliding_friction=20.0, min_motor_force=10.0):

        self.name = 'Cart-Pole'
        self.start_position = position[0]

        # Pendulum parameters
        self.pole_length = pole_length
        self.pole_width = pole_width
        self.cart_width = cart_width
        self.cart_height = cart_height
        self.start_angle = start_angle
        self.joint_friction_torque = joint_friction_torque
        self.sliding_friction = sliding_friction
        self.min_motor_force = min_motor_force
        self.cart_position = b2Vec2(position[0] - pole_length*cos(start_angle), position[1])

        # Cart force inputs (manipulated variables)
        self.inputs = {
            'FR1': Variable('bool', name='FR1'),
            'FR2': Variable('bool', name='FR2'),
            'FR3': Variable('bool', name='FR3'),
            'FL1': Variable('bool', name='FL1'),
            'FL2': Variable('bool', name='FL2'),
            'FL3': Variable('bool', name='FL3')
        }

        self.force_settings = {
            'FR1': 1,
            'FR2': 2,
            'FR3': 4,
            'FL1': -1,
            'FL2': -2,
            'FL3': -4
        }

        self.force = 0.0

        self.start_position = float(position[0])

        # Model variables (outputs)
        self.outputs = {
            'F': Variable('float', name='F', init_value=0.0),
            'x': Variable('float', name='x', init_value=self.start_position),
            'dxdt': Variable('float', name='dxdt', init_value=0.0)
        }

    def add_to_box2d(self, world):

        # Create a dynamic body for the cart
        self.cart_body = world.CreateDynamicBody(
            position=self.cart_position
        )

        # Create a dynamic body for the pole
        self.pole_body = world.CreateDynamicBody(
            position=self.cart_position,
            angle=self.start_angle
        )

        self.pole_fixture = self.pole_body.CreatePolygonFixture(
            box=(0.5*self.pole_length, 0.5*self.pole_width, (0.5*self.pole_length, 0), 0.0),
            density=1,
            friction=0.3
        )

        self.cart_fixture = self.cart_body.CreatePolygonFixture(
            box=(0.5*self.cart_width, 0.1*self.cart_height),
            density=1,
            friction=0.3
        )

        self.cart_fixture = self.cart_body.CreateCircleFixture(
            pos=(-0.5*self.cart_width, 0),
            radius=0.5*self.cart_height,
            density=1,
            friction=0.3
        )

        self.cart_fixture = self.cart_body.CreateCircleFixture(
            pos=(0.5*self.cart_width, 0),
            radius=0.5*self.cart_height,
            density=1,
            friction=0.3
        )

        # Join the two bodies together
        # Use a motor to simulate friction
        self.joint = world.CreateRevoluteJoint(
            bodyA=self.cart_body,
            bodyB=self.pole_body,
            anchor=self.cart_body.worldCenter,
            maxMotorTorque=self.joint_friction_torque,
            motorSpeed=0.0,
            enableMotor=True)

    def update_inputs(self):
        """Applies the values of the input variables to
        the bodies in the physical model.  Run this method
        before each time step of the physics simulation."""

        # Check each input and adjust torque applied
        force_setting = sum(
            [self.force_settings[t] for t in self.inputs if self.inputs[t].value]
        )

        self.force = force_setting*self.min_motor_force
        self.pole_body.ApplyForce(force=(self.force, 0.0), point=(0.0, 0.0), wake=True)

    def update_outputs(self):
        """Updates the values of the output variables to
        reflect the current state of the model.  Run this
        method after each time step of the physics
        simulation."""

        # This is necessary to prevent the angle
        # from getting too large or small
        self.pole_body.angle %= 2*pi

        # For this model, the control variable is the horizontal
        # position of the top of the pole. x=0 is the center position.

        pole_angle = self.pole_body.angle
        self.outputs['x'].value = self.pole_length*cos(pole_angle) + self.pole_body.position.x

        dadt = -self.pole_body.angularVelocity

        # Horizontal speed of top of base of pole
        dxdt = self.pole_body.linearVelocity.x

        # Horizontal speed of top of pole
        self.outputs['dxdt'].value = dadt*self.pole_length/(2*pi) + dxdt

        # Force applied to cart
        self.outputs['F'].value = -self.force

    def reset(self):
        """Resets the model variables to their initial
        state."""

        self.cart_body.position = self.cart_position
        self.cart_body.angularVelocity = 0.0
        self.cart_body.linearVelocity = b2Vec2(0.0, 0.0)
        self.pole_body.angle = self.start_angle
        self.pole_body.position = self.cart_position
        self.pole_body.angularVelocity = 0.0
        self.pole_body.linearVelocity = b2Vec2(0.0, 0.0)
        self.update_outputs()
