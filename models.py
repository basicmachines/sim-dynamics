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

from Box2D import b2Vec2
from vars import Variable
from math import cos, pi

# TODO: Consider a superclass that models inherit from
# class Box2DModel(object):
#     """Dynamic simulation model class using Box2D.
#     """
#     pass


class Pendulum(object):
    """
    Dynamic simulation model of a pendulum using Box2D.

    Attributes:
        position (tuple or b2Vec2): (x, y) co-ordinates of pendulum's
                                    fulcrum.
        arm_length (float): Dimensions of pendulum arm
        arm_width (float): ... etc.
        ball_radius (float):
        start_angle (float): Initial angle of pendulum. Clockwise in
                             radians. start_angle = 0.0 is vertical up.
        joint_friction_torque (float): Determines the amount of friction
                                       in the fulcrum joint.
        min_motor_torque (float): Determines how much torque is applied by
                                  each unit of input torque.
        inputs (list): List of input variables (of type vars.Variable).
        outputs (list): List of output variables (of type vars.Variable).
        torque_settings (dict): Dictionary mapping inputs ('TP1', 'TP2', ...)
                                to units of torque.
        torque_inputs (dict): Dictionary mapping units of torque to a desired
                              set of boolean torque inputs. (This is needed
                              by controllers such as PID).
        torque (float): Applied torque calculated in update_inputs().
        anchor_body (b2Body): Object to represent the pendulum physics.
        body (b2Body): ... etc.
        arm_fixture (b2Body)
        ball_fixture (b2Body)
        joint (b2Body)
    """

    def __init__(self, position=(0.0, 0.0), arm_length=8.0, arm_width=0.4,
                 ball_radius=1.0, start_angle=pi, joint_friction_torque=10.0,
                 min_motor_torque=20.0, density=1.0, torque_inputs=None):

        self.name = 'Pendulum'
        self.position = b2Vec2(position)

        # Pendulum parameters
        self.arm_length = arm_length
        self.arm_width = arm_width
        self.ball_radius = ball_radius
        self.start_angle = start_angle
        self.joint_friction_torque = joint_friction_torque
        self.motor_torque_unit = min_motor_torque
        self.density = density

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

        if torque_inputs is None:
            # Dictionary used to convert a desired torque
            # value to a set of boolean outputs
            self.torque_inputs = {
                0: [],
                1: ['TP1'],
                2: ['TP2'],
                3: ['TP1', 'TP2'],
                4: ['TP3'],
                5: ['TP1', 'TP3'],
                6: ['TP2', 'TP3'],
                7: ['TP1', 'TP2', 'TP3'],
                -1: ['TN1'],
                -2: ['TN2'],
                -3: ['TN1', 'TN2'],
                -4: ['TN3'],
                -5: ['TN1', 'TN3'],
                -6: ['TN2', 'TN3'],
                -7: ['TN1', 'TN2', 'TN3']
            }

        self.torque = 0.0

        # Pendulum variables (outputs)
        self.outputs = {
            'T': Variable('float', name='T', init_value=0.0),
            'a': Variable('float', name='a', init_value=self.start_angle),
            'dadt': Variable('float', name='dadt', init_value=0.0)
        }

    def add_to_box2d(self, world):
        """Create the Box2D objects in world.

        Args:
            world: Box2D world object
        """

        # Create a static body for the pendulum's anchor
        self.anchor_body = world.CreateStaticBody(
            position=self.position
        )

        # Create a dynamic body for the pendulum
        self.body = world.CreateDynamicBody(
            position=self.position,
            angle=0.5*pi - self.start_angle
        )

        self.arm_fixture = self.body.CreatePolygonFixture(
            box=(0.5*self.arm_length, 0.5*self.arm_width, (0.5*self.arm_length, 0), 0.0),
            density=self.density,
            friction=0.3
        )

        self.ball_fixture = self.body.CreateCircleFixture(
            pos=(self.arm_length, 0),
            radius=self.ball_radius,
            density=self.density,
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

        self.torque = torque_setting*self.motor_torque_unit

        # Note minus sign because Box2D world angles are anti-clockwise
        self.body.ApplyTorque(-self.torque, wake=True)

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
        # by the statement above
        self.outputs['a'].value = 0.5*pi - self.body.angle

        # Alternative way to implement above without affecting
        # self.body.angle:
        # self.outputs['a'].value = ((1.5*pi - self.body.angle) % (2*pi)) - pi

        # Speed of rotation
        self.outputs['dadt'].value = -self.body.angularVelocity

        # Torque applied to pendulum
        self.outputs['T'].value = self.torque

    def reset(self):
        """Resets the model variables to their initial
        states."""

        self.body.angularVelocity = 0.0
        self.body.linearVelocity = b2Vec2(0.0, 0.0)
        self.body.position = self.position
        self.body.angle = 0.5*pi - self.start_angle
        self.update_outputs()


class CartPole(object):
    """
    Dynamic simulation model of a cart-pole system using Box2D.

    Attributes:
        position (tuple or b2Vec2): (x, y) co-ordinates of cart.
        pole_length (float): Dimensions of cart-pole system
        pole_width (float): ...
        cart_width (float):
        cart_height (float):
        start_angle (float): Initial angle of pole. Clockwise in
                             radians. start_angle = 0.0 is vertical up.
        joint_friction_torque (float): Determines the amount of friction
                                       in the fulcrum joint.
        sliding_friction (float): Determines the amount of friction
                                  between the cart and ground.
        min_motor_force (float): Determines how much force is applied by
                                 each unit of input force.
        cart_position (float): x position of cart
        start_position (float): Initial x position of top of pole
        inputs (list): List of input variables (of type vars.Variable).
        outputs (list): List of output variables (of type vars.Variable).
        force_settings (dict): Dictionary of values mapping inputs
                               ('FR1', 'FR2', etc.) to units of force.
        force (float): Applied force calculated in update_inputs().
        cart_body (b2Body): Object to represent the pendulum physics.
        pole_body (b2Body): ... etc.
        cart_fixture (b2Body)
        pole_fixture (b2Body)
        cart_wheel1 (b2Body)
        cart_wheel2 (b2Body)
        joint (b2Body)
        """

    def __init__(self, position=(0, 0), pole_length=12.0, pole_width=0.4, cart_width=4.0,
                 cart_height=2.0, start_angle=0.5*pi, joint_friction_torque=10.0,
                 sliding_friction=20.0, min_motor_force=10.0, density=1.0):

        self.name = 'Cart-Pole'
        self.start_position = float(position[0])

        # Pendulum parameters
        self.pole_length = pole_length
        self.pole_width = pole_width
        self.cart_width = cart_width
        self.cart_height = cart_height
        self.start_angle = start_angle
        self.joint_friction_torque = joint_friction_torque
        self.sliding_friction = sliding_friction
        # TODO: Sliding friction is not doing anything yet

        self.motor_force_unit = min_motor_force
        self.density = density
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

        # Model variables (outputs)
        self.outputs = {
            'F': Variable('float', name='F', init_value=0.0),
            'x': Variable('float', name='x', init_value=self.start_position),
            'dxdt': Variable('float', name='dxdt', init_value=0.0)
        }

    def add_to_box2d(self, world):
        """Create the Box2D objects in world.

        Args:
            world: Box2D world object
        """

        # Create a dynamic body for the cart
        self.cart_body = world.CreateDynamicBody(
            position=self.cart_position
        )

        # Create a dynamic body for the pole
        self.pole_body = world.CreateDynamicBody(
            position=self.cart_position,
            angle=self.start_angle
        )

        self.cart_fixture = self.cart_body.CreatePolygonFixture(
            box=(0.5*self.cart_width, 0.1*self.cart_height),
            density=self.density,
            friction=0.3
        )

        self.pole_fixture = self.pole_body.CreatePolygonFixture(
            box=(0.5*self.pole_length, 0.5*self.pole_width, (0.5*self.pole_length, 0), 0.0),
            density=self.density,
            friction=0.3
        )

        self.cart_wheel1 = self.cart_body.CreateCircleFixture(
            pos=(-0.5*self.cart_width, 0),
            radius=0.5*self.cart_height,
            density=self.density,
            friction=0.3
        )

        self.cart_wheel2 = self.cart_body.CreateCircleFixture(
            pos=(0.5*self.cart_width, 0),
            radius=0.5*self.cart_height,
            density=self.density,
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

        # Check each input and adjust force applied
        force_setting = sum(
            [self.force_settings[t] for t in self.inputs if self.inputs[t].value]
        )

        self.force = force_setting*self.motor_force_unit
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

        # Horizontal velocity of top of base of pole
        dxdt = self.pole_body.linearVelocity.x

        # Horizontal velocity of top of pole
        self.outputs['dxdt'].value = dadt*self.pole_length/(2*pi) + dxdt

        # Force applied to cart
        # TODO: Why is this minus sign there?
        self.outputs['F'].value = -self.force

    def reset(self):
        """Resets the model variables to their initial
        states."""

        self.cart_body.position = self.cart_position
        self.cart_body.angularVelocity = 0.0
        self.cart_body.linearVelocity = b2Vec2(0.0, 0.0)
        self.pole_body.angle = self.start_angle
        self.pole_body.position = self.cart_position
        self.pole_body.angularVelocity = 0.0
        self.pole_body.linearVelocity = b2Vec2(0.0, 0.0)
        self.update_outputs()


class Segway(object):
    """
    Dynamic simulation model of a segway using Box2D.

    Attributes:
        start_position (float): x co-ordinates of top of pole.
        pole_length (float): Dimensions of segway
        pole_width (float): ...
        wheel_radius (float):
        start_angle (float): Initial angle of segwsy. Clockwise in
                             radians. start_angle = 0.0 is vertical up.
        max_motor_torque (float): Maximum torque that the motor can apply
                                  to the wheel to change speed.
        motor_speed_unit (float): Determines rotation speed of wheel for
                                  each unit of input speed.
        wheel_start_position (tuple or b2Vec2): Initial (x, y) position of
                                                wheel centre
        inputs (list): List of input variables (of type vars.Variable).
        outputs (list): List of output variables (of type vars.Variable).
        speed_settings (dict): Dictionary mapping inputs ('SP1', 'SP2', ...)
                            to units of speed.
        speed_inputs (dict): Dictionary mapping units of speed to a desired
                             set of boolean speed inputs. (This is needed
                             by controllers such as PID).
        speed (float): Speed of wheels (calculated in update_inputs).
        wheel_body (b2Body): Object to represent the pendulum physics.
        pole_body (b2Body): ... etc.
        wheel_fixture (b2Body)
        pole_fixture (b2Body)
        joint (b2Body)
        """

    def __init__(self, position=(0, 0), pole_length=6.0, pole_width=0.4, wheel_radius=2.0,
                 start_angle=0.5*pi, max_motor_torque=320.0, motor_speed_unit=0.5*pi,
                 density=1.0, speed_inputs=None):

        self.name = 'Segway'
        self.start_position = float(position[0])

        # Pendulum parameters
        self.pole_length = pole_length
        self.pole_width = pole_width
        self.wheel_radius = wheel_radius
        self.start_angle = start_angle
        self.max_motor_torque = max_motor_torque
        self.motor_speed_unit = motor_speed_unit
        self.density = density
        self.wheel_start_position = b2Vec2(
            self.start_position - pole_length*cos(start_angle),
            position[1]
        )

        # Segway speed inputs (manipulated variables)
        self.inputs = {
            'SP1': Variable('bool', name='SP1'),
            'SP2': Variable('bool', name='SP2'),
            'SP3': Variable('bool', name='SP3'),
            'SN1': Variable('bool', name='SN1'),
            'SN2': Variable('bool', name='SN2'),
            'SN3': Variable('bool', name='SN3')
        }

        self.speed_settings = {
            'SP1': 1,
            'SP2': 2,
            'SP3': 4,
            'SN1': -1,
            'SN2': -2,
            'SN3': -4
        }

        if speed_inputs is None:
            self.speed_inputs = {
                0: [],
                1: ['SP1'],
                2: ['SP2'],
                3: ['SP1', 'SP2'],
                4: ['SP3'],
                5: ['SP1', 'SP3'],
                6: ['SP2', 'SP3'],
                7: ['SP1', 'SP2', 'SP3'],
                -1: ['SN1'],
                -2: ['SN2'],
                -3: ['SN1', 'SN2'],
                -4: ['SN3'],
                -5: ['SN1', 'SN3'],
                -6: ['SN2', 'SN3'],
                -7: ['SN1', 'SN2', 'SN3']
            }

        self.speed = 0.0

        # Model variables (outputs)
        self.outputs = {
            'R': Variable('float', name='R', init_value=0.0),
            'a': Variable('float', name='a', init_value=self.start_angle),
            'dadt': Variable('float', name='dadt', init_value=0.0),
            'x': Variable('float', name='x', init_value=self.start_position),
            'dxdt': Variable('float', name='dxdt', init_value=0.0)
        }

    def add_to_box2d(self, world):
        """Create the Box2D objects in world.

        Args:
            world: Box2D world object
        """

        # Create a dynamic body for the cart
        self.wheel_body = world.CreateDynamicBody(
            position=self.wheel_start_position
        )

        # Create a dynamic body for the pole
        self.pole_body = world.CreateDynamicBody(
            position=self.wheel_start_position,
            angle=self.start_angle
        )

        self.wheel_fixture = self.wheel_body.CreateCircleFixture(
            pos=(0.0, 0.0),
            radius=0.5*self.wheel_radius,
            density=self.density,
            friction=4.0 # Needed to stop the wheels slipping
        )

        self.pole_fixture = self.pole_body.CreatePolygonFixture(
            box=(0.5*self.pole_length, 0.5*self.pole_width, (0.5*self.pole_length, 0), 0.0),
            density=self.density,
            friction=0.3
        )

        # This fixture is purely to make the
        # wheel rotation visible
        self.wheel_hub = self.wheel_body.CreatePolygonFixture(
            box=(0.25*self.wheel_radius, 0.25*self.wheel_radius, (0, 0), 0.0),
            density=0.0,
            friction=0.0
        )

        # Mass on pole fixture
        self.pole_mass_fixture = self.pole_body.CreateCircleFixture(
            pos=(0.5*self.pole_length, 0.0),
            radius=0.4*self.wheel_radius,
            density=self.density,
            friction=0.3
        )

        # Join the two bodies together
        # Use a motor to simulate friction
        self.joint = world.CreateRevoluteJoint(
            bodyA=self.wheel_body,
            bodyB=self.pole_body,
            anchor=self.wheel_body.worldCenter,
            maxMotorTorque=self.max_motor_torque,
            motorSpeed=0.0,
            enableMotor=True)

    def update_inputs(self):
        """Applies the values of the input variables to
        the bodies in the physical model.  Run this method
        before each time step of the physics simulation."""

        # Check inputs and update speed of motor
        speed_setting = sum(
            [self.speed_settings[t] for t in self.inputs if self.inputs[t].value]
        )

        self.speed = speed_setting*self.motor_speed_unit

        # Note minus sign because Box2D world angles are anti-clockwise
        self.joint.motorSpeed = self.speed

    def update_outputs(self):
        """Updates the values of the output variables to
        reflect the current state of the model.  Run this
        method after each time step of the physics
        simulation."""

        # The angle of the pole is measured from the vertical
        # position (clockwise = positive). The range of the
        # angle is limited to -pi to pi by the statement above
        self.outputs['a'].value = 0.5*pi - self.pole_body.angle

        # Speed of rotation
        self.outputs['dadt'].value = -self.pole_body.angularVelocity

        # For this model, the control variable is the horizontal
        # position of the top of the pole. x=0 is the center position.

        pole_angle = self.pole_body.angle
        self.outputs['x'].value = self.pole_length*cos(pole_angle) + self.pole_body.position.x

        dadt = -self.pole_body.angularVelocity

        # Horizontal speed of top of base of pole
        dxdt = self.pole_body.linearVelocity.x

        # Horizontal speed of top of pole
        self.outputs['dxdt'].value = dadt*self.pole_length/(2*pi) + dxdt

        # Rotational speed of wheels
        self.outputs['R'].value = self.speed

    def reset(self):
        """Resets the model variables to their initial
        states."""

        self.wheel_body.position = self.wheel_start_position
        self.wheel_body.angularVelocity = 0.0
        self.wheel_body.linearVelocity = b2Vec2(0.0, 0.0)
        self.pole_body.angle = self.start_angle
        self.pole_body.position = self.wheel_start_position
        self.pole_body.angularVelocity = 0.0
        self.pole_body.linearVelocity = b2Vec2(0.0, 0.0)
        self.update_outputs()
