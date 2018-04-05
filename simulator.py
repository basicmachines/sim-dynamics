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
from pygame.locals import QUIT, KEYDOWN, K_ESCAPE, K_SPACE, K_0, K_1, K_2, \
                          K_3, K_4, K_5, K_6, K_7, K_8, K_9, K_r

# Box2D is the main library. Box2D.b2 maps to Box2D.Box2D
import Box2D
from Box2D.b2 import staticBody, dynamicBody, polygonShape, circleShape

import datetime
import logging

from data_output import ModelStateRecorder

# Set logging level
#  DEBUG: Reports many details about variables and events
#   INFO: Reports events such as key-presses
#   WARN: Only report errors and warnings
#  ERROR: Only report errors
# For logging to a file use add filename='logfile.txt'
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s %(levelname)s %(message)s'
)

# ------- Box2D settings and constants ------

# Box2D deals with meters, but we want to display pixels,
# so define a conversion factor:
PPM = 20.0  # pixels per meter
TARGET_FPS = 30
TIME_STEP = 1.0 / TARGET_FPS
SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480
# (640, 480) pixels is equivalent to (32, 24) Box2D units
VELOCITY_ITERS, POSITION_ITERS = 8, 3


class Simulator(object):

    def __init__(self, model, controllers=None, output_file_location="outputs",
                 key_instructions=None):

        self.model = model
        self.controllers = controllers
        self.output_file_location = output_file_location
        self.key_instructions = key_instructions

        logging.info("Starting simulator")

        # -------------- pygame setup ---------------

        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
        pygame.display.set_caption('Physical Model Simulator')
        self.clock = pygame.time.Clock()

        self.object_colors = {
            staticBody: (64, 64, 64, 255),
            dynamicBody: (192, 192, 192, 128),
        }

        # ------- pybox2d world setup -------

        # The following functions extend the Box2D shape classes
        # with a method that draws them in Pygame

        def my_draw_polygon(polygon, body, fixture):
            vertices = [(body.transform * v) * PPM for v in polygon.vertices]
            vertices = [(v[0], SCREEN_HEIGHT - v[1]) for v in vertices]
            pygame.draw.polygon(self.screen, self.object_colors[body.type], vertices)

        polygonShape.draw = my_draw_polygon

        def my_draw_circle(circle, body, fixture):
            position = body.transform * circle.pos * PPM
            position = (position[0], SCREEN_HEIGHT - position[1])
            pygame.draw.circle(self.screen, self.object_colors[body.type], [int(
                x) for x in position], int(circle.radius * PPM))
            # Note: Python 3.x will enforce that pygame get the integers it requests,
            #       and it will not convert from float.

        circleShape.draw = my_draw_circle

        # Create the world
        self.world = Box2D.b2.world(gravity=(0, -10), doSleep=True)
        logging.info("Box2D world initialized")

        # Keep track of number of time steps
        self.time_step_t = 0

        # Create a static body to represent the ground
        self.ground_body = self.world.CreateStaticBody(
             position=(0, 0),
             shapes=polygonShape(box=(50, 1))
        )

        # ----------- model setup ------------

        self.model.add_to_box2d(self.world)
        logging.info("%s model added", self.model.name)

        # Data output file
        self.model_state_recorder = ModelStateRecorder(model, output_file_location=output_file_location)

        # Prepare objects for screen display

        # Choose fonts
        pygame.font.init()
        self.fonts = {
            '14_bold': pygame.font.SysFont("bitstreamverasans", 14, bold=True, italic=False),
            '12': pygame.font.SysFont("bitstreamverasans", 12, bold=False, italic=False),
            '12_mono': pygame.font.SysFont("monospace", 12, bold=False, italic=False)
        }

        # Choose colors
        self.text_color1 = (128, 128, 128, 255)
        self.text_color2 = (255, 255, 0, 255)
        self.background_color = (0, 0, 0, 0)

        # Title
        self.fonts['14_bold'].set_underline(1)
        self.title_text = self.fonts['14_bold'].render(self.model.name, True, self.text_color1)
        self.fonts['14_bold'].set_underline(0)

        # Other text
        self.text_messages = [
            'Keys:',
            'SPACE - pause/resume',
            '{} - select controller (1=keyboard)'.format(range(1, len(controllers) + 1)),
            'r - start/stop recording state',
        ] + self.key_instructions + ['ESCAPE - quit']

        # Create a list of text fields to display
        self.text_fields = []

        # Create a dictionary of variable fields to display
        self.number_fields = {}

        self.line_spacing = self.fonts['12'].size("abc")[1] + 2
        self.number_field_size = self.fonts['12_mono'].size("-0000.00")

        for i, text in enumerate(self.text_messages):

            y = (1 + len(self.text_fields))*self.line_spacing

            self.text_fields.append({
                'surface': self.fonts['12'].render(text, True, self.text_color1),
                'pos': (4, y)
            })

        for p in (['t', 'input', 'recording'] + self.model.inputs.keys() + self.model.outputs.keys()):

            y = (1 + len(self.number_fields))*self.line_spacing

            self.text_fields.append({
                'surface': self.fonts['12'].render(p, True, self.text_color1),
                'pos': (SCREEN_WIDTH - self.number_field_size[0] - self.fonts['12'].size(p)[0] - 12, y)
            })

            self.number_fields[p] = {
                'pos': (SCREEN_WIDTH - self.number_field_size[0] - 4, y)
            }

        self.controller = 0

    def run(self):

        running = True
        paused = True
        recording = False

        logging.info("Simulator ready...")

        while running:

            # Check the event queue
            for event in pygame.event.get():
                if event.type == QUIT:
                    logging.info("User closed the window")
                    running = False

                if event.type == KEYDOWN:

                    if event.key == K_ESCAPE:
                        logging.info("User pressed ESCAPE")
                        running = False

                    if event.key == K_SPACE:
                        paused = not paused
                        if paused:
                            logging.info("Simulation paused")
                        else:
                            logging.info("Simulation restarted")

                    # Change the controller if a number key pressed
                    if K_1 <= event.key < (K_1 + len(self.controllers)):
                        self.controller = int(event.key - K_1)
                        logging.info("Control switched to %s", self.controllers[self.controller].name)

                    if event.key == K_r:
                        recording = not recording
                        if recording:
                            self.model_state_recorder.start()
                            logging.info("Recording started")
                        else:
                            self.model_state_recorder.stop()
                            logging.info("Recording stopped")

            # Call the current controller to set
            # the input values
            self.controllers[self.controller].update_actions()

            if not paused:
                # Update the Box2D model inputs
                self.model.update_inputs()

                # Make Box2D simulate the physics of our world for one step.
                self.world.Step(TIME_STEP, VELOCITY_ITERS, POSITION_ITERS)
                self.time_step_t += 1

                # Update the state of the model based on the Box2D model
                self.model.update_outputs()

                # Save current model state to the output file
                if recording:
                    self.model_state_recorder.save_state()

            # Clear screen
            self.screen.fill(self.background_color)

            # Display title text
            self.screen.blit(
                self.title_text,
                ((SCREEN_WIDTH - self.title_text.get_width()) // 2, 4)
            )

            # Display other text messages
            for t in self.text_fields:
                self.screen.blit(t['surface'], t['pos'])

            # Display model input and output values
            for (p, d) in (self.model.inputs.items() + self.model.outputs.items()):
                if d.type == 'float':
                    f = "{:8.2f}"
                else:
                    f = "{}"
                surface = self.fonts['12_mono'].render(f.format(d.value), False,
                                                       self.text_color2)
                self.screen.blit(surface, self.number_fields[p]['pos'])

            # Display timestep
            surface = self.fonts['12_mono'].render("{:8.2f}".format(self.time_step_t*TIME_STEP),
                                                   False, self.text_color2)
            self.screen.blit(surface, self.number_fields['t']['pos'])

            # Display control input source
            surface = self.fonts['12_mono'].render("{:8s}".format(
                          self.controllers[self.controller].name), False, self.text_color2)
            self.screen.blit(surface, self.number_fields['input']['pos'])

            # Display recording state
            surface = self.fonts['12_mono'].render("{}".format(recording),
                                                   False, self.text_color2)
            self.screen.blit(surface, self.number_fields['recording']['pos'])

            # Draw the world
            for body in self.world.bodies:
                for fixture in body.fixtures:
                    fixture.shape.draw(body, fixture)

            # Flip the screen and try to keep at the target FPS
            pygame.display.flip()
            self.clock.tick(TARGET_FPS)

        if recording:
            self.model_state_recorder.stop()

        pygame.quit()
        logging.info('Execution terminating...')
