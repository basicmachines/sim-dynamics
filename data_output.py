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

import datetime
import logging
import os


class ModelStateRecorder(object):
    """
    Object for performing output file operations for recording
    model states each time step.

    Attributes:
        model (e.g. Pendulum)
        output_file_location (string)
        name (string)
        output_filename (string)
        output_file (file)
        recording (bool)
        previous_state (string)
    """

    def __init__(self, model, output_file_location="outputs", name=None):
        """Opens a text file in preparation for writing the model
        state and provides methods to save the model state data
        each time step.  Finally, close the file when stop() is
        called.

        Args:
            model: dyanmic model object - e.g. of Pendulum class
            output_file_location: string describing file location
            name: specify a string to use as the beginning of the
                  filename. If not provided, model.name will be used.
        """

        self.model = model
        self.output_file_location = output_file_location

        if name is None:
            self.name = model.name

        ts = datetime.datetime.now()
        self.output_filename = "{}_{}.txt".format(self.name,
                                                  ts.strftime('%y%m%d-%H%M%S'))

        self.output_file = None
        self.recording = False
        self.previous_state = None

    def open_file(self):
        """Opens text file for writing model state data to.
        If the file already exists then data is appended.

        Returns:
            output_file (file)

        Raises:
            IOError: if output file could not be opened.
        """

        path = (self.output_file_location + "/" +
                self.output_filename)

        new_file = False if os.path.exists(path) else True

        try:
            self.output_file = open(path, 'a')
        except IOError:
            raise IOError("Could not write to file %s" %
                          self.output_filename.__repr__())

        if new_file:

            labels = ['Timestamp']
            for k in self.model.inputs.keys():
                labels.append(str(k))

            for k in self.model.outputs.keys():
                labels.append(str(k))

            for k in self.model.outputs.keys():
                labels.append("{}_p1".format(k))

            logging.info("Output file %s opened", self.output_filename.__repr__())
            self.output_file.write(",".join(labels) + '\n')

    def start(self):
        """Opens the output file and starts saving the model
        state.
        """

        if self.output_file is None or self.output_file.closed:
            self.open_file()

        self.recording = True

    def stop(self):
        """Closes the output file and stops saving the model
        state.
        """

        if self.recording:
            self.output_file.close()

        self.recording = False

    def save_state(self):
        """Saves the previous and current state of the model
        to the output file specified when the method
        start_recording() was called. To begin
        recording, simply call start_recording()."""

        if self.output_file.closed:
            raise IOError("Output file is not open for writing.")

        # Start saving state after first time step
        if self.previous_state:

            d = list()
            for i in self.model.outputs.values():
                d.append("{}".format(i.value))
            string_of_outputs = ",".join(d)

            line = ','.join((self.previous_state, string_of_outputs)) + '\n'
            self.output_file.write(line)

        ts = datetime.datetime.now()
        d = [ts.strftime('%Y-%m-%d %H:%M:%S.%f')]

        for i in self.model.inputs.values():
            d.append("{}".format(i.value))

        for i in self.model.outputs.values():
            d.append("{}".format(i.value))

        # TODO: Modify so that it is always comparable with current outputs.
        # Currently angles do not match when 'wrap-around' occurs
        self.previous_state = ",".join(d)
