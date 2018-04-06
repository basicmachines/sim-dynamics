#!/usr/bin/env python
"""Dynamic simulation of a pendulum.
"""

from pygame.locals import K_z, K_x, K_c, K_b, K_n, K_m

from simulator import Simulator

# Import the model you want to simulate
from models import Pendulum

# Import the controller(s) you want to simulate
from controllers import PIDController, KeyboardInput

TARGET_FPS = 30

# Initialize model

model = Pendulum(position=(16, 12))

# ----------- Setup Keyboard Controller ---------------

# Map the keys to the model inputs
key_actions = {
    K_z: 'TP3', # Positive torque values (counter-clockwise)
    K_x: 'TP2',
    K_c: 'TP1',
    K_m: 'TN3', # Negative torque values (clockwise)
    K_n: 'TN2',
    K_b: 'TN1'
}

kbd_controller = KeyboardInput(model.inputs, key_actions=key_actions)

key_instructions = [
    'z, x, c - apply anti-clockwise torque',
    'b, n, m - apply clockwise torque'
]

# ----------- Setup PID Controller ---------------

# Dictionary used to convert a desired torque
# setting to a set of boolean outputs
torque_settings = {
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

pid_controller = PIDController(
    cv=model.outputs['a'],
    mv=model.inputs,
    kp=75.0,
    ki=8.0,
    kd=300.0,
    set_point=0.0,
    mv_max=7,
    mv_min=-7,
    bool_outputs=torque_settings,
    time_step=1.0 / TARGET_FPS
)

# ------------- Run Simulation -----------------

simulator = Simulator(
    model=model,
    controllers=[kbd_controller, pid_controller],
    key_instructions=key_instructions
)

simulator.run()
