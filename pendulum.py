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
    K_m: 'TP3', # Positive torque values (counter-clockwise)
    K_n: 'TP2',
    K_b: 'TP1',
    K_z: 'TN3', # Negative torque values (clockwise)
    K_x: 'TN2',
    K_c: 'TN1'
}

kbd_controller = KeyboardInput(model.inputs, key_actions=key_actions)

key_instructions = [
    'z, x, c - apply anti-clockwise torque',
    'b, n, m - apply clockwise torque'
]

# ----------- Setup PID Controller ---------------

pid_controller = PIDController(
    cv=model.outputs['a'],
    mv=model.inputs,
    kp=75.0,
    ki=8.0,
    kd=300.0,
    set_point=0.0,
    mv_max=7,
    mv_min=-7,
    bool_outputs=model.torque_settings,
    time_step=1.0 / TARGET_FPS
)

# ------------- Run Simulation -----------------

simulator = Simulator(
    model=model,
    controllers=[kbd_controller, pid_controller],
    key_instructions=key_instructions
)

simulator.run()
