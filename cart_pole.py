

from pygame.locals import K_z, K_x, K_c, K_b, K_n, K_m

from simulator import Simulator

# Import the model(s) you want to simulate
from models import CartPole

# Import the controller(s) you want to simulate
from controllers import PIDController, KeyboardInput

TARGET_FPS = 30

# Initialize model

model = CartPole(position=(16, 2))

# ----------- Setup Keyboard Controller ---------------

# Map the keys to the model inputs
key_actions = {
    K_z: 'FL3',  # Force cart left
    K_x: 'FL2',
    K_c: 'FL1',
    K_m: 'FR3',  # Force cart right
    K_n: 'FR2',
    K_b: 'FR1'
}

kbd_controller = KeyboardInput(model.inputs, key_actions=key_actions)

key_instructions = [
    'z, x, c - apply force left',
    'b, n, m - apply force right'
]

# ----------- Setup PID Controller ---------------

# Dictionary used to convert a desired torque
# setting to a set of boolean outputs
force_settings = {
    0: [],
    1: ['FR1'],
    2: ['FR2'],
    3: ['FR1', 'FR2'],
    4: ['FR3'],
    5: ['FR1', 'FR3'],
    6: ['FR2', 'FR3'],
    7: ['FR1', 'FR2', 'FR3'],
    -1: ['FL1'],
    -2: ['FL2'],
    -3: ['FL1', 'FL2'],
    -4: ['FL3'],
    -5: ['FL1', 'FL3'],
    -6: ['FL2', 'FL3'],
    -7: ['FL1', 'FL2', 'FL3']
}

pid_controller = PIDController(
    cv=model.outputs['a'],
    mv=model.inputs,
    kp=-30.0,
    ki=0.0,
    kd=0.0,
    set_point=0.0,
    mv_max=7,
    mv_min=-7,
    bool_outputs=force_settings,
    time_step=1.0 / TARGET_FPS
)

# ------------- Run Simulation -----------------

simulator = Simulator(
    model=model,
    controllers=[kbd_controller, pid_controller],
    key_instructions=key_instructions
)

simulator.run()
