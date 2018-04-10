# sim-dynamics

A dynamic physical model simulator using [Box2D](https://github.com/pybox2d) within a
self-contained [pygame](https://www.pygame.org/) environment for the purpose of
experimenting with dynamic models and dynamic model
controllers.

Three example models are included: a simple pendulum, a
cart-pole system and a segway. 

<img src="/images/screenshot_pendulum.png" width=400 alt="Screenshot">

<img src="/images/screenshot_cartpole.png" width=200 alt="Screenshot">
<img src="/images/screenshot_segway.png" width=200 alt="Screenshot">

The systems can be influenced manually through keyboard
commands or controlled by a simple controller (PID).

Files contained in the module:

1. simulator.py - main program file
2. vars.py - Variable class
3. data_output.py - file output operations
4. models.py - example models
5. controllers.py - contains a keyboard and a PID controller

Example simulations:

1. pendulum.py - simple pendulum
2. cart-pole.py - cart-and-pole system
3. segway.py - segway

The pyBox2d shape classes are extended with some drawing
code so that they render in pygame.  Adapted from the examples 
in the pybox2d repository:
https://github.com/pybox2d/pybox2d/tree/master/examples/simple
