"""This script will test that your pygame environment is working.
There is a known issue with pygame running on Max OS X with
Python version 3.

See her for more details:
https://github.com/pygame/pygame/issues/203
"""

import pygame
from pygame.locals import QUIT, KEYDOWN, K_ESCAPE

import logging
logging.basicConfig(level=logging.DEBUG)
logging.info("Starting test...")

screen = pygame.display.set_mode((640, 480), 0, 32)
pygame.display.set_caption('Test')

pygame.font.init()
font14 = pygame.font.SysFont("monospace", 14, bold=True, italic=False)
font24 = pygame.font.SysFont("monospace", 24, bold=True, italic=False)
color1 = (255, 255, 255, 255)
background_color = (0, 0, 0, 0)
title_text = font14.render("Press some keys to test Pygame is working.", True, color1)

clock = pygame.time.Clock()
running = True
key = "_"

while running:

    # Check the event queue
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False

        if event.type == KEYDOWN:

            if event.key == K_ESCAPE:
                running = False

            if 32 <= event.key < 128:
                key = chr(event.key)
                logging.info("%s pressed", key)

    output = font24.render(key, True, color1)

    # Clear screen
    screen.fill(background_color)

    screen.blit(title_text, (120, 240))

    screen.blit(output, (120, 320))

    pygame.display.flip()
    clock.tick(30)