import pygame
from pygame.locals import QUIT, KEYDOWN, K_ESCAPE, K_SPACE, K_0, K_1, K_2

import logging
logging.basicConfig(level=logging.DEBUG)
logging.info("Starting test...")

screen = pygame.display.set_mode((640, 480), 0, 32)

pygame.display.set_caption('Test')

running = True

while running:

    # Check the event queue
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False

        if event.type == KEYDOWN:

            if event.key == K_ESCAPE:
                running = False

            logging.info("%s pressed", chr(event.key))