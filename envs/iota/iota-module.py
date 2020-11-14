import pygame
from pygame.locals import QUIT, KEYDOWN, K_ESCAPE
import Box2D
from Box2D.b2 import staticBody, dynamicBody, polygonShape, world

PPM = 20.0
TARGET_FPS = 60
TIME_STEP = 1.0 / TARGET_FPS
SCREEN_WIDTH , SCREEN_HEIGHT = 640, 480

screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT),0 ,32)
pygame.display.set_caption('Simple pygame example')
clock = pygame.time.Clock()

world = world(gravity=(0, -10), doSleep=True )
ground_body = world.CreateStaticBody(
    position = (0,1),
    shapes = polygonShape(box=(50,5)),
)

dynamic_body = world.CreateDynamicBody(
    position = (10,15),
    angle=15,
)
box = dynamic_body.CreatePolygonFixture(box=(2,1), density = 1, friction=0.3)

colors = {
    staticBody:(255, 255, 255, 255),
    dynamicBody:(127, 127, 127, 255),
}
running = True
while running:
    for event in pygame.event.get():
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            running = False
    screen.fill((0,0,0,0))
    for body in (ground_body, dynamic_body):
        for fixture in body.fixtures:
            shape = fixture.shape
            vertices = [(body.transform*v)*PPM for v in shape.vertices]
            vertices = [(v[0], SCREEN_HEIGHT - v[1]) for v in vertices]
            pygame.draw.polygon(screen, colors[body.type], vertices )
    world.Step(TIME_STEP, 10, 10)
    pygame.display.flip()
    clock.tick(TARGET_FPS)

pygme.quit()
