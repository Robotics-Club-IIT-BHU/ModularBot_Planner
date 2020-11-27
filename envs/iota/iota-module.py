import pygame
from pygame.locals import QUIT, KEYDOWN, K_ESCAPE
import Box2D
from Box2D.b2 import staticBody, dynamicBody, polygonShape,circleShape, world
import numpy as np

PPM = 20.0
TARGET_FPS = 60
TIME_STEP = 1.0 / TARGET_FPS
SCREEN_WIDTH , SCREEN_HEIGHT = 640, 480
FLAG = True
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT),0 ,32)
pygame.display.set_caption('Simple pygame example')
clock = pygame.time.Clock()

world = world(gravity=(0,0), doSleep=True )
ground_body = world.CreateStaticBody(
    position = (0,0),
    shapes = polygonShape(box=(50,1)),
)
bodies1 = [world.CreateDynamicBody(position=( 3 + 0.1*i , 9)) for i in range(5)]
circles = [body.CreateCircleFixture(radius=0.5, density=1, friction=0.3) for body in bodies1]
bodies2 = [world.CreateDynamicBody(position=(5*i + 3 , 5)) for i in range(5)]
boxes = [body.CreatePolygonFixture(box=(2,1), density=1, friction=0.3) for body in bodies2]

colors = {
    staticBody: (255, 255, 255, 255),
    dynamicBody: (127, 127, 127, 255),
}
def init_arrow(color, scale=None):
    global PPM
    if scale is None:
        if PPM == 20:
            scale = 1
        else:
            scale = PPM/20.0
    x , y = 8*scale, 8*scale
    arrow = pygame.Surface((x,y))
    arrow.fill((0,0,0))
    pygame.draw.line(arrow,color,(0,0),(x//2,y//2))
    pygame.draw.line(arrow,color,(0,y),(x//2,y//2))
    arrow.set_colorkey((0,0,0))
    return arrow
global_arrow = init_arrow((0,0,255))

def rotate_arrow(arrow,pos,angle):
    nar = pygame.transform.rotate(arrow, -np.degrees(angle))
    nrect = nar.get_rect(center=pos)
    return nar, nrect

def my_draw_polygon(polygon, body, fixture):
    vertices = [(body.transform*v)*PPM for v in polygon.vertices]
    vertices = [(v[0], SCREEN_HEIGHT - v[1]) for v in vertices]
    pygame.draw.polygon(screen, colors[body.type], vertices)
polygonShape.draw = my_draw_polygon

def my_draw_circle(circle, body, fixture):
    global global_arrow,screen
    position = body.transform * circle.pos * PPM
    position = (position[0], SCREEN_HEIGHT - position[1])
    nar, nrect = rotate_arrow(global_arrow.copy(), position, body.transform.angle)
    pygame.draw.circle(screen, colors[body.type], [int(x) for x in position], int(circle.radius*PPM))
    screen.blit(nar, (nrect.x, nrect.y))
circleShape.draw = my_draw_circle

running = True
while running:
    for event in pygame.event.get():
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            running = False
    screen.fill((0,0,0,0))
    for body in world.bodies:
        for fixture in body.fixtures:
            fixture.shape.draw(body, fixture)
    world.Step(TIME_STEP, 10, 10)
    pygame.display.flip()
    clock.tick(TARGET_FPS)

pygame.quit()
