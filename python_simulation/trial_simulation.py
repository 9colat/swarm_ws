import simpy
import random
import statistics
import pygame
import time
import array as arr


pygame.init()

# all the constants
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
ROBOT_SIZE = (25,25)
numberofObstacles = 10
numberofRobots = 1000


#create a grid (continued in the main loop)
SCREEN = pygame.display.set_mode([SCREEN_WIDTH, SCREEN_HEIGHT])


#our obstacle class
class Obstacle(pygame.sprite.Sprite):
    def __init__(self):
        super(Obstacle, self).__init__()
        self.surf = pygame.Surface(OBSTACLE_SIZE)
        self.surf.fill(RED)
        self.rect = self.surf.get_rect(
            center = (#(SCREEN_WIDTH/2,SCREEN_HEIGHT/2)
                random.randint(0, SCREEN_WIDTH),
                random.randint(0, SCREEN_HEIGHT), #now it spawns randomly on the screen
            )
        )
        self.speed = random.randint(19, 20) #self speed - random, for now
    def update(self):
        self.rect.move_ip(0,0) #moving direction
        #below just keeps the guy on the screen
        if self.rect.left < 0:
            self.rect.left = 0
        if self.rect.right > SCREEN_WIDTH:
            self.rect.right = SCREEN_WIDTH
        if self.rect.top <= 0:
            self.rect.top = 0
        if self.rect.bottom >= SCREEN_HEIGHT:
            self.rect.bottom = SCREEN_HEIGHT


#our robot class
class Robot(pygame.sprite.Sprite):
    def __init__(self):
        super(Robot, self).__init__()
        self.surf = pygame.Surface(ROBOT_SIZE)
        self.surf.fill(BLUE)
        self.rect = self.surf.get_rect(
            center = (#(SCREEN_WIDTH/2,SCREEN_HEIGHT/2)
                random.randint(0, SCREEN_WIDTH),
                random.randint(0, SCREEN_HEIGHT), #now it spawns randomly on the screen
            )
        )
        self.speed = random.randint(19, 20) #self speed - random, for now
    def update(self):
        self.rect.move_ip(0,0) #moving direction
        #below just keeps the guy on the screen
        if self.rect.left < 0:
            self.rect.left = 0
        if self.rect.right > SCREEN_WIDTH:
            self.rect.right = SCREEN_WIDTH
        if self.rect.top <= 0:
            self.rect.top = 0
        if self.rect.bottom >= SCREEN_HEIGHT:
            self.rect.bottom = SCREEN_HEIGHT




ADDOBSTACLE = pygame.USEREVENT + 1
pygame.time.set_timer(ADDOBSTACLE, 100)
obstacles = pygame.sprite.Group()
ADDROBOT = pygame.USEREVENT + 2
pygame.time.set_timer(ADDROBOT, 200)
robots = pygame.sprite.Group()

#for rendering
all_sprites = pygame.sprite.Group()

clock = pygame.time.Clock()


#start the simulation
running = True
while running:


    obstacles.update()
    robots.update()


    for event in pygame.event.get():

        #here are the obstacles spawning
        if (len(obstacles) < numberofObstacles):
            if event.type == ADDOBSTACLE:
                OBSTACLE_SIZE = (random.randint(10,100),random.randint(10, 100))
                new_obstacle = Obstacle()
                obstacles.add(new_obstacle)
                all_sprites.add(new_obstacle)

        if (len(obstacles) == numberofObstacles):
            if event.type == ADDROBOT:
                if (len(robots) < numberofRobots):
                    new_robot = Robot()
                    robots.add(new_robot)
                    all_sprites.add(new_robot)
                    if pygame.sprite.groupcollide(obstacles, robots, False, True) == True:
                        print("now")
                        for entity in robots:
                            print("i am")
                            entity.kill()
                            print("dead")
                    #if pygame.sprite.groupcollide(robots, new_robot, False, True) == True:
                    #    print("misa")
                    #    for entity in robots:
                    #        print("will")
                    #        entity.kill()
                    #        print("dead too")


    # Flip the display
    pygame.display.flip()

    # Fill the background with white
    SCREEN.fill(WHITE)

    for entity in all_sprites:
        SCREEN.blit(entity.surf, entity.rect)

    #for closing the simulation
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    clock.tick(60)


# Done! Time to quit.

pygame.quit()
