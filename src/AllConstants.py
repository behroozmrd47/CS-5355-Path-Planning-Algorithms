import math

worldImg = 1
FLOOR_IMG_ADR = 'Slide' + str(worldImg) + '.JPG'
ROBOT_RADIUS = 20

actionDict = {(0, 0): 'stop', (1, 0): 'X+', (1, 1): 'X+Y+', (0, 1): 'Y+', (-1, 1): 'X-Y+', (-1, 0): 'X-',
              (-1, -1): 'X-Y-', (0, -1): 'Y-', (1, -1): 'X+Y-'}

if FLOOR_IMG_ADR == 'Slide1.JPG':
    start_state = ((100, 760), 270)
    goal_state = ((600, 360), 270)
if FLOOR_IMG_ADR == 'Slide2.JPG':
    start_state = ((600, 500), 90)
    goal_state = ((200, 800), 90)
if FLOOR_IMG_ADR == 'Slide3.JPG':
    start_state = ((80, 80), 90)
    goal_state = ((800, 80), 90)
if FLOOR_IMG_ADR == 'Slide4.JPG':  ##Blocked map
    start_state = ((80, 80), 90)
    goal_state = ((800, 432), 90)
# start = ((200, 100), 270)
# # start = ((830, 700), 270)
# # start = ((920, 60), 270)
# # start = ((200, 100), 270)
# # goal = ((600, 360), 270)
# goal = ((440, 400), 90)

timeCons = 0.1  ##In seconds
Ang_Speed = 1  ##Rad per second
Forward_Speed = 10  ##meters per second
Error = 2  ##

Init_Vel = [0.0, 0.0]  ##Initial velocity in speed and omega