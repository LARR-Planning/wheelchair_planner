import sys
from os.path import dirname, abspath
sys.path.append(dirname(dirname(abspath(__file__))))
from simulator.WheelTractionSim import WheelTractionSim
from planner import MPCAckermann as mpc
import time
import pygame
from math import sqrt, pi
import numpy as np

tick = 100
dt = 1 / tick
wheel_sim = WheelTractionSim(dirname(dirname(abspath(__file__)))+"/simulator/settings.yaml")
print("hello wheel chair")
running = True
clock = pygame.time.Clock()

start = key_cur = time.time()
v_r = 0
phi = 0
yaw_rate_command = 0
i = 0
keys = pygame.key.get_pressed()
syeon_model = mpc.ackermann_mpc()
while running:
    events = pygame.event.get()
    for event in events:
        if event.type == pygame.KEYDOWN:
            keys = pygame.key.get_pressed()
            if keys[pygame.K_q] or keys[pygame.K_ESCAPE]:
                running = False
            elif keys[pygame.K_r]:
                print("reset robot position")
                # theta = pi * random.random()
                ######
                # Enter the next guide line's angle HERE!
                theta = wheel_sim.oT_l.rotation.yaw - pi*2/3
                # wheel_sim.oT_l.rotation.yaw : Current guide line's angle
                # -pi/2 : turn right w.r.t. current guide line
                ######
                wheel_sim.reset_line(theta)
            elif keys[pygame.K_l]:
                print("reset robot position")
                # theta = pi * random.random()
                ######
                # Enter the next guide line's angle HERE!
                theta = wheel_sim.oT_l.rotation.yaw + pi*2/3
                # wheel_sim.oT_l.rotation.yaw : Current guide line's angle
                # -pi/2 : turn right w.r.t. current guide line
                ######
                wheel_sim.reset_line(theta)
            elif keys[pygame.K_u]:
                print("Undocking Wheelchair")
                wheel_sim.docking(False)
            elif keys[pygame.K_d]:
                print("Docking Wheelchair")
                wheel_sim.docking(True)
    # Solve MPC optimization
    first = time.time()
    syeon_model.state_update(wheel_sim.lT_r.position.y_val, wheel_sim.lT_r.rotation.yaw,
                             sqrt(wheel_sim.x_vel ** 2 + wheel_sim.y_vel ** 2), wheel_sim.yaw_rate)
    syeon_model.ackermann_mpc()
    vel_command, steering_angle = syeon_model.control_output[0]
    x_vel, y_vel, ang_vel = vel_command * np.cos(steering_angle), vel_command * np.sin(
        steering_angle), vel_command * np.sin(steering_angle) / syeon_model.l_wh
    end = time.time()
    for i in range(int((end-first)//dt)+1):
        wheel_sim.step(x_vel, y_vel, ang_vel)
        # print(f"y_error : {wheel_sim.lT_r.position.y_val}, theta_error: {wheel_sim.lT_r.rotation.yaw} ")
        i += 1
    clock.tick(tick)
    cur = time.time()
    if (cur - key_cur) > 0.1:
        key_cur = cur
    i += 1
    if (cur - start) > 1:
        start = cur
        print(f"loop time : {i} Hz, MPC computation time : {end - first : .2f}")
        i = 0
