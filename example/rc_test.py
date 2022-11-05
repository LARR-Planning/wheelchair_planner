from simulator.WheelTractionSim import WheelTractionSim
from planner import MPCAckermann as mpc
import time
import pygame
from math import sqrt
import numpy as np

tick = 100
dt = 1 / tick
wheel_sim = WheelTractionSim("settings.yaml")
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
    # keys = pygame.key.get_pressed()
    events = pygame.event.get()

    # if events[pygame.QUIT]:
    #     running = False
    # elif events[pygame.MOUSEBUTTONDOWN]:
    #     pass
    for event in events:
        if event.type == pygame.KEYDOWN:
            keys = pygame.key.get_pressed()
            if keys[pygame.K_SPACE]:
                pass
            elif keys[pygame.K_q] or keys[pygame.K_ESCAPE]:
                running = False
            elif keys[pygame.K_UP]:
                print("increase robot velocity")
                v_r += 0.1
            elif keys[pygame.K_DOWN]:
                print("decrease robot velocity")
                v_r -= 0.1
            elif keys[pygame.K_RIGHT]:
                print("move to right")
                phi -= 0.01
            elif keys[pygame.K_LEFT]:
                print("move to left")
                phi += 0.01
            elif keys[pygame.K_r]:
                print("reset robot position")
                wheel_sim.reset_line()
            elif keys[pygame.K_u]:
                print("Undocking Wheelchair")
                wheel_sim.docking(False)
                v_r = 0
                phi = 0
                yaw_rate_command = 0
            elif keys[pygame.K_d]:
                print("Docking Wheelchair")
                wheel_sim.docking(True)
                v_r = 0
                phi = 0
                yaw_rate_command = 0
            if not wheel_sim.with_chair:
                if keys[pygame.K_k]:
                    print("turn to left")
                    yaw_rate_command += 0.1
                elif keys[pygame.K_l]:
                    print("turn to right")
                    yaw_rate_command -= 0.1

    if wheel_sim.with_chair:
        # x_vel, y_vel, ang_vel = gen_from_rc(v_r, phi, wheel_sim.L)
        # MPC example
        syeon_model.state_update(wheel_sim.lT_r.position.y_val, wheel_sim.lT_r.rotation.yaw,
                                 sqrt(wheel_sim.x_vel ** 2 + wheel_sim.y_vel ** 2), wheel_sim.yaw_rate)
        syeon_model.ackermann_mpc()
        vel_command, steering_angle = syeon_model.for_simulation_command
        x_vel, y_vel, ang_vel = vel_command * np.cos(steering_angle), vel_command * np.sin(
            steering_angle), vel_command * np.sin(steering_angle) / syeon_model.l_wh
    else:
        x_vel = v_r
        y_vel = phi
        ang_vel = yaw_rate_command
    for i in range(3):
        wheel_sim.step(x_vel, y_vel, ang_vel)
    clock.tick(tick)
    cur = time.time()
    if (cur - key_cur) > 0.1:
        key_cur = cur
        key_flag = True
    i += 1
    if (cur - start) > 1:
        start = cur
        print(f"loop time : {i} Hz")
        i = 0
