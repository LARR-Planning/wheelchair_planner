from simulator.WheelTractionSim import WheelTractionSim
import time
from simulator.utils import gen_from_rc
import pygame

tick = 100
dt = 1 / tick
wheel_sim = WheelTractionSim("../simulator/settings.yaml")
print("hello wheel chair")
running = True
clock = pygame.time.Clock()

start = time.time()
v_r = 0
phi = 0
yaw_rate_command = 0
i = 0
keys = pygame.key.get_pressed()
while running:
    events = pygame.event.get()
    for event in events:
        if event.type == pygame.KEYDOWN:
            keys = pygame.key.get_pressed()
            if keys[pygame.K_SPACE]:
                pass
            elif keys[pygame.K_q] or keys[pygame.K_ESCAPE]:
                running = False
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
            if not wheel_sim.with_chair:
                if keys[pygame.K_k]:
                    print("turn to left")
                    yaw_rate_command += 0.1
                elif keys[pygame.K_l]:
                    print("turn to right")
                    yaw_rate_command -= 0.1
    if wheel_sim.with_chair:
        x_vel, y_vel, ang_vel = gen_from_rc(v_r, phi, wheel_sim.L)
    else:
        # sim w/o wheelchair
        x_vel = v_r
        y_vel = phi
        ang_vel = yaw_rate_command
    wheel_sim.step(x_vel, y_vel, ang_vel)
    clock.tick(tick)
    cur = time.time()
    i += 1
    if (cur - start) > 1:
        start = cur
        print(f"loop time : {i} Hz")
        i = 0
