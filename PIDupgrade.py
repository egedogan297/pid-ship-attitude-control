import numpy as np
import matplotlib.pyplot as plt
import turtle
import math
import time

TIME_STEP = 0.05
SETPOINT = 0
SIM_TIME = 2000
INITIAL_ANGLE = math.radians(50)
GRAVITY = 3.71
MASS = 200000
LENGTH = 400
MOI = (1/12) * MASS * LENGTH**2
ARM = LENGTH / 2
GRAVITY_ARM = 20
MAX_THRUST = 270000
BASE_THRUST = 135000
KP = 300000
KI = 300
KD = 4000000

def graph(times, angles):
    angles_deg = []
    for a in angles:
        angles_deg.append(math.degrees(a))
    plt.plot(times, angles_deg, label='Ship Tilt Angle')
    plt.axhline(y=0, color='r', linestyle='--', label='Target')
    plt.xlabel('Time')
    plt.ylabel('Angle (degrees)')
    plt.title('OPERATION: BALANCE')
    plt.legend()
    plt.show()

class Ship(object):
    def __init__(self):
        self.body = turtle.Turtle()
        self.body.hideturtle()
        self.body.color('black')
        self.body.width(6)
        self.dots = [turtle.Turtle(), turtle.Turtle()]
        for d in self.dots:
            d.hideturtle()
            d.penup()
        self.theta = INITIAL_ANGLE
        self.dtheta = 0
        self.ddtheta = 0
    def set_ddtheta(self, thrust_left, thrust_right):
        torque = (thrust_right - thrust_left) * ARM
        gravity_torque = MASS * GRAVITY * GRAVITY_ARM * math.sin(self.theta)
        self.ddtheta = (torque + gravity_torque) / MOI
    def set_dtheta(self):
        self.dtheta = self.dtheta + self.ddtheta * TIME_STEP
    def set_theta(self):
        self.theta = self.theta + self.dtheta * TIME_STEP
    def get_theta(self):
        return self.theta

class PID(object):
    def __init__(self, KP, KI, KD, target):
        self.kp = KP
        self.ki = KI
        self.kd = KD
        self.setpoint = target
        self.error = 0
        self.integral_error = 0
        self.error_last = 0

    def compute(self, current):
        self.error = self.setpoint - current
        self.integral_error = self.integral_error + self.error * TIME_STEP
        derivative_error = (self.error - self.error_last) / TIME_STEP
        self.error_last = self.error
        return self.kp*self.error + self.ki*self.integral_error + self.kd*derivative_error

class simulation(object):
    def __init__(self):
        self.ship = Ship()
        self.pid = PID(KP, KI, KD, SETPOINT)
        self.screen = turtle.Screen()
        self.screen.setup(1280, 900)
        self.screen.tracer(0)
        self.horizon = turtle.Turtle()
        self.horizon.hideturtle()
        self.horizon.penup()
        self.horizon.color('red')
        self.horizon.goto(-300, 0)
        self.horizon.pendown()
        self.horizon.goto(300, 0)
        self.sim = True
        self.timer = 0
        self.angles = np.array([])
        self.times = np.array([])

    def cycle(self):
        while self.sim:
            correction = self.pid.compute(self.ship.get_theta())
            thrust_right = BASE_THRUST + correction / 2
            thrust_left = BASE_THRUST - correction / 2
            if thrust_right > MAX_THRUST:
                thrust_right = MAX_THRUST
            if thrust_right < 0:
                thrust_right = 0
            if thrust_left > MAX_THRUST:
                thrust_left = MAX_THRUST
            if thrust_left < 0:
                thrust_left = 0

            self.ship.set_ddtheta(thrust_left, thrust_right)
            self.ship.set_dtheta()
            self.ship.set_theta()
            theta = self.ship.get_theta()
            x1, y1 = -250 * math.cos(theta), -250 * math.sin(theta)
            x2, y2 = 250 * math.cos(theta), 250 * math.sin(theta)

            self.ship.body.clear()
            self.ship.body.penup()
            self.ship.body.goto(x1, y1)
            self.ship.body.pendown()
            self.ship.body.goto(x2, y2)
            self.ship.dots[0].clear()
            self.ship.dots[0].goto(x2, y2)
            self.ship.dots[0].dot(20, 'yellow')
            self.ship.dots[1].clear()
            self.ship.dots[1].goto(x1, y1)
            self.ship.dots[1].dot(20, 'yellow')
            self.screen.update()
            time.sleep(TIME_STEP)
            self.timer = self.timer + 1
            self.angles = np.append(self.angles, theta)
            self.times = np.append(self.times, self.timer)

            if self.timer > SIM_TIME:
                print("SIM ENDED")
                self.sim = False

        graph(self.times, self.angles)

def main():
    sim = simulation()
    sim.cycle()

main()