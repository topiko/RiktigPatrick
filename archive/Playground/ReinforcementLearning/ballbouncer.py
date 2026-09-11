import numpy as np


class MovingTarget():

    def __init__(self, dt, x0=0, y0=0, vx0=0, vy0=0):
        self.dt = dt
        self.x_ = x0
        self.y_ = y0
        self.vx_ = vx0
        self.vy_ = vy0

    def update(self):
        """Override with your update"""
        return

    def get_pos(self):
        return self.x_, self.y_

    def get_vel(self):
        return self.vx_, self.vy_

    def set_vel(self, vx, vy):
        self.vx_ = vx
        self.vy_ = vy

class BouncingBall(MovingTarget):

    def __init__(self, dt, g=1, fric=0, vinit=1):
        self.g = g
        self.fric=fric
        super().__init__(dt, 0,0,vinit,vinit)


    def out_of_box(self):
        return [self.x_ < 0, self.x_ > 1, self.y_ < 0, self.y_ > 1]

    def update(self):

        location_arr = self.out_of_box()
        ax = -self.fric * self.vx_
        ay = -self.fric * self.vy_ - self.g

        if not any(location_arr):
            self.x_  += self.vx_ * self.dt
            self.y_  += self.vy_ * self.dt
            self.vx_ += ax * self.dt
            self.vy_ += ay * self.dt
        else:
            if location_arr[0]:
                self.vx_ = -self.vx_
                self.x_ = 0
            if location_arr[1]:
                self.vx_ = -self.vx_
                self.x_ = 1
            if location_arr[2]:
                self.vy_ = -self.vy_
                self.y_ = 0
            if location_arr[3]:
                self.vy_ = -self.vy_
                self.y_ = 1




