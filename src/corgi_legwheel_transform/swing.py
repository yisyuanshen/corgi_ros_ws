import numpy as np
import nlopt
from bezier import *
import time


class SwingProfile:
    def __init__(self, L, h, dh, dL1, dL2, dL3, dL4, offset_x=0, offset_y=0, diff_h=0):
        self.L = L
        self.h = h
        self.dh = dh
        self.dL1 = dL1
        self.dL2 = dL2
        self.dL3 = dL3
        self.dL4 = dL4
        self.control_points = []

        self.offset_x = offset_x
        self.offset_y = offset_y
        self.diff_h = diff_h
        
        self.getControlPoint()
        self.bezier = Bezier(self.control_points)

    def getControlPoint(self):
        c0 = np.array([0, 0])
        c1 = c0 - np.array([self.dL1, 0])
        c2 = c1 - np.array([self.dL2, 0]) + np.array([0, self.h])
        c3 = c2
        c4 = c2
        c5 = c4 + np.array([0.5 * self.L + self.dL1 + self.dL2, 0])
        c6 = c5
        c7 = c5 + np.array([0, self.dh])
        c8 = c7 + np.array([0.5 * self.L + self.dL3 + self.dL4, 0])
        c9 = c8
        c10 = c8 - np.array([self.dL4, self.h + self.dh]) + np.array([0, self.diff_h])
        c11 = c10 - np.array([self.dL3, 0])

        self.control_points = [c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11]

    def getFootendPoint(self, t_duty):
        return self.bezier.getBzPoint(t_duty, self.offset_x, self.offset_y)


class SwingLegPlanner:
    def __init__(self, dt, T_sw, T_st):
        self.dt = dt
        self.T_sw = T_sw
        self.T_st = T_st

        self.step_L = 0.1
        self.step_h = 0.05
        self.step_dh = 0.01
        self.v_liftoff = np.array([0, 0])
        self.v_touchdown = np.array([0, 0])

        self.opt = None
        self.opt_lb = np.array([-0.0 for _ in range(2)])
        self.opt_ub = np.array([0.5 for _ in range(2)])
        self.optimizerSetup()
        self.dL1_preset = 0.05
        # self.dL2_preset = -0.05
        self.dL2_preset = -0.0
        self.dL3_preset = 0.05
        # self.dL4_preset = -0.05
        self.dL4_preset = -0.0

    def optimizerSetup(self):
        self.opt = nlopt.opt(nlopt.LN_COBYLA, 2)
        self.opt.set_xtol_abs(1e-5)
        self.opt.set_maxeval(40)
        self.opt.set_upper_bounds(self.opt_ub)
        self.opt.set_lower_bounds(self.opt_lb)

    def solveSwingTrajectory(self, p_lo, p_td, step_h, v_lo, v_td):
        # p_lo, lift-off point in world frame
        # p_td, touch-down point in world frame
        # step_h, step height

        step_L = np.linalg.norm(p_lo - p_td)
        self.step_L = step_L
        self.step_h = step_h
        self.step_dh = 0.01
        self.v_liftoff = v_lo
        self.v_touchdown = v_td

        # optimization setup
        # self.opt.set_min_objective(self.objectiveFunc)
        # x_initial = np.array([0.05, -0.05, 0.05, -0.05])
        # x_opt = self.opt.optimize(x_initial)

        # lift off
        self.opt.set_min_objective(self.objectiveFunc_lo)
        self.opt.add_inequality_constraint(self.constraint_lo)
        x_lo_0 = np.array([self.dL1_preset, self.dL2_preset])
        x_lo_opt = self.opt.optimize(x_lo_0)

        # touch down
        self.optimizerSetup()
        self.opt.set_min_objective(self.objectiveFunc_td)
        self.opt.add_inequality_constraint(self.constraint_td)
        x_td_0 = np.array([self.dL3_preset, self.dL4_preset])
        x_td_opt = self.opt.optimize(x_td_0)

        # print(x_lo_opt[0])
        # print(x_lo_opt[1])
        # print(x_td_opt[0])
        # print(x_td_opt[1])

        return SwingProfile(
            self.step_L,
            self.step_h,
            self.step_dh,
            x_lo_opt[0],
            x_lo_opt[1],
            x_td_opt[0],
            x_td_opt[1],
            p_lo[0],
            p_lo[1],
        )

    def objectiveFunc(self, x, grad):
        sp_ = SwingProfile(
            self.step_L, self.step_h, self.step_dh, x[0], x[1], x[2], x[3]
        )

        d_duty = self.dt / self.T_sw
        duty_0 = 0.0
        duty_1 = d_duty
        duty_2 = 1.0 - d_duty
        duty_3 = 1.0
        p0_ = sp_.getFootendPoint(duty_0)
        p1_ = sp_.getFootendPoint(duty_1)
        p2_ = sp_.getFootendPoint(duty_2)
        p3_ = sp_.getFootendPoint(duty_3)
        v_lo_ = (p1_ - p0_) / self.dt
        v_td_ = (p3_ - p2_) / self.dt

        err_lo_ = np.linalg.norm((self.v_liftoff - v_lo_))
        err_td_ = np.linalg.norm((self.v_touchdown - v_td_))

        return err_lo_ + err_td_

    def objectiveFunc_lo(self, x, grad):
        sp_ = SwingProfile(
            self.step_L,
            self.step_h,
            self.step_dh,
            x[0],
            x[1],
            self.dL3_preset,
            self.dL4_preset,
        )
        d_duty = 0.001 / self.T_sw
        duty_0 = 0.0
        duty_1 = d_duty
        p0_ = sp_.getFootendPoint(duty_0)
        p1_ = sp_.getFootendPoint(duty_1)
        v_lo_ = (p1_ - p0_) / self.dt
        err_lo_ = np.linalg.norm((self.v_liftoff - v_lo_))
        return err_lo_

    def objectiveFunc_td(self, x, grad):
        sp_ = SwingProfile(
            self.step_L,
            self.step_h,
            self.step_dh,
            self.dL1_preset,
            self.dL2_preset,
            x[0],
            x[1],
        )
        d_duty = 0.001 / self.T_sw
        duty_2 = 1.0 - d_duty
        duty_3 = 1.0
        p2_ = sp_.getFootendPoint(duty_2)
        p3_ = sp_.getFootendPoint(duty_3)
        v_td_ = (p3_ - p2_) / self.dt
        err_td_ = np.linalg.norm((self.v_touchdown - v_td_))
        return err_td_

    def constraint_lo(self, x, grad):
        l1, l2 = x
        cons = 0  # <= 0
        cons = -l1 - l2 - (self.step_L / 2 - 0.01)
        return cons

    def constraint_td(self, x, grad):
        l3, l4 = x
        cons = 0  # <= 0
        cons = (self.step_L / 2 + 0.01) - (self.step_L + l3 + l4)
        return cons


if __name__ == "__main__":
    step_h = 0.04
    step_l = 0.15
    p_lo = np.array([0, -0.1])
    p_td = np.array([step_l, -0.1])

    swp = SwingLegPlanner(0.01, 0.6, 1.8)
    v_ = np.array([-step_l / 2.4, 0])
    start = time.time()
    sp = swp.solveSwingTrajectory(p_lo, p_td, step_h, v_, v_)
    end = time.time()
    print("time elapsed: ", end - start)
    print(sp.dL1)
    print(sp.dL2)
    print(sp.dL3)
    print(sp.dL4)
    d = np.linspace(0, 1, 10000)
    curve_points = [sp.getFootendPoint(_) for _ in d]
    x_ = [p[0] for p in curve_points]
    y_ = [p[1] for p in curve_points]
    plt.plot(x_, y_)
    plt.show()
