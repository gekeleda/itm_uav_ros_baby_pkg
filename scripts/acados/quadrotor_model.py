#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2021-03-21 22:23:04
LastEditors: Wei Luo
LastEditTime: 2021-03-30 00:29:12
Note: Quadrotor Model
'''
from os import stat
import numpy as np
import casadi as ca
from acados_template import AcadosModel


class QuadRotorModel(object):
    def __init__(self, ):
        g_ = 9.8066
        # control inputs
        roll_ref_ = ca.SX.sym('roll_ref_')
        pitch_ref_ = ca.SX.sym('pitch_ref_')
        thrust_ref_ = ca.SX.sym('thrust_ref_')
        controls_ = ca.vcat([roll_ref_, pitch_ref_,
                             thrust_ref_])
        # model constants after SI
        # roll_gain = 0.85
        # roll_tau = 0.2
        # pitch_gain = 0.86
        # pitch_tau = 0.3

        roll_gain = 2.477
        roll_tau = 0.477
        pitch_gain = 2.477
        pitch_tau = 0.477
        # model states
        x_ = ca.SX.sym('x_')
        y_ = ca.SX.sym('y_')
        z_ = ca.SX.sym('z_')
        vx_ = ca.SX.sym('vx_')
        vy_ = ca.SX.sym('vy_')
        vz_ = ca.SX.sym('vz_')
        roll_ = ca.SX.sym('roll_')
        pitch_ = ca.SX.sym('pitch_')
        yaw_ = ca.SX.sym('yaw_')
        states_ = ca.vcat([x_, y_, z_, vx_, vy_, vz_, roll_,
                           pitch_, yaw_])

        dragacc1_, dragacc2_ = 0.0, 0.0 # ignore drag currently
        rhs = []
        rhs.append(states_[3])
        rhs.append(states_[4])
        rhs.append(states_[5])
        rhs.append((ca.cos(roll_) * ca.cos(yaw_) * ca.sin(pitch_) +
                    ca.sin(roll_) * ca.sin(yaw_)) * thrust_ref_ - dragacc1_)
        rhs.append((ca.cos(roll_) * ca.sin(pitch_) * ca.sin(yaw_) -
                    ca.cos(yaw_) * ca.sin(roll_)) * thrust_ref_ - dragacc2_)
        rhs.append(-g_ + ca.cos(pitch_) * ca.cos(roll_) * thrust_ref_)
        rhs.append((roll_gain * roll_ref_ - roll_) / roll_tau)
        rhs.append((pitch_gain * pitch_ref_ - pitch_) / pitch_tau)
        rhs.append(0.0)
        self.f = ca.Function('f', [states_, controls_], [ca.vcat(rhs)])

        # acados model
        x_dot = ca.SX.sym('x_dot', len(rhs))
        f_impl = x_dot - self.f(states_, controls_)
        model = AcadosModel()
        model.f_expl_expr = self.f(states_, controls_)
        model.f_impl_expr = f_impl
        model.x = states_
        model.xdot = x_dot
        model.u = controls_
        model.p = [] # ca.vcat([])
        model.name = 'quadrotor'

        constraints = ca.types.SimpleNamespace()
        constraints.roll_min = np.deg2rad(-85)
        constraints.pitch_min = np.deg2rad(-85)
        constraints.roll_max = np.deg2rad(85)
        constraints.pitch_max = np.deg2rad(85)
        constraints.thrust_min = 0.5*g_
        constraints.thrust_max = 1.9*g_

        self.model = model
        self.constraints = constraints