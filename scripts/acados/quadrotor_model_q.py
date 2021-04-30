#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2021-03-26 12:03:00
LastEditors: Wei Luo
LastEditTime: 2021-03-29 15:50:06
Note: Note
'''
import casadi as ca
from acados_template import AcadosModel

class QuadRotorModel(object):
    def __init__(self,):
        g_ = 9.8066

        # control input
        roll_rate_ref_ = ca.SX.sym('roll_rate_ref')
        pitch_rate_ref_ = ca.SX.sym('pitch_rate_ref')
        yaw_rate_ref_ = ca.SX.sym('yaw_rate_ref')
        thrust_ref_ = ca.SX.sym('thrust_ref')
        controls = ca.vcat([roll_rate_ref_, pitch_rate_ref_, yaw_rate_ref_, thrust_ref_])

        # model state
        x_ = ca.SX.sym('x')
        y_ = ca.SX.sym('y')
        z_ = ca.SX.sym('z')
        vx_ = ca.SX.sym('vx')
        vy_ = ca.SX.sym('vy')
        vz_ = ca.SX.sym('vz')
        qw_ = ca.SX.sym('qw')
        qx_ = ca.SX.sym('qx')
        qy_ = ca.SX.sym('qy')
        qz_ = ca.SX.sym('qz')

        # states  [p, q, v]
        states = ca.vcat([x_, y_, z_, qw_, qx_, qy_, qz_, vx_, vy_, vz_])

        rhs = [
            vx_,
            vy_,
            vz_,
            0.5*(-roll_rate_ref_*qx_- pitch_rate_ref_*qy_ - yaw_rate_ref_*qz_),
            0.5*(roll_rate_ref_*qw_ + yaw_rate_ref_*qy_ - pitch_rate_ref_*qz_),
            0.5*(pitch_rate_ref_*qw_ - yaw_rate_ref_*qx_ + roll_rate_ref_*qz_),
            0.5*(yaw_rate_ref_*qw_ + pitch_rate_ref_*qx_ - roll_rate_ref_*qy_),
            2*(qw_*qy_ + qx_*qz_) * thrust_ref_,
            2*(qy_*qz_ - qw_*qx_) * thrust_ref_,
            (qw_*qw_ - qx_*qx_ - qy_*qy_ + qz_*qz_) * thrust_ref_ - g_,
        ]

        self.f = ca.Function('f', [states, controls], [ca.vcat(rhs)])

        x_dot = ca.SX.sym('x_dot', len(rhs))
        f_impl = x_dot - self.f(states, controls)

        model = AcadosModel()
        model.f_expl_expr = self.f(states, controls)
        model.f_impl_expr = f_impl
        model.x = states
        model.xdot = x_dot
        model.u = controls
        model.p = []
        model.name = 'quadrotor_q'

        constraints = ca.types.SimpleNamespace()
        constraints.roll_rate_min = -6.0
        constraints.roll_rate_max = 6.0
        constraints.pitch_rate_min = -6.0
        constraints.pitch_rate_max = 6.0
        constraints.yaw_rate_min = -3.14
        constraints.yaw_rate_max = 3.14
        constraints.thrust_min = 2.0
        constraints.thrust_max = g_*1.5


        self.model = model
        self.constraints = constraints