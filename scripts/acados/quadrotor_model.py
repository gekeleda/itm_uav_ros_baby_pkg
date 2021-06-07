#!/usr/bin/env python
# coding=UTF-8
'''
Author: David Gekeler
Date: 2021-05-04 13:39:08
'''
import numpy as np
import casadi as ca
from acados_template import AcadosModel



class QuadRotorModel(object):
    def __init__(self, ):
        #constants
        g = 9.8066
        l = 1.

        # control inputs
        roll_ref = ca.SX.sym('roll_ref_')
        pitch_ref = ca.SX.sym('pitch_ref_')
        yaw_ref = ca.SX.sym('yaw_ref_')
        thrust_ref = ca.SX.sym('thrust_ref_')
        controls = ca.vcat([roll_ref, pitch_ref, yaw_ref, thrust_ref])

        # model constants after SI
        roll_gain = 2.477
        roll_tau = 0.477
        pitch_gain = 2.477
        pitch_tau = 0.477

        # roll_gain = 1.477
        # roll_tau = 0.477
        # pitch_gain = 1.477
        # pitch_tau = 0.477

        # model states
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        z = ca.SX.sym('z')
        vx = ca.SX.sym('vx')
        vy = ca.SX.sym('vy')
        vz = ca.SX.sym('vz')
        s = ca.SX.sym('s') # pendulum x coordinate
        r = ca.SX.sym('r') # pendulum y coordinate
        ds = ca.SX.sym('ds')
        dr = ca.SX.sym('dr')
        roll = ca.SX.sym('roll')
        pitch = ca.SX.sym('pitch')
        yaw = ca.SX.sym('yaw')
        states = ca.vcat([x, y, z, vx, vy, vz, s, r, ds, dr, roll, pitch, yaw])

        ### DYNAMICS
        dragacc1, dragacc2 = 0.0, 0.0
        h = ca.sqrt(l*l - s*s - r*r)
        ddx = thrust_ref * (ca.sin(roll)*ca.sin(yaw) + ca.cos(roll)*ca.cos(yaw)*ca.sin(pitch)) - dragacc1
        ddy = thrust_ref * (ca.cos(roll)*ca.sin(pitch)*ca.sin(yaw) - ca.sin(roll)*ca.cos(yaw)) - dragacc2
        ddz = thrust_ref * ca.cos(roll)*ca.cos(pitch) - g

        rhs = [states[3], states[4], states[5]]
        rhs.append(ddx)
        rhs.append(ddy)
        rhs.append(ddz)
        rhs.append(states[8])
        rhs.append(states[9])

        rhs.append( #dds
            -1/(l*l*h*h) * (l*l*l*l*ddx + s*s*s*(s*ddx-ddy) + l*l*s*(ds*ds+dr*dr) - l*l*ddx*(2*s*s+r*r) - s*r*r*ds*ds + s*s*s*s*s*(ddz+g)/h + s*r*r*r*ddy
            + s*s*s*r*(ddy+r*2*ddz/h)+ 2*s*s*r*ds*dr + l*l*l*l*s*ddz/h - l*l*s*r*ddy + 1/h * (2*g*s*s*s*r*r + g*l*l*l*l*s - 2*l*l*s*s*s*ddz + s*r*r*r*r*ddz
            - 2*g*l*l*s*s*s + g*s*r*r*r*r - 2*l*l*s*r*r*ddz-2*g*l*l*s*r*r
            ))
        )
        rhs.append( #ddr
            -1/(l*l*h*h) * (l*l*l*l*ddy - r*r*r*(r*ddy-ds*ds) + s*s*r*r*ddy + l*l*r*(ds*ds+dr*dr) - l*l*s*s*ddy - 2*l*l*r*r*ddy - s*s*r*dr*dr + s*r*(r*r*ddx+s*s*ddx)
            + 2*s*r*r*ds*dr - l*l*s*r*ddx + 1/h * (r*r*r*r*r*(ddz+g) + 2*s*s*r*r*r*ddz + l*l*l*l*r*ddz + 2*r*r*r*(g*s*s-l*l*ddz-2*g*l*l) + g*l*l*l*l*r + s*s*s*s*r*(g+ddz)
            - 2*l*l*s*s*r*(ddz+g)
        ))
        )

        rhs.append((roll_gain * roll_ref - roll) / roll_tau) #droll
        rhs.append((pitch_gain * pitch_ref - pitch) / pitch_tau) #dpitch
        rhs.append(yaw_ref) #dyaw

        self.f = ca.Function('f', [states, controls], [ca.vcat(rhs)])

        # acados model
        x_dot = ca.SX.sym('x_dot', len(rhs))
        f_impl = x_dot - self.f(states, controls)
        model = AcadosModel()
        model.f_expl_expr = self.f(states, controls)
        model.f_impl_expr = f_impl
        model.x = states
        model.xdot = x_dot
        model.u = controls
        model.p = [] # ca.vcat([])
        model.name = 'quadrotor'

        ### CONSTRAINTS
        constraints = ca.types.SimpleNamespace()
        constraints.roll_min = np.deg2rad(-85)
        constraints.pitch_min = np.deg2rad(-85)
        constraints.roll_max = np.deg2rad(85)
        constraints.pitch_max = np.deg2rad(85)
        constraints.yaw_min = np.deg2rad(-85)
        constraints.yaw_max = np.deg2rad(85)
        constraints.thrust_min = 0.5*g
        constraints.thrust_max = 1.9*g
        constraints.s_min = -l
        constraints.s_max = l
        constraints.r_min = -l
        constraints.r_max = l

        self.model = model
        self.constraints = constraints