import casadi as cs
import opengen as og
import numpy as np

N = 10  # The MPC horizon length
NX = 3  # The number of elements in the state vector
NU = 2  # The number of elements in the control vector
sampling_time = 0.1
NSim = 100

Q = cs.DM.eye(NX) * [1.0, 1.0, 0.0001]
R = cs.DM.eye(NU) * [0.1, 50.0]
QN = cs.DM.eye(NX) * [10.0, 10.0, 0.0001]


def dynamics_ct(_x, _u):
    return cs.vcat([_u[0] * cs.cos(_x[2]),
                    _u[0] * cs.sin(_x[2]),
                    _u[1]])


def dynamics_dt(x, u):
    dx = dynamics_ct(x, u)
    return cs.vcat([x[i] + sampling_time * dx[i] for i in range(NX)])


# The stage cost for x and u
def stage_cost(_x, _u, _x_ref=None, _u_ref=None):
    if _x_ref is None:
        _x_ref = cs.DM.zeros(_x.shape)
    if _u_ref is None:
        _u_ref = cs.DM.zeros(_u.shape)
    dx = _x - _x_ref
    du = _u - _u_ref
    return cs.mtimes([dx.T, Q, dx]) + cs.mtimes([du.T, R, du])


# The terminal cost for x
def terminal_cost(_x, _x_ref=None):
    if _x_ref is None:
        _x_ref = cs.DM.zeros(_x.shape)
    dx = _x - _x_ref
    return cs.mtimes([dx.T, QN, dx])


x_0 = cs.MX.sym("x_0", NX)
x_ref = cs.MX.sym("x_ref", NX)
u_k = [cs.MX.sym('u_' + str(i), NU) for i in range(N)]

# Create the cost function
x_t = x_0
total_cost = 0

for t in range(0, N):
    total_cost += stage_cost(x_t, u_k[t], x_ref)  # update cost
    x_t = dynamics_dt(x_t, u_k[t])  # update state

total_cost += terminal_cost(x_t, x_ref)  # terminal cost

optimization_variables = []
optimization_parameters = []

optimization_variables += u_k
optimization_parameters += [x_0]
optimization_parameters += [x_ref]

optimization_variables = cs.vertcat(*optimization_variables)
optimization_parameters = cs.vertcat(*optimization_parameters)

umin = [-2.0, -1.0] * N  # - cs.DM.ones(NU * N) * cs.inf
umax = [2.0, 1.0] * N  # cs.DM.ones(NU * N) * cs.inf

bounds = og.constraints.Rectangle(umin, umax)

problem = og.builder.Problem(optimization_variables,
                             optimization_parameters,
                             total_cost) \
    .with_constraints(bounds)

ros_config = og.config.RosConfiguration() \
    .with_package_name("open_nmpc_controller") \
    .with_node_name("open_mpc_controller_node") \
    .with_rate((int)(1.0/sampling_time)) \
    .with_description("Cool ROS node.")

build_config = og.config.BuildConfiguration()\
    .with_build_directory("optimization_engine")\
    .with_build_mode("release")\
    .with_build_c_bindings() \
    .with_ros(ros_config)

meta = og.config.OptimizerMeta()\
    .with_optimizer_name("mpc_controller")

solver_config = og.config.SolverConfiguration()\
    .with_tolerance(1e-5)

builder = og.builder.OpEnOptimizerBuilder(problem,
                                          meta,
                                          build_config,
                                          solver_config)
builder.build()
