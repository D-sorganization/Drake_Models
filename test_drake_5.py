from pydrake.solvers import MathematicalProgram
import numpy as np
import time

n_steps = 500
n_v = 30

prog = MathematicalProgram()
q = prog.NewContinuousVariables(n_steps, n_v, "q")
v = prog.NewContinuousVariables(n_steps, n_v, "v")

start = time.time()
for k in range(n_steps - 1):
    for j in range(n_v):
        prog.AddLinearEqualityConstraint(
            q[k + 1, j] - q[k, j] - 0.1 * v[k + 1, j] == 0
        )
print(f"Scalar loop: {time.time() - start:.4f}s")

prog2 = MathematicalProgram()
q2 = prog2.NewContinuousVariables(n_steps, n_v, "q")
v2 = prog2.NewContinuousVariables(n_steps, n_v, "v")

start = time.time()
# q2[k+1] - q2[k] - 0.1 * v2[k+1] == 0
# we can write it as A * x = 0
for k in range(n_steps - 1):
    prog2.AddLinearEqualityConstraint(q2[k + 1, :] - q2[k, :] - 0.1 * v2[k + 1, :], np.zeros(n_v))

print(f"Vector constraint: {time.time() - start:.4f}s")

prog3 = MathematicalProgram()
q3 = prog3.NewContinuousVariables(n_steps, n_v, "q")
v3 = prog3.NewContinuousVariables(n_steps, n_v, "v")
start = time.time()
dt = 0.1
A = np.hstack([np.eye(n_v), -np.eye(n_v), -dt * np.eye(n_v)])
b = np.zeros(n_v)
vars_all = np.empty((n_steps - 1, 3 * n_v), dtype=q3.dtype)
vars_all[:, :n_v] = q3[1:, :]
vars_all[:, n_v:2*n_v] = q3[:-1, :]
vars_all[:, 2*n_v:] = v3[1:, :]
for k in range(n_steps - 1):
    prog3.AddLinearEqualityConstraint(A, b, vars_all[k])
print(f"Matrix constraint (optimized): {time.time() - start:.4f}s")
