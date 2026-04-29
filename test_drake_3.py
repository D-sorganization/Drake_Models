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
# eq is q[k+1] - q[k] - 0.1 * v[k+1] = 0
# we can write it as A * x = 0
# where x is [q[k+1], q[k], v[k+1]]
A = np.hstack([np.eye(n_v), -np.eye(n_v), -0.1 * np.eye(n_v)])
b = np.zeros(n_v)
for k in range(n_steps - 1):
    x = np.concatenate([q2[k+1, :], q2[k, :], v2[k+1, :]])
    prog2.AddLinearEqualityConstraint(A, b, x)

print(f"Matrix constraint: {time.time() - start:.4f}s")
