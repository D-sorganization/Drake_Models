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
for k in range(n_steps - 1):
    prog2.AddLinearEqualityConstraint(
        q2[k + 1, :] - q2[k, :] - 0.1 * v2[k + 1, :], np.zeros(n_v)
    )
print(f"Array constraint: {time.time() - start:.4f}s")
