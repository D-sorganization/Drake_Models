from pydrake.solvers import MathematicalProgram
import numpy as np
import time

n_steps = 500
n_q = 30
n_v = 30
offset = n_q - n_v
dt = 0.1

prog = MathematicalProgram()
q = prog.NewContinuousVariables(n_steps, n_q, "q")
v = prog.NewContinuousVariables(n_steps, n_v, "v")

start = time.time()
for k in range(n_steps - 1):
    for j in range(n_v):
        prog.AddLinearEqualityConstraint(
            q[k + 1, offset + j] - q[k, offset + j] - dt * v[k + 1, j] == 0
        )
print(f"Scalar loop: {time.time() - start:.4f}s")

prog2 = MathematicalProgram()
q2 = prog2.NewContinuousVariables(n_steps, n_q, "q")
v2 = prog2.NewContinuousVariables(n_steps, n_v, "v")

start = time.time()
for k in range(n_steps - 1):
    prog2.AddLinearEqualityConstraint(
        q2[k + 1, offset:] - q2[k, offset:] - dt * v2[k + 1, :], np.zeros(n_v)
    )

print(f"Vector expression constraint: {time.time() - start:.4f}s")
