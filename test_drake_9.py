from pydrake.solvers import MathematicalProgram
import numpy as np
import time

n_steps = 100
n_q = 30
n_v = 30
offset = n_q - n_v
dt = 0.1

prog = MathematicalProgram()
q = prog.NewContinuousVariables(n_steps, n_q, "q")
v = prog.NewContinuousVariables(n_steps, n_v, "v")

start = time.time()
added = 0
for k in range(n_steps - 1):
    for j in range(n_v):
        prog.AddLinearEqualityConstraint(
            q[k + 1, offset + j] - q[k, offset + j] - dt * v[k + 1, j] == 0
        )
        added += 1
print(f"Scalar loop: {time.time() - start:.4f}s, added={added}")

prog2 = MathematicalProgram()
q2 = prog2.NewContinuousVariables(n_steps, n_q, "q")
v2 = prog2.NewContinuousVariables(n_steps, n_v, "v")

start = time.time()
added = 0
A = np.hstack([np.eye(n_v), -np.eye(n_v), -dt * np.eye(n_v)])
b = np.zeros(n_v)
vars_all = np.empty((n_steps - 1, 3 * n_v), dtype=q2.dtype)
vars_all[:, :n_v] = q2[1:, offset:]
vars_all[:, n_v:2*n_v] = q2[:-1, offset:]
vars_all[:, 2*n_v:] = v2[1:, :]
for k in range(n_steps - 1):
    prog2.AddLinearEqualityConstraint(A, b, vars_all[k])
    added += n_v

print(f"Matrix constraint: {time.time() - start:.4f}s, added={added}")
