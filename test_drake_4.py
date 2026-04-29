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
A = np.hstack([np.eye(n_v), -np.eye(n_v), -0.1 * np.eye(n_v)])
b = np.zeros(n_v)
# avoid np.concatenate in the loop
x_all = np.empty((n_steps - 1, 3 * n_v), dtype=object)
x_all[:, :n_v] = q2[1:]
x_all[:, n_v:2*n_v] = q2[:-1]
x_all[:, 2*n_v:] = v2[1:]

for k in range(n_steps - 1):
    prog2.AddLinearEqualityConstraint(A, b, x_all[k])

print(f"Matrix constraint (optimized): {time.time() - start:.4f}s")
