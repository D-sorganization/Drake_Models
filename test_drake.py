from pydrake.solvers import MathematicalProgram
import numpy as np
import time

prog = MathematicalProgram()
q = prog.NewContinuousVariables(100, 10, "q")
v = prog.NewContinuousVariables(100, 10, "v")

start = time.time()
for k in range(99):
    for j in range(10):
        prog.AddLinearEqualityConstraint(
            q[k + 1, j] - q[k, j] - 0.1 * v[k + 1, j] == 0
        )
print(f"Scalar loop: {time.time() - start:.4f}s")

prog2 = MathematicalProgram()
q2 = prog2.NewContinuousVariables(100, 10, "q")
v2 = prog2.NewContinuousVariables(100, 10, "v")

start = time.time()
for k in range(99):
    prog2.AddLinearEqualityConstraint(
        q2[k + 1, :] - q2[k, :] - 0.1 * v2[k + 1, :], np.zeros(10)
    )
print(f"Array constraint: {time.time() - start:.4f}s")
