import timeit
import numpy as np

setup = "import numpy as np; n=200; positions=np.random.rand(n, 20); dt=0.01; velocities=np.empty(positions.shape, dtype=positions.dtype); velocities[0]=0; np.subtract(positions[1:], positions[:-1], out=velocities[1:])"

print("dt_inv:", timeit.timeit("dt_inv = 1.0 / dt; velocities[1:] *= dt_inv", setup=setup, number=100000))
print("/= dt:", timeit.timeit("velocities[1:] /= dt", setup=setup, number=100000))
