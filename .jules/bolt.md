## 2025-02-20 - Fast Sum of Squares in NumPy
**Learning:** `np.vdot(x, x)` is significantly faster (often 2x-5x) than `np.sum(x**2)` for computing the sum of squared elements in an array. It avoids allocating an intermediate array and directly uses optimized BLAS routines under the hood.
**Action:** When calculating the sum of squares, especially in hot paths like cost function evaluation during trajectory optimization, prefer `np.vdot(x, x)` or `np.dot(x, x)` (for 1D) over `np.sum(x**2)` to save memory and improve speed.

## 2025-02-20 - Fast Sum of Squares in NumPy
**Learning:** `np.vdot(x, x)` is significantly faster (often 2x-5x) than `np.sum(x**2)` for computing the sum of squared elements in an array. It avoids allocating an intermediate array and directly uses optimized BLAS routines under the hood.
**Action:** When calculating the sum of squares, especially in hot paths like cost function evaluation during trajectory optimization, prefer `np.vdot(x, x)` or `np.dot(x, x)` (for 1D) over `np.sum(x**2)` to save memory and improve speed.
