## 2025-02-20 - Fast Sum of Squares in NumPy
**Learning:** `np.vdot(x, x)` is significantly faster (often 2x-5x) than `np.sum(x**2)` for computing the sum of squared elements in an array. It avoids allocating an intermediate array and directly uses optimized BLAS routines under the hood.
**Action:** When calculating the sum of squares, especially in hot paths like cost function evaluation during trajectory optimization, prefer `np.vdot(x, x)` or `np.dot(x, x)` (for 1D) over `np.sum(x**2)` to save memory and improve speed.

## 2025-02-20 - Fast Sum of Squares in NumPy
**Learning:** `np.vdot(x, x)` is significantly faster (often 2x-5x) than `np.sum(x**2)` for computing the sum of squared elements in an array. It avoids allocating an intermediate array and directly uses optimized BLAS routines under the hood.
**Action:** When calculating the sum of squares, especially in hot paths like cost function evaluation during trajectory optimization, prefer `np.vdot(x, x)` or `np.dot(x, x)` (for 1D) over `np.sum(x**2)` to save memory and improve speed.
## 2025-05-18 - Replacing `np.linalg.norm` with `math.hypot`
**Learning:** `np.linalg.norm` involves significant dispatch overhead for small, fixed-size arrays (like 3-vectors) which are validated heavily during model generation. Using `math.hypot(arr[0], arr[1], arr[2])` yields a 3-4x performance improvement for evaluating the norm of 3-element NumPy arrays. This follows an established pattern of using `math` functions in place of `numpy` functions for scalars or very small arrays within fast-path validations.
**Action:** Always consider `math` module functions as alternatives to `numpy` equivalents when dealing with small, fixed-size arrays or scalars in frequently invoked routines (such as precondition checks).

## 2025-05-18 - Replacing `np.dot` with scalar math
**Learning:** `np.dot` involves significant dispatch overhead for small, fixed-size arrays (like 3-vectors) which are used heavily in inertia computations. Using explicit scalar math (`d[0]*d[0] + d[1]*d[1] + d[2]*d[2]`) yields a 3-4x performance improvement for evaluating the dot product of 3-element NumPy arrays. This follows an established pattern of using explicit scalar math in place of `numpy` functions for small arrays within fast-path computations.
**Action:** Always consider explicit scalar math as an alternative to `numpy` equivalents when dealing with small, fixed-size arrays in frequently invoked routines (such as parallel axis shift computations).
