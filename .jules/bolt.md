## 2025-02-20 - Fast Sum of Squares in NumPy
**Learning:** `np.vdot(x, x)` is significantly faster (often 2x-5x) than `np.sum(x**2)` for computing the sum of squared elements in an array. It avoids allocating an intermediate array and directly uses optimized BLAS routines under the hood.
**Action:** When calculating the sum of squares, especially in hot paths like cost function evaluation during trajectory optimization, prefer `np.vdot(x, x)` or `np.dot(x, x)` (for 1D) over `np.sum(x**2)` to save memory and improve speed.

## 2025-02-20 - Fast Sum of Squares in NumPy
**Learning:** `np.vdot(x, x)` is significantly faster (often 2x-5x) than `np.sum(x**2)` for computing the sum of squared elements in an array. It avoids allocating an intermediate array and directly uses optimized BLAS routines under the hood.
**Action:** When calculating the sum of squares, especially in hot paths like cost function evaluation during trajectory optimization, prefer `np.vdot(x, x)` or `np.dot(x, x)` (for 1D) over `np.sum(x**2)` to save memory and improve speed.
## 2025-05-18 - Replacing `np.linalg.norm` with `math.hypot`
**Learning:** `np.linalg.norm` involves significant dispatch overhead for small, fixed-size arrays (like 3-vectors) which are validated heavily during model generation. Using `math.hypot(arr[0], arr[1], arr[2])` yields a 3-4x performance improvement for evaluating the norm of 3-element NumPy arrays. This follows an established pattern of using `math` functions in place of `numpy` functions for scalars or very small arrays within fast-path validations.
**Action:** Always consider `math` module functions as alternatives to `numpy` equivalents when dealing with small, fixed-size arrays or scalars in frequently invoked routines (such as precondition checks).

## 2025-05-18 - Fast sum-of-squares for small fixed vectors
**Learning:** For small, fixed-size 3-vectors (like displacement in 3D geometry calculations), using `np.dot(d, d)` is slower than explicitly unpacking and doing a manual sum of squares (`dx*dx + dy*dy + dz*dz`). The dispatch and overhead of NumPy overshadows the BLAS optimization on tiny inputs.
**Action:** When computing dot products or sum-of-squares on explicit 3-element arrays in hot geometry/inertia functions, prefer manual multiplication and addition to bypass `numpy` dispatch overhead.
## 2025-05-18 - Fast array validation in preconditions
**Learning:** `np.all(np.isfinite(a))` involves Python function call overhead and dispatch. Calling `.all()` directly on the array returned by `np.isfinite` (`np.isfinite(a).all()`) avoids this overhead, yielding a ~40% speedup for array finiteness checks.
**Action:** Prefer `arr.all()` over `np.all(arr)` where `arr` is an ndarray, particularly in hot-path precondition validations.

## 2025-05-18 - Fast shape checking without np.asarray overhead
**Learning:** `np.asarray(arr)` introduces overhead when checking shapes of objects that are already ndarrays. A fast path that extracts the shape using `getattr(arr, "shape", None)` before falling back to `np.asarray()` yields ~10-20% speedup for NumPy arrays.
**Action:** When enforcing shape constraints on generic array-like objects in frequently called routines, attempt to extract `.shape` directly to avoid `np.asarray` overhead.
