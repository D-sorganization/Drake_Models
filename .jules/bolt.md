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

## 2025-05-18 - Loop-Invariant Hoisting in Trajectory Setup
**Learning:** In Drake trajectory setup routines (like `_add_control_costs`), generating matrices like `weight * np.eye(n_u)` or `np.zeros(n_u)` inside the loop over `n_steps` incurs massive overhead. In our test, placing allocations inside the loop was ~70x slower than pre-computing them.
**Action:** Always hoist loop-invariant matrix/array allocations (like `np.zeros`, `np.eye`, `np.ones`) outside the `n_steps` loops when setting up costs and constraints for mathematical programs.
## 2025-05-18 - Loop-Invariant Allocation Hoisting in Phase Setup
**Learning:** Even in loops with a smaller number of iterations (like phase setups), repeatedly generating constant dense matrices with `np.eye()` incurs unnecessary allocation overhead and time complexity. In Drake programs, these matrices can be safely precomputed and reused across multiple `AddQuadraticCost` calls.
**Action:** Always identify identical, loop-invariant matrices (like state and terminal Q matrices) and extract their computation to before the loop.

## 2025-05-18 - Replacing `np.concatenate` in loops with preallocated slices
**Learning:** In constraint setup routines (`_add_dynamics_constraints`), using `np.concatenate` inside a tight loop over `n_steps` to assemble variable slices is a bottleneck. It incurs continuous memory allocations and list creation overhead. Preallocating a monolithic array (`np.empty`) and block-assigning slices before the loop avoids per-iteration allocation and yields a ~3x performance improvement in generating mathematical program constraints.
**Action:** When feeding slice-assembled vectors or equations into solvers repeatedly, prefer allocating one large array outside the loop and filling it via slice assignments over repeatedly calling `np.concatenate` inside the loop.
## 2025-05-18 - Matrix-vector constraints instead of scalar loops
**Learning:** Adding constraints element-by-element inside a nested python loop (`prog.AddLinearEqualityConstraint(q[..., j] ... )`) creates significant overhead by continuously creating expression objects and solver bindings. Utilizing matrix-vector structures (like `Ax = b`) allows adding constraints in bulk (`prog.AddLinearEqualityConstraint(A, b, vars_all[k])`), resulting in ~2x-3x speedup.
**Action:** When adding linear or quadratic equality constraints across multiple dimensions, prefer matrix-vector equations over nested dimensional loops. Combine this with monolithic preallocated variable blocks to maximize the solver construction speed.
