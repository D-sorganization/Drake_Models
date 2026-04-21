## 2024-05-24 - Numpy overhead in scalar validation
**Learning:** Using `np.isfinite` for scalar validation (like in precondition checks) introduces significant overhead compared to `math.isfinite` (~10x slower). When a scalar check is called heavily (e.g. during model generation with many geometry calculations), this overhead adds up.
**Action:** Use `math.isfinite` instead of `np.isfinite` for validating scalar values. Reserve numpy functions for arrays.
