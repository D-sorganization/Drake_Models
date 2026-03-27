use ndarray::{Array2, Axis};
use numpy::{IntoPyArray, PyArray2, PyReadonlyArray2};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use rayon::prelude::*;

/// Simplified single-joint inertia (kg*m^2) — placeholder for full rigid-body model.
const DEFAULT_JOINT_INERTIA: f64 = 1.0;

/// Standard gravitational acceleration (m/s^2).
const GRAVITY_ACCEL: f64 = 9.80665;

/// Default segment length for simplified gravity torque (metres).
const DEFAULT_SEGMENT_LENGTH: f64 = 0.4;

/// Parallel batch inverse dynamics using simplified Lagrangian mechanics.
///
/// Computes torques for N configurations using a single-DOF Lagrangian model
/// per joint: `tau_j = I * qdd_j + m * g * L * cos(q_j)`, where I is a
/// simplified joint inertia, g is gravitational acceleration, and L is a
/// representative segment length.
///
/// This is a *simplified* model suitable for trajectory-optimization warm
/// starts. For production-grade inverse dynamics, use Drake's
/// `MultibodyPlant.CalcInverseDynamics`.
///
/// Parameters
/// ----------
/// positions : np.ndarray, shape (N, n_joints)
///     Joint angles (radians) for each configuration in the batch.
/// velocities : np.ndarray, shape (N, n_joints)
///     Joint angular velocities (rad/s) for each configuration.
/// accelerations : np.ndarray, shape (N, n_joints)
///     Joint angular accelerations (rad/s^2) for each configuration.
///
/// Returns
/// -------
/// np.ndarray, shape (N, n_joints)
///     Estimated joint torques (N*m) for each configuration.
///
/// Raises
/// ------
/// ValueError
///     If the three input arrays do not have identical shapes.
#[pyfunction]
fn inverse_dynamics_batch<'py>(
    py: Python<'py>,
    positions: PyReadonlyArray2<'py, f64>,
    velocities: PyReadonlyArray2<'py, f64>,
    accelerations: PyReadonlyArray2<'py, f64>,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let pos = positions.as_array();
    let vel = velocities.as_array();
    let acc = accelerations.as_array();

    // --- input validation (closes #67) ---
    if pos.shape() != vel.shape() {
        return Err(PyValueError::new_err(format!(
            "positions shape {:?} != velocities shape {:?}",
            pos.shape(),
            vel.shape()
        )));
    }
    if pos.shape() != acc.shape() {
        return Err(PyValueError::new_err(format!(
            "positions shape {:?} != accelerations shape {:?}",
            pos.shape(),
            acc.shape()
        )));
    }
    if pos.nrows() == 0 || pos.ncols() == 0 {
        return Err(PyValueError::new_err(
            "input arrays must have at least 1 row and 1 column",
        ));
    }

    let n = pos.nrows();
    let m = pos.ncols();

    let results: Vec<Vec<f64>> = (0..n)
        .into_par_iter()
        .map(|i| {
            (0..m)
                .map(|j| {
                    // Simplified Lagrangian: tau = I*qdd + m*g*L*cos(q)
                    let q = pos[[i, j]];
                    let _qd = vel[[i, j]];
                    let qdd = acc[[i, j]];
                    let inertia_term = DEFAULT_JOINT_INERTIA * qdd;
                    let gravity_term =
                        DEFAULT_JOINT_INERTIA * GRAVITY_ACCEL * DEFAULT_SEGMENT_LENGTH * q.cos();
                    inertia_term + gravity_term
                })
                .collect()
        })
        .collect();

    let mut output = Array2::<f64>::zeros((n, m));
    for (i, row) in results.iter().enumerate() {
        for (j, &val) in row.iter().enumerate() {
            output[[i, j]] = val;
        }
    }

    Ok(output.into_pyarray(py).into())
}

/// Parallel batch center-of-mass computation from Cartesian segment positions.
///
/// Computes the mass-weighted average of segment positions. Each row of
/// ``positions`` must contain Cartesian coordinates laid out as
/// ``[x0, y0, z0, x1, y1, z1, ...]`` — i.e., ``n_segments * 3`` columns.
/// Each row of ``masses`` contains the corresponding segment masses.
///
/// **Important:** The positions input must be *Cartesian* segment centres,
/// **not** joint angles. If you have joint angles, run forward kinematics
/// first to obtain segment positions.
///
/// Parameters
/// ----------
/// positions : np.ndarray, shape (N, n_segments * 3)
///     Cartesian segment centres [x0, y0, z0, x1, y1, z1, ...] for each
///     configuration in the batch.
/// masses : np.ndarray, shape (N, n_segments)
///     Segment masses for each configuration.
///
/// Returns
/// -------
/// np.ndarray, shape (N, 3)
///     Center-of-mass [x, y, z] for each configuration.
///
/// Raises
/// ------
/// ValueError
///     If ``positions`` column count is not exactly ``3 * masses`` column
///     count, or if batch sizes (row counts) differ.
#[pyfunction]
fn com_batch<'py>(
    py: Python<'py>,
    positions: PyReadonlyArray2<'py, f64>,
    masses: PyReadonlyArray2<'py, f64>,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let pos = positions.as_array();
    let mass = masses.as_array();

    // --- input validation (closes #67) ---
    if pos.nrows() != mass.nrows() {
        return Err(PyValueError::new_err(format!(
            "positions has {} rows but masses has {} rows",
            pos.nrows(),
            mass.nrows()
        )));
    }
    let n_segments = mass.ncols();
    if pos.ncols() != n_segments * 3 {
        return Err(PyValueError::new_err(format!(
            "positions must have 3*n_segments={} columns (got {}). \
             Positions must be Cartesian [x0,y0,z0,...], not joint angles.",
            n_segments * 3,
            pos.ncols()
        )));
    }
    if pos.nrows() == 0 || n_segments == 0 {
        return Err(PyValueError::new_err(
            "input arrays must have at least 1 row and 1 segment",
        ));
    }

    let n = pos.nrows();

    let results: Vec<[f64; 3]> = (0..n)
        .into_par_iter()
        .map(|i| {
            let total_mass: f64 = mass.row(i).sum();
            if total_mass <= 0.0 {
                return [0.0, 0.0, 0.0];
            }
            let mut com = [0.0f64; 3];
            for j in 0..n_segments {
                let m_j = mass[[i, j]];
                com[0] += m_j * pos[[i, j * 3]];
                com[1] += m_j * pos[[i, j * 3 + 1]];
                com[2] += m_j * pos[[i, j * 3 + 2]];
            }
            com[0] /= total_mass;
            com[1] /= total_mass;
            com[2] /= total_mass;
            com
        })
        .collect();

    let mut output = Array2::<f64>::zeros((n, 3));
    for (i, com) in results.iter().enumerate() {
        output[[i, 0]] = com[0];
        output[[i, 1]] = com[1];
        output[[i, 2]] = com[2];
    }

    Ok(output.into_pyarray(py).into())
}

/// Parallel phase interpolation between keyframes.
///
/// Parameters
/// ----------
/// phase_targets : np.ndarray, shape (n_phases, n_joints)
///     Target joint angles at each phase keyframe.
/// time_fractions : np.ndarray, shape (n_phases, 1)
///     Normalized time [0, 1] for each phase keyframe.
/// query_times : np.ndarray, shape (N, 1)
///     Times at which to interpolate.
///
/// Returns
/// -------
/// np.ndarray, shape (N, n_joints)
///     Linearly interpolated joint angles at each query time.
///
/// Raises
/// ------
/// ValueError
///     If ``time_fractions`` and ``phase_targets`` have different row counts,
///     or if column dimensions are wrong.
#[pyfunction]
fn interpolate_phases_rs<'py>(
    py: Python<'py>,
    phase_targets: PyReadonlyArray2<'py, f64>,
    time_fractions: PyReadonlyArray2<'py, f64>,
    query_times: PyReadonlyArray2<'py, f64>,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let targets = phase_targets.as_array();
    let times = time_fractions.as_array();
    let queries = query_times.as_array();

    // --- input validation (closes #67) ---
    if targets.nrows() != times.nrows() {
        return Err(PyValueError::new_err(format!(
            "phase_targets has {} rows but time_fractions has {} rows",
            targets.nrows(),
            times.nrows()
        )));
    }
    if times.ncols() != 1 {
        return Err(PyValueError::new_err(format!(
            "time_fractions must have 1 column, got {}",
            times.ncols()
        )));
    }
    if queries.ncols() != 1 {
        return Err(PyValueError::new_err(format!(
            "query_times must have 1 column, got {}",
            queries.ncols()
        )));
    }
    if targets.nrows() < 2 {
        return Err(PyValueError::new_err(
            "phase_targets must have at least 2 phases",
        ));
    }

    let n_queries = queries.nrows();
    let n_joints = targets.ncols();
    let n_phases = targets.nrows();

    let results: Vec<Vec<f64>> = (0..n_queries)
        .into_par_iter()
        .map(|i| {
            let t = queries[[i, 0]].clamp(0.0, 1.0);

            // Find bracketing phases
            let mut lo = 0usize;
            let mut hi = n_phases - 1;
            for k in 0..n_phases - 1 {
                if times[[k, 0]] <= t && t <= times[[k + 1, 0]] {
                    lo = k;
                    hi = k + 1;
                    break;
                }
            }

            let t_lo = times[[lo, 0]];
            let t_hi = times[[hi, 0]];
            let alpha = if (t_hi - t_lo).abs() < 1e-12 {
                0.0
            } else {
                (t - t_lo) / (t_hi - t_lo)
            };

            (0..n_joints)
                .map(|j| {
                    let v_lo = targets[[lo, j]];
                    let v_hi = targets[[hi, j]];
                    v_lo + alpha * (v_hi - v_lo)
                })
                .collect()
        })
        .collect();

    let mut output = Array2::<f64>::zeros((n_queries, n_joints));
    for (i, row) in results.iter().enumerate() {
        for (j, &val) in row.iter().enumerate() {
            output[[i, j]] = val;
        }
    }

    Ok(output.into_pyarray(py).into())
}

/// Drake Models Rust accelerator module.
#[pymodule]
fn drake_models_core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(inverse_dynamics_batch, m)?)?;
    m.add_function(wrap_pyfunction!(com_batch, m)?)?;
    m.add_function(wrap_pyfunction!(interpolate_phases_rs, m)?)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gravity_constant() {
        assert!((GRAVITY_ACCEL - 9.80665).abs() < 1e-10);
    }

    #[test]
    fn test_default_segment_length_positive() {
        assert!(DEFAULT_SEGMENT_LENGTH > 0.0);
    }

    #[test]
    fn test_default_joint_inertia_positive() {
        assert!(DEFAULT_JOINT_INERTIA > 0.0);
    }
}
