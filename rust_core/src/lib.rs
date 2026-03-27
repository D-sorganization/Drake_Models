use ndarray::{Array2, Axis};
use numpy::{IntoPyArray, PyArray2, PyReadonlyArray2};
use pyo3::prelude::*;
use rayon::prelude::*;

/// Parallel batch inverse dynamics: compute torques for N configurations.
///
/// Parameters
/// ----------
/// positions : np.ndarray, shape (N, n_joints)
///     Joint positions for each configuration in the batch.
/// velocities : np.ndarray, shape (N, n_joints)
///     Joint velocities for each configuration.
/// accelerations : np.ndarray, shape (N, n_joints)
///     Joint accelerations for each configuration.
///
/// Returns
/// -------
/// np.ndarray, shape (N, n_joints)
///     Computed torques for each configuration (placeholder: tau = acc - vel).
#[pyfunction]
fn inverse_dynamics_batch<'py>(
    py: Python<'py>,
    positions: PyReadonlyArray2<'py, f64>,
    velocities: PyReadonlyArray2<'py, f64>,
    accelerations: PyReadonlyArray2<'py, f64>,
) -> Bound<'py, PyArray2<f64>> {
    let pos = positions.as_array();
    let vel = velocities.as_array();
    let acc = accelerations.as_array();

    let n = pos.nrows();
    let m = pos.ncols();

    let results: Vec<Vec<f64>> = (0..n)
        .into_par_iter()
        .map(|i| {
            (0..m)
                .map(|j| {
                    // Placeholder inverse dynamics: tau = acc - vel
                    // Real implementation would use recursive Newton-Euler
                    let _q = pos[[i, j]];
                    let qd = vel[[i, j]];
                    let qdd = acc[[i, j]];
                    qdd - qd
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

    output.into_pyarray(py).into()
}

/// Parallel batch center-of-mass computation.
///
/// Parameters
/// ----------
/// positions : np.ndarray, shape (N, n_joints)
///     Joint positions for each configuration.
/// masses : np.ndarray, shape (N, n_segments)
///     Segment masses for each configuration.
///
/// Returns
/// -------
/// np.ndarray, shape (N, 3)
///     Center-of-mass [x, y, z] for each configuration.
#[pyfunction]
fn com_batch<'py>(
    py: Python<'py>,
    positions: PyReadonlyArray2<'py, f64>,
    masses: PyReadonlyArray2<'py, f64>,
) -> Bound<'py, PyArray2<f64>> {
    let pos = positions.as_array();
    let mass = masses.as_array();

    let n = pos.nrows();

    let results: Vec<[f64; 3]> = (0..n)
        .into_par_iter()
        .map(|i| {
            // Placeholder CoM: weighted average of joint positions
            let total_mass: f64 = mass.row(i).sum();
            if total_mass <= 0.0 {
                return [0.0, 0.0, 0.0];
            }
            let n_joints = pos.ncols();
            let mut com = [0.0f64; 3];
            for j in 0..n_joints.min(mass.ncols()) {
                let m_j = mass[[i, j]];
                com[0] += m_j * pos[[i, j]];
                if j + 1 < n_joints {
                    com[1] += m_j * pos[[i, j + 1]];
                }
                if j + 2 < n_joints {
                    com[2] += m_j * pos[[i, j + 2]];
                }
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

    output.into_pyarray(py).into()
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
#[pyfunction]
fn interpolate_phases_rs<'py>(
    py: Python<'py>,
    phase_targets: PyReadonlyArray2<'py, f64>,
    time_fractions: PyReadonlyArray2<'py, f64>,
    query_times: PyReadonlyArray2<'py, f64>,
) -> Bound<'py, PyArray2<f64>> {
    let targets = phase_targets.as_array();
    let times = time_fractions.as_array();
    let queries = query_times.as_array();

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

    output.into_pyarray(py).into()
}

/// Drake Models Rust accelerator module.
#[pymodule]
fn drake_models_core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(inverse_dynamics_batch, m)?)?;
    m.add_function(wrap_pyfunction!(com_batch, m)?)?;
    m.add_function(wrap_pyfunction!(interpolate_phases_rs, m)?)?;
    Ok(())
}
