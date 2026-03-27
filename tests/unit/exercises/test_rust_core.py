"""Tests for Rust accelerator Cargo.toml configuration."""

import os


class TestRustCoreSetup:
    def test_cargo_toml_exists(self) -> None:
        """Verify Cargo.toml exists in rust_core directory."""
        repo_root = os.path.dirname(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        )
        cargo_path = os.path.join(repo_root, "rust_core", "Cargo.toml")
        assert os.path.isfile(cargo_path), f"Cargo.toml not found at {cargo_path}"

    def test_cargo_toml_has_pyo3(self) -> None:
        """Verify Cargo.toml declares pyo3 dependency."""
        repo_root = os.path.dirname(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        )
        cargo_path = os.path.join(repo_root, "rust_core", "Cargo.toml")
        with open(cargo_path) as f:
            content = f.read()
        assert "pyo3" in content
        assert "rayon" in content
        assert "ndarray" in content
        assert "numpy" in content

    def test_cargo_toml_package_name(self) -> None:
        """Verify the crate is named drake_models_core."""
        repo_root = os.path.dirname(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        )
        cargo_path = os.path.join(repo_root, "rust_core", "Cargo.toml")
        with open(cargo_path) as f:
            content = f.read()
        assert 'name = "drake_models_core"' in content

    def test_lib_rs_exists(self) -> None:
        """Verify lib.rs exists."""
        repo_root = os.path.dirname(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        )
        lib_path = os.path.join(repo_root, "rust_core", "src", "lib.rs")
        assert os.path.isfile(lib_path), f"lib.rs not found at {lib_path}"

    def test_lib_rs_has_required_functions(self) -> None:
        """Verify lib.rs declares required PyO3 functions."""
        repo_root = os.path.dirname(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        )
        lib_path = os.path.join(repo_root, "rust_core", "src", "lib.rs")
        with open(lib_path) as f:
            content = f.read()
        assert "inverse_dynamics_batch" in content
        assert "com_batch" in content
        assert "interpolate_phases_rs" in content
