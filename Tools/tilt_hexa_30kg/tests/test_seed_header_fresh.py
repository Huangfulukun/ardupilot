"""
tests/test_seed_header_fresh.py -- Verify that the generated header is up-to-date.

Regenerates to a temp file and fails if the committed header differs.
This ensures the YAML is always the single source of truth.
"""

import os
import sys
import tempfile
import subprocess
import pytest


@pytest.fixture
def script_dir():
    return os.path.join(os.path.dirname(__file__), "..", "config")


@pytest.fixture
def committed_header():
    return os.path.join(os.path.dirname(__file__), "..", "..", "..",
                        "libraries", "AP_TiltHexa", "AP_TiltHexa_SeedDefaults.h")


def test_header_fresh(script_dir, committed_header):
    """Regenerate header to temp file and compare with committed version."""
    gen_script = os.path.join(script_dir, "generate_thx_defaults.py")
    yaml_path = os.path.join(script_dir, "tilt_hexa_30kg_seed.yaml")

    if not os.path.exists(gen_script):
        pytest.skip("Generator script not found")

    # Regenerate to temp file
    with tempfile.NamedTemporaryFile(mode="w", suffix=".h", delete=False) as tmp:
        tmp_path = tmp.name

    try:
        result = subprocess.run(
            [sys.executable, gen_script, "--yaml", yaml_path, "--out", tmp_path],
            capture_output=True, text=True, timeout=30,
        )
        if result.returncode != 0:
            pytest.fail(f"Generator failed: {result.stderr}")

        # Compare
        with open(committed_header, "r") as f:
            committed = f.read()
        with open(tmp_path, "r") as f:
            generated = f.read()

        if committed != generated:
            # Show diff
            import difflib
            diff = list(difflib.unified_diff(
                committed.splitlines(True),
                generated.splitlines(True),
                fromfile="committed",
                tofile="generated",
            ))
            diff_text = "".join(diff[:50])  # limit output
            pytest.fail(
                f"Header is stale! Run:\n"
                f"  python3 {gen_script}\n\n"
                f"Diff:\n{diff_text}"
            )
    finally:
        if os.path.exists(tmp_path):
            os.unlink(tmp_path)
