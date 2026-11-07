import importlib.util
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).parents[1]
spec = importlib.util.spec_from_file_location("bump_pins", ROOT / "scripts/bump_pins.py")
bp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bp)

PYPROJECT = '''[project]
name = "examplebot"
version = "4.0.0"
dependencies = [
    "raccoon-cli==171",
    "raccoon-library==153",
    # "raccoon-library[sim]==153",   # uncomment to run the simulator locally
]
'''


class VersionTests(unittest.TestCase):
    def test_tag_and_pypi_spelling_are_the_same(self):
        self.assertEqual("173", bp.normalize_version("v173"))
        self.assertEqual("173", bp.normalize_version("173"))
        self.assertEqual("1.0.159", bp.normalize_version(" v1.0.159 "))

    def test_garbage_is_rejected(self):
        for value in ("", "v", "latest", "1.x", "173; rm", "../1"):
            with self.subTest(value), self.assertRaises(ValueError):
                bp.normalize_version(value)


class BumpTests(unittest.TestCase):
    def test_moves_both_pins_and_the_commented_sim_line(self):
        text, old = bp.bump(PYPROJECT, "v173", "v155")
        self.assertIn('"raccoon-cli==173"', text)
        self.assertIn('"raccoon-library==155"', text)
        self.assertIn('# "raccoon-library[sim]==155"', text)
        self.assertEqual({"cli": "171", "lib": "153"}, old)

    def test_downgrade_is_applied_the_bundle_is_the_truth(self):
        text, _ = bp.bump(PYPROJECT, "100", "100")
        self.assertIn('"raccoon-cli==100"', text)

    def test_same_versions_change_nothing(self):
        text, _ = bp.bump(PYPROJECT, "171", "153")
        self.assertEqual(PYPROJECT, text)

    def test_bundle_sets_the_project_version(self):
        text, _ = bp.bump(PYPROJECT, "171", "153", "2026.11.07-2")
        self.assertIn('version = "2026.11.07-2"', text)
        self.assertEqual(PYPROJECT.replace("4.0.0", "2026.11.07-2"), text)

    def test_no_bundle_keeps_the_version(self):
        text, _ = bp.bump(PYPROJECT, "173", "155")
        self.assertIn('version = "4.0.0"', text)

    def test_bad_bundle_name_is_rejected(self):
        with self.assertRaises(ValueError):
            bp.bump(PYPROJECT, "171", "153", '1"\nx')

    def test_missing_pin_is_an_error(self):
        with self.assertRaises(ValueError):
            bp.bump("dependencies = []\n", "1", "1")

    def test_only_the_pins_change(self):
        text, _ = bp.bump(PYPROJECT, "173", "155")
        self.assertEqual(PYPROJECT.replace("171", "173").replace("153", "155"), text)


class CliTests(unittest.TestCase):
    def run_script(self, *args, text=PYPROJECT):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "pyproject.toml"
            path.write_text(text)
            output = Path(tmp) / "out"
            result = subprocess.run(
                [sys.executable, str(ROOT / "scripts/bump_pins.py"), "--pyproject", str(path), *args],
                capture_output=True, text=True, env={**os.environ, "GITHUB_OUTPUT": str(output)})
            return result, path.read_text(), output.read_text() if output.exists() else ""

    def test_changed(self):
        result, text, out = self.run_script("--cli", "v173", "--lib", "v155")
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn("raccoon-cli==173", text)
        self.assertIn("changed=true", out)
        self.assertIn("old_cli=171", out)

    def test_unchanged(self):
        result, text, out = self.run_script("--cli", "171", "--lib", "153")
        self.assertEqual(0, result.returncode)
        self.assertEqual(PYPROJECT, text)
        self.assertIn("changed=false", out)

    def test_bad_version_fails_and_leaves_the_file(self):
        result, text, _ = self.run_script("--cli", "main", "--lib", "153")
        self.assertEqual(1, result.returncode)
        self.assertEqual(PYPROJECT, text)


if __name__ == "__main__":
    unittest.main()
