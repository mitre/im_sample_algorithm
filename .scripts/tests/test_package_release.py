import json
import platform
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'package_release.py'
ROOT = SCRIPT.parents[1]


class PackageReleaseTests(unittest.TestCase):
    def test_archive_contains_installation_and_cxx17_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            prefix = root / 'install'
            prefix.mkdir()
            (prefix / 'LICENSE').write_text('test license')
            version = subprocess.check_output(
                [sys.executable, str(SCRIPT.parent / 'release_policy.py')],
                cwd=ROOT, text=True).strip().split()[-1]
            result = subprocess.run(
                [sys.executable, str(SCRIPT), '--prefix', str(prefix),
                 '--tag', f'{version}-rc.1', '--platform',
                 'macos15' if platform.system() == 'Darwin' else 'ubuntu22.04',
                 '--output', str(root / 'dist')], cwd=ROOT, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            packages = list((root / 'dist').glob('*.tar.gz'))
            self.assertEqual(len(packages), 1)
            with tarfile.open(packages[0]) as archive:
                metadata_path = next(name for name in archive.getnames()
                                     if name.endswith('/build-metadata.json'))
                metadata = json.load(archive.extractfile(metadata_path))
                self.assertEqual(metadata['cxx_standard'], 17)
                self.assertEqual(metadata['tag'], f'{version}-rc.1')
                self.assertEqual(metadata['version'], version)
                self.assertEqual(len(metadata['commit']), 40)
                self.assertIn(metadata_path.rsplit('/', 1)[0] + '/LICENSE', archive.getnames())

    def test_invalid_tag_does_not_create_assets(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            result = subprocess.run(
                [sys.executable, str(SCRIPT), '--prefix', str(root),
                 '--tag', '0.0.0', '--platform', 'macos15',
                 '--output', str(root / 'dist')], cwd=ROOT, capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse((root / 'dist').exists())
            self.assertFalse((root / 'build-metadata.json').exists())


if __name__ == '__main__':
    unittest.main()
