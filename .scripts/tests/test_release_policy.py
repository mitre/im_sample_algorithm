import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'release_policy.py'
spec = importlib.util.spec_from_file_location('release_policy', SCRIPT)
policy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(policy)


class ReleasePolicyTests(unittest.TestCase):
    def test_cmake_version_ignores_comments(self):
        self.assertEqual(policy.cmake_version(
            '# project(im_sample_algorithm VERSION 9.0.0)\n'
            '#[=[project(im_sample_algorithm VERSION 8.0.0)]=]\n'
            'project(im_sample_algorithm VERSION 6.0.2 LANGUAGES CXX)'), '6.0.2')

    def test_cmake_version_rejects_ambiguous_or_invalid_versions(self):
        for version in ('6.0.2.1', '06.0.2', '${VERSION}', '6.0.2-rc.1'):
            with self.subTest(version=version), self.assertRaises(ValueError):
                policy.cmake_version(f'project(im_sample_algorithm VERSION {version})')
        with self.assertRaises(ValueError):
            policy.cmake_version('project(im_sample_algorithm VERSION 6.0.2)\n' * 2)

    def test_tag_identity(self):
        self.assertFalse(policy.validate_tag('6.0.2', '6.0.2'))
        self.assertTrue(policy.validate_tag('6.0.2-rc.12', '6.0.2'))
        for tag in ('v6.0.2', '6.0.3', '6.0.2.1', '6.0.2-rc.0', '6.0.2-rc.01',
                    '6.0.2-beta.1', '6.0.2-rc.1\n', '6.0.2/extra'):
            with self.subTest(tag=tag), self.assertRaises(ValueError):
                policy.validate_tag(tag, '6.0.2')

    def test_stable_requires_default_branch_but_rc_can_be_on_branch(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)

            def git(*args):
                subprocess.run(['git', *args], cwd=root, check=True, capture_output=True)

            git('init', '-b', 'master')
            git('config', 'user.email', 'test@example.com')
            git('config', 'user.name', 'Policy Test')
            cmake = root / 'CMakeLists.txt'
            cmake.write_text('project(im_sample_algorithm VERSION 6.0.1)')
            git('add', '.')
            git('commit', '-m', 'Initial')
            git('checkout', '-b', 'candidate')
            cmake.write_text('project(im_sample_algorithm VERSION 6.0.2)')
            git('commit', '-am', 'Candidate')
            git('tag', '-a', '6.0.2-rc.1', '-m', 'RC')
            git('tag', '6.0.2')
            output = root / 'github-output'

            def check(tag):
                return subprocess.run(
                    [sys.executable, str(SCRIPT), '--tag', tag, '--stable-ref', 'master'],
                    cwd=root, capture_output=True, text=True,
                    env={**os.environ, 'GITHUB_OUTPUT': str(output)})

            self.assertEqual(check('6.0.2-rc.1').returncode, 0)
            self.assertIn('prerelease=true', output.read_text())
            self.assertNotEqual(check('6.0.2').returncode, 0)
            git('checkout', 'master')
            git('merge', '--ff-only', 'candidate')
            self.assertEqual(check('6.0.2').returncode, 0)
            self.assertIn('prerelease=false', output.read_text())
            git('checkout', 'HEAD~1')
            self.assertNotEqual(check('6.0.2').returncode, 0)


if __name__ == '__main__':
    unittest.main()
