#!/usr/bin/env python3
"""Archive a tested installation with compiler and commit metadata."""

import argparse
import json
from pathlib import Path
import platform
import subprocess
import tarfile

from release_policy import cmake_version, validate_tag


def command(*argv):
    return subprocess.check_output(argv, text=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, required=True)
    parser.add_argument('--tag', required=True)
    parser.add_argument('--platform', required=True,
                        choices=['rockylinux8', 'rockylinux9', 'ubuntu22.04', 'macos15'])
    parser.add_argument('--output', type=Path, default=Path('dist'))
    args = parser.parse_args()
    version = cmake_version(Path('CMakeLists.txt').read_text())
    validate_tag(args.tag, version)
    if not args.prefix.is_dir():
        parser.error('The installation prefix must exist')

    compiler = 'appleclang' if args.platform.startswith('macos') else 'gcc'
    version_flag = '-dumpversion' if compiler == 'appleclang' else '-dumpfullversion'
    metadata = {
        'version': version,
        'tag': args.tag,
        'commit': command('git', 'rev-parse', 'HEAD'),
        'platform': args.platform,
        'architecture': platform.machine(),
        'compiler': command('c++', '--version'),
        'compiler_version': command('c++', version_flag),
        'cmake': command('cmake', '--version'),
        'build_type': 'Release',
        'cxx_standard': 17,
    }
    (args.prefix / 'build-metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    args.output.mkdir(parents=True, exist_ok=True)
    name = (f'im_sample_algorithm-{args.tag}-{args.platform}-{platform.machine()}'
            f'-{compiler}-{metadata["compiler_version"]}-Release')
    with tarfile.open(args.output / f'{name}.tar.gz', 'w:gz') as archive:
        archive.add(args.prefix, arcname=name)
    print(args.output / f'{name}.tar.gz')


if __name__ == '__main__':
    main()
