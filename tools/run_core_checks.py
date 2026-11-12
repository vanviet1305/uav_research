#!/usr/bin/env python3
"""Compile and run real controller C++ tests without ROS; no dependencies downloaded."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys


def main():
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--eigen', type=Path, default=Path('/usr/include/eigen3'))
    parser.add_argument('--build-dir', type=Path)
    parser.add_argument('--cxx', default=shutil.which('g++') or shutil.which('clang++'))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    build = (args.build_dir or root / 'build' / 'core_checks').resolve()
    eigen = args.eigen.resolve()
    if not args.cxx or not (eigen / 'Eigen' / 'Dense').is_file():
        parser.error('Install a C++17 compiler and pass --eigen DIRECTORY containing Eigen/Dense')
    build.mkdir(parents=True, exist_ok=True)
    suffix = '.exe' if sys.platform == 'win32' else ''
    transcript = []

    def run(command):
        result = subprocess.run([str(part) for part in command], text=True, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, cwd=build)
        print(result.stdout, end='', flush=True)
        transcript.append('COMMAND: '+repr([str(part) for part in command])+'\n'+result.stdout)
        (build / 'validation.log').write_text('\n'.join(transcript), encoding='utf-8')
        if result.returncode:
            raise SystemExit(result.returncode)

    for name in ['test_p01', 'test_p01_controller', 'test_x500_control',
                 'p01_allocation_check', 'p01_offline_case1', 'p01_x500_offline']:
        print('Building '+name, flush=True)
        # Relative paths also work with MinGW's narrow-character linker when the
        # Windows workspace has Vietnamese characters in its absolute path.
        run([args.cxx, '-std=c++17', '-O2', '-Wall', '-Wextra', '-Wpedantic',
             '-I', os.path.relpath(root / 'include', build), '-isystem', os.path.relpath(eigen, build),
             os.path.relpath(root / 'src' / (name+'.cpp'), build), '-o', name+suffix])
        if name == 'p01_x500_offline':
            for mode, period in [('hover', '.01'), ('case1', '.01'), ('case1', '.02'),
                                 ('case1_yaw', '.01'), ('yaw_recovery', '.01')]:
                run([build / (name+suffix), mode+period+'.csv', mode, period])
        elif name == 'p01_offline_case1':
            run([build / (name+suffix), 'p01_ideal.csv'])
        else:
            run([build / (name+suffix)])
    print('PASS. Independent C++ checks only; ROS/PX4/Gazebo not exercised. Results: '+str(build))


if __name__ == '__main__':
    main()
