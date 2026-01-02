from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
import sys
import os

try:
    import pybind11
    PYBIND11_AVAILABLE = True
except ImportError:
    PYBIND11_AVAILABLE = False
    print("WARNING: pybind11 not found. C++ PID will not be available.")

package_name = 'auv_slam'

# Only build C++ extension if pybind11 is available
ext_modules = []
if PYBIND11_AVAILABLE:
    # C++ PID source files
    pid_sources = [
        'pid_cpp/wrapper.cpp',
        'pid_cpp/heave_pid_rt.cpp',
    ]

    # Include directories
    include_dirs = [
        pybind11.get_include(),
        'pid_cpp',
    ]

    # Compiler flags
    extra_compile_args = [
        '-std=c++17',
        '-O3',
        '-fPIC',
        '-Wall',
        '-Wno-unused-variable',
        '-Wno-unused-parameter',
    ]

    # Create extension module
    ext_modules = [
        Extension(
            'auv_slam.heave_pid_rt_py',
            sources=pid_sources,
            include_dirs=include_dirs,
            language='c++',
            extra_compile_args=extra_compile_args,
        )
    ]

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/autonomous_gate.launch.py']),
        ('share/' + package_name + '/launch', ['launch/ball.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Autonomous underwater vehicle SLAM and navigation with C++ PID',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gate_detector.py = auv_slam.gate_detector:main',
            'gate_navigator.py = auv_slam.gate_navigator:main',
            'pwm_mapper.py = auv_slam.pwm_mapper:main',
            'safety_monitor.py = auv_slam.safety_monitor:main',
            'serial_bridge.py = auv_slam.serial_bridge:main',
            'ball_detector.py = auv_slam.ball_detector:main',
            'ball_follower.py = auv_slam.ball_follower:main',
            'heave_pid_controller.py = auv_slam.heave_pid_controller:main',
            'cmd_mixer.py = auv_slam.cmd_mixer:main',
            'test.py = auv_slam.test:main'
        ],
    },
    ext_modules=ext_modules,
)