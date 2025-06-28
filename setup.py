# ROS 1 setup using catkin
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=[
        'multi_target_tracking',
        'multi_target_tracking.src',
        'multi_target_tracking.src.distributed_trackers',
        'multi_target_tracking.src.trackers',
        'multi_target_tracking.src.filters',
        'multi_target_tracking.src.gaters',
        'multi_target_tracking.src.likelihoods',
        'multi_target_tracking.src.track',
    ],
)

setup(**setup_args)