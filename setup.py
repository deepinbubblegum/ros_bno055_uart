from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['ros_bno055_uart'],
    package_dir={'': 'src'}
)

setup(**setup_args)
