from setuptools import setup
from setuptools import find_namespace_packages

package_name = 'pan_tilt_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_namespace_packages(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Pan Tilt Controller Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pan_tilt_control = pan_tilt_controller.pan_tilt_control:main',
        ],
    },
)