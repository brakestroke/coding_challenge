from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'diff2ackermann'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luca',
    maintainer_email='luca@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'diff_to_ackermann = diff2ackermann.module:main',
        'diff_cmd_publisher = diff2ackermann.diff_cmd_pub:main',
        ],
        
    },
)
