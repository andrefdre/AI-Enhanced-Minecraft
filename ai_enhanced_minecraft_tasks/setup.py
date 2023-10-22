from glob import glob
import os
from setuptools import setup

package_name = 'ai_enhanced_minecraft_tasks'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andre',
    maintainer_email='andref@ua.pt',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate_datasets = ai_enhanced_minecraft_tasks.generate_datasets:main',
        ],
    },
)
