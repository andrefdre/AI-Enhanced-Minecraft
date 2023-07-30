from setuptools import setup

package_name = 'ai_enhanced_minecraft_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andre',
    maintainer_email='andref@ua.pt',
    description='This repository serves as a comprehensive resource for developers and Minecraft enthusiasts interested in exploring the capabilities of AI-controlled players within the game.',
    license='GPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = ai_enhanced_minecraft_bringup.publisher:main',
            'listener = ai_enhanced_minecraft_bringup.listener:main',
            'capture_keys = ai_enhanced_minecraft_bringup.capture_keys:main',
            'fabric_api = ai_enhanced_minecraft_bringup.fabric_api:main',
        ],
    },
)
