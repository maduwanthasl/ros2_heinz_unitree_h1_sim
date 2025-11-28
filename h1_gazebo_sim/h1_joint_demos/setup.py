from setuptools import find_packages, setup

package_name = 'h1_joint_demos'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shevi',
    maintainer_email='hirushamaduwantha0@gmail.com',
    description='Joint-space demo nodes for Unitree H1 simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_joint_mover = h1_joint_demos.simple_joint_mover:main',
            'handshake_motion = h1_joint_demos.handshake_motion:main',
            'right_hand_wave = h1_joint_demos.right_hand_wave:main',
            'pd_ankle_balance = h1_joint_demos.pd_ankle_balance:main',

        ],
    },
    extras_require={
        'test': [
            'pytest',
        ],
    },
)
