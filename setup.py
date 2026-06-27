from setuptools import setup

package_name = 'ros_utility_board_interface'

setup(
    name=package_name,
    version='2.0.0',
    py_modules=['rubi', 'rubi_ops'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'rubi_rules.yaml']),
    ],
    install_requires=['setuptools', 'dearpygui', 'pyyaml', 'psutil'],
    zip_safe=True,
    maintainer='Ali Pahlevani',
    maintainer_email='a.pahlevani1998@gmail.com',
    description='RUBI - ROS Utility Board Interface: a lightweight ROS 2 control board.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rubi = rubi:main',
        ],
    },
)
