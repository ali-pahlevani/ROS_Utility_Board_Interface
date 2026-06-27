import os

from setuptools import setup

package_name = 'ros_utility_board_interface'
here = os.path.abspath(os.path.dirname(__file__))

try:
    with open(os.path.join(here, 'README.md'), encoding='utf-8') as f:
        long_description = f.read()
except OSError:
    long_description = 'RUBI - ROS Utility Board Interface.'

setup(
    name=package_name,
    version='2.0.0',
    py_modules=['rubi', 'rubi_ops'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'rubi_rules.yaml']),
    ],
    python_requires='>=3.8',
    install_requires=['setuptools', 'dearpygui', 'pyyaml', 'psutil'],
    zip_safe=True,
    author='Ali Pahlevani',
    author_email='a.pahlevani1998@gmail.com',
    maintainer='Ali Pahlevani',
    maintainer_email='a.pahlevani1998@gmail.com',
    description='RUBI - ROS Utility Board Interface: a lightweight single-window '
                'ROS 2 control board.',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='https://github.com/ali-pahlevani/ROS_Utility_Board_Interface',
    project_urls={
        'Source': 'https://github.com/ali-pahlevani/ROS_Utility_Board_Interface',
        'Website': 'https://www.SLAMbotics.org',
    },
    license='MIT',
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Topic :: Scientific/Engineering',
    ],
    keywords='ros2 ros monitoring introspection robotics dashboard qos rosbag',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rubi = rubi:main',
        ],
    },
)
