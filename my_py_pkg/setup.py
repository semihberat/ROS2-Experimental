from setuptools import find_packages, setup

package_name = 'my_py_pkg'

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
    maintainer='semih',
    maintainer_email='dev.berat@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "my_first_node = my_py_pkg.my_first_node:main",
            "first_node_oop = my_py_pkg.first_node_oop:main",
            "oop_py_template = my_py_pkg.oop_py_template:main",
            "robot_news_station = my_py_pkg.robot_news_station:main",
            "robot_listener = my_py_pkg.robot_listener_node:main",
            "hybrid_robot = my_py_pkg.hybrid_node:main",
            "add_two_ints_server = my_py_pkg.add_two_ints_server:main",
            "add_two_ints_client_no_oop = my_py_pkg.add_two_ints_client_no_oop:main",
            "add_two_ints_client = my_py_pkg.add_two_ints_client:main"
        ],
    },
)
