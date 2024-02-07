from setuptools import setup

package_name = 'perception_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tx2',
    maintainer_email='soler.pedrojavier@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scenario_classificator = perception_pkg.scenario_classificator:main',
            'scenario_visualizer = perception_pkg.scenario_visualizer:main',
            'distance_estimator = perception_pkg.distance_estimator:main',
        ],
    },
)
