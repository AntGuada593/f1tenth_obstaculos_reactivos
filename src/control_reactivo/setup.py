from setuptools import setup

package_name = 'control_reactivo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anthony',
    maintainer_email='anthony@todo.todo',
    description='Controlador F1TENTH',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ejecutable_pure_pursuit = control_reactivo.nodo_pure_pursuit:main',
            'ejecutable_gap = control_reactivo.nodo_gap_follower:main',
            'ejecutable_zombie = control_reactivo.zombie_gap_follower:main',
        ],
    },
)
