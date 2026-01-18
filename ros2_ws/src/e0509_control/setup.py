from setuptools import setup

package_name = 'e0509_control'

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
    maintainer='jh',
    maintainer_email='jh@todo.todo',
    description='MoveIt2 control for Doosan E0509',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 🔥 이 줄이 없어서 에러가 난 것
            'moveit_controller = e0509_control.moveit_controller:main',
            'gui_app = e0509_control.gui_app:main',
        ],
    },
)
