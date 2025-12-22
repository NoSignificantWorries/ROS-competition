from setuptools import find_packages, setup

package_name = 'autorace_core_command_13'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/resources", ["resources/mask.png", "resources/path.json"])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dmitry',
    maintainer_email='dmitry.a.karpachev@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "sensors = autorace_core_command_13.sensors:main",
            "drive = autorace_core_command_13.drive:main",
        ],
    },
)
