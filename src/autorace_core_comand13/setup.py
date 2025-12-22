from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'autorace_core_comand13'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ресурсы с маской стрелки
        (os.path.join('share', package_name, 'resources', 'masks'),
         ['resources/masks/mask.png']),
        # launch-файлы
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='b_val4@mail.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "move_robot = autorace_core_comand13.get_img:main",
            "move = autorace_core_comand13.move:main",
        ],
    },
)
