from glob import glob
from setuptools import setup

setup(
    name='minibot', version='0.1.0', packages=['minibot'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/minibot']),
                ('share/minibot', ['package.xml']),
                ('share/minibot/launch', glob('launch/*.launch.py')),
                ('share/minibot/worlds', glob('worlds/*.sdf'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Ron Snijders', maintainer_email='ronsnijdersron@gmail.com',
    description='Two-wheel rover simulator and serial hardware driver',
    license='MIT', entry_points={'console_scripts': ['hardware = minibot.hardware:main',
                                                   'dock_demo = minibot.dock_demo:main']},
)
