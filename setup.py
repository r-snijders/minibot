from glob import glob
from setuptools import setup

setup(
    name='minibot', version='0.1.0', packages=['minibot'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/minibot']),
                ('share/minibot', ['package.xml']),
                ('share/minibot/launch', glob('launch/*.launch.py')),
                ('share/minibot/config', glob('config/*.yaml')),
                ('share/minibot/worlds', glob('worlds/*.sdf'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Ron Snijders', maintainer_email='ronsnijdersron@gmail.com',
    description='Two-wheel rover simulator and serial hardware driver',
    license='MIT', entry_points={'console_scripts': ['hardware = minibot.hardware:main',
                                                   'vlm = minibot.vlm:main',
                                                   'reasoning = minibot.reasoning:main',
                                                   'autonomy = minibot.autonomy:main',
                                                   'velocity_gate = minibot.velocity_gate:main',
                                                   'sim_battery = minibot.sim_battery:main',
                                                   'person_detector = minibot.person_detector:main',
                                                   'speech = minibot.speech:main']},
)
