from setuptools import find_packages
from setuptools import setup

package_name = 'turtlebot3_camera_stream'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='Ann-Kathrin Stracke',
    maintainer_email='ann-kathrin.stracke@stud.hs-mannheim.de',
    description=(
        'Reads the /image_raw/compressed topic and controls the turtlebot depending on the scanned images.'
    ), 
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_stream = turtlebot3_camera_stream.script.camera_stream:main'
        ],
    },
)