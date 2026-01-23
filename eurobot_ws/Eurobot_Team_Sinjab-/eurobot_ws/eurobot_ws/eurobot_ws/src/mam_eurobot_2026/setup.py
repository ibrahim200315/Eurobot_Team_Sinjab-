from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'mam_eurobot_2026'

def files_only(pattern):
    return [p for p in glob(pattern, recursive=True) if os.path.isfile(p)]

launch_files = files_only('launch/*.launch.py')
world_files  = files_only('worlds/*')

# Keep folder structure by installing each model directory to its own target
crate_files         = files_only('models/crate/**/*')
crate_yellow_files  = files_only('models/crate_yellow/**/*')
mat_files           = files_only('models/mat/**/*')
simple_robot_files  = files_only('models/simple_robot/**/*')

data_files = [
    # ament index marker (make sure resource/mam_eurobot_2026 exists)
    ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
    # package manifest
    (f'share/{package_name}', ['package.xml']),
    # launch & worlds
    (f'share/{package_name}/launch',  launch_files),
    (f'share/{package_name}/worlds',  world_files),
    # models — each gets its own directory, preserving relative paths
    (f'share/{package_name}/models/crate',          crate_files),
    (f'share/{package_name}/models/crate_yellow',   crate_yellow_files),
    (f'share/{package_name}/models/mat',            mat_files),
    (f'share/{package_name}/models/simple_robot',   simple_robot_files),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ibrahim',
    maintainer_email='ibrahim@todo.todo',
    description='Eurobot 2026 package (launch/worlds/models)',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
    'crate_perception = mam_eurobot_2026.crate_perception:main',
    'lidar_debug = mam_eurobot_2026.lidar_debug:main',
    
    ]},
)

