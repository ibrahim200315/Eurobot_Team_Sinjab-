from setuptools import setup, find_packages

package_name = 'mam_eurobot_2026'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Freddy Liendo',
    maintainer_email='liendomf@univ-smb.fr',
    description='Template package for Eurobot 2026',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'py_test = mam_eurobot_2026.py_test:main',
            'crate_identify = mam_eurobot_2026.crate_identify:main',
        ],
    },
)

