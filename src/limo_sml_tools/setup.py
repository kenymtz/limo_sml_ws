from glob import glob

from setuptools import find_packages, setup

package_name = 'limo_sml_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py')
        ),
    ],
    install_requires=[
        'setuptools',
        'pyyaml',
    ],
    zip_safe=True,
    maintainer='limopro',
    maintainer_email='skuarex@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_snapshot = limo_sml_tools.nodes.robot_snapshot:main',
        ],
    },
)
