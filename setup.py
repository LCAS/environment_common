from setuptools import setup

package_name = 'environment_common'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    zip_safe=True,
    maintainer='james',
    maintainer_email='primordia@live.com',
    description='Management package for packages based on LCAS/environment_template',
    tests_require=['pytest'],
    entry_points={
    entry_points={
        'console_scripts': [
            'make_datum.py = environment_common.convertors.make_datum:main',
            'make_klm.py = environment_common.convertors.make_klm:main',
            'get_satellite.py = environment_common.convertors.get_satellite:main'
        ],
    },
)
