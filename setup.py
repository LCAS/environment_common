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
        'console_scripts': [
            'kml_to_datum.py = environment_common.convertors.kml_to_datum:main',
            'kml_to_tmap.py = environment_common.convertors.kml_to_tmap:main',
            'tmap_to_kml.py = environment_common.convertors.tmap_to_kml:main'
        ],
    },
)
