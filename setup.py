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
            'metric_pgm.py = environment_common.convertors.metric_pgm:main',
            'tmap.py = environment_common.convertors.tmap:main',
            'satellite.py = environment_common.convertors.satellite:main',
            'wave_form_collapse.py = environment_common.procedural.wave_function_collapse:main',
        ],
    },
)
