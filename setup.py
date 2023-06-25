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
#            'gazebo_world__TO__metric_pgm = generators/metric_pgm___FROM___gazebo_world:main',
#            'satellite_png__TO__network_tmap = generators/network_tmap___FROM___satellite_png:main',
#            'datum_yaml__TO__satellite_png = generators/satellite_png___FROM___datum_yaml:main',
#            'generator__USING__wave_form_collapse = procedural_generation_tools/wave_form_collapse:main',
        ],
    },
)
