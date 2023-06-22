from setuptools import setup

package_name = 'environment_common'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[''],
    zip_safe=True,
    maintainer='james',
    maintainer_email='primordia@live.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gazebo_world__TO__metric_pgm = generators/metric_pgm___FROM___gazebo_world:main',
            'satellite_png__TO__network_tmap = generators/network_tmap___FROM___satellite_png:main',
            'datum_yaml__TO__satellite_png = generators/satellite_png___FROM___datum_yaml:main',
            'generator__USING__wave_form_colappse = procedural_generation_tools/wave_form_collapse:main'
        ],
    },
)
