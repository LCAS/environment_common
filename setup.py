from setuptools import setup
from glob import glob
import os

package_name = 'environment_common'
pkg = package_name
convert = f'{pkg}.convertors'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{pkg}']),
        (f'share/{pkg}', ['package.xml']),
        (f"share/{pkg}/config", glob(os.path.join('config', '*.rviz'))),
        (f"share/{pkg}/launch", glob(os.path.join('launch', '*launch.[pxy][yml]*'))),
        (f"share/{pkg}/launch", glob(os.path.join('launch', '*'))),
    ],
    zip_safe=True,
    maintainer='james',
    maintainer_email='primordia@live.com',
    description='Management package for packages based on LCAS/environment_template',
    license='Apache 2.0',
    tests_require=['pytest', 'geopy'],
    entry_points={
        'console_scripts': [
            f'kml_to_datum.py = {convert}.kml_to_datum:main',
            f'kml_to_tmap.py =  {convert}.kml_to_tmap:main',

            f'objects_to_gazebo.py = {convert}.objects_to_gazebo:main',

            f'osm_to_datum.py = {convert}.osm_to_datum:main',
            f'osm_to_kml.py =   {convert}.osm_to_kml:main',
            f'osm_to_tmap.py =  {convert}.osm_to_tmap:main',

            f'datum_to_kml.py =       {convert}.datum_to_kml:main',
            f'datum_to_satellite.py = {convert}.datum_to_satellite:main',
            f'datum_to_metric.py =    {convert}.datum_to_metric:main',

            f'tmap_to_kml.py = {convert}.tmap_to_kml:main',

            f'metric_to_transparent.py = {convert}.metric_to_transparent:main',
            f'metric_to_kml.py =         {convert}.metric_to_kml:main'
        ],
    },
)
