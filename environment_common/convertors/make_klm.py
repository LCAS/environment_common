import sys, os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import yaml
from pprint import pprint


template_start = """<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:gx="http://www.google.com/kml/ext/2.2" xmlns:kml="http://www.opengis.net/kml/2.2" xmlns:atom="http://www.w3.org/2005/Atom">
<Document>
	<name>%s</name>"""

template_style = """
	<gx:CascadingStyle kml:id="__managed_style_%s">
		<Style>
			<LineStyle>
				<color>%s</color>
				<width>%s</width>
			</LineStyle>
			<PolyStyle>
				<color>%s</color>
			</PolyStyle>
		</Style>
	</gx:CascadingStyle>"""


template_style_join = """
	<StyleMap id="__managed_style_%s">
		<Pair>
			<key>normal</key>
			<styleUrl>#__managed_style_%s</styleUrl>
		</Pair>
		<Pair>
			<key>highlight</key>
			<styleUrl>#__managed_style_%s</styleUrl>
		</Pair>
	</StyleMap>"""


template_placemark = """
	<Placemark id="%s">
		<name>%s</name>
		<LookAt>
			<longitude>%s</longitude>
			<latitude>%s</latitude>
			<altitude>%s</altitude>
			<heading>0</heading>
			<tilt>0</tilt>
			<gx:fovy>35</gx:fovy>
			<range>100</range>
			<altitudeMode>absolute</altitudeMode>
		</LookAt>
		<styleUrl>#__managed_style_%s</styleUrl>
		<Polygon>
			<outerBoundaryIs>
				<LinearRing>
					<coordinates>
                        %s
					</coordinates>
				</LinearRing>
			</outerBoundaryIs>
		</Polygon>
	</Placemark>"""


template_end = """
</Document>
</kml>"""



def main(args=None):
    datum_path = os.path.join(args['src'], 'config', 'location', 'datum.yaml')
    with open(datum_path) as f:
        data = f.read()
        datum = yaml.safe_load(data)

    place_id = args['location_name']
    lat = datum['datum_latitude']
    lon = datum['datum_longitude']
    alt = 0
    fence = ' '.join([f'{g[0]},{g[1]},{alt}' for g in datum['gnss_fence']['gnss_fence_coords']])

    line_col = args['line_col']
    line_width = args['line_width']
    fill_col = args['fill_col']

    klm = template_start % f"{place_id}_klm_autogen"
    klm += template_style % ("a1",  line_col, line_width, fill_col)
    klm += template_style % ("a2", line_col, line_width, fill_col)
    klm += template_style_join % ("a", "a1", "a2")
    klm += template_placemark % ("a0", place_id, lon, lat, alt, "a", fence)
    klm += template_end

    klm_path = os.path.join(args['src'], 'config', 'location', 'region.klm')
    with open(klm_path, 'w') as f:
        f.write(klm)


if __name__ == '__main__':
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    location_name = 'riseholme_polytunnel'
    main({'src': src, 'location_name':location_name, 'line_col':'ff2f2fd3', 'line_width':'24', 'fill_col':'c02f2fd3'})

