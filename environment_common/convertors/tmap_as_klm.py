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


e = 'environment_template'
src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
location_name = 'riseholme_polytunnel'
args = {'src': src, 'location_name':location_name, 'line_col':'ff2f2fd3', 'line_width':'6', 'fill_col':'c02f2fd3'}

def main(args=None):
    pass
if True:
    line_col = args['line_col']
    line_width = args['line_width']
    fill_col = args['fill_col']

    datum_path = os.path.join(args['src'], 'config', 'location', 'datum.yaml')
    with open(datum_path) as f:
        data = f.read()
        datum = yaml.safe_load(data)

    place_id = args['location_name']
    lat = datum['datum_latitude']
    lon = datum['datum_longitude']

    tmap_path = os.path.join(args['src'], 'config', 'topological', 'network.tmap2')
    with open(tmap_path) as f:
        data = f.read()
        tmap = yaml.safe_load(data)

    node_pose_list = dict()
    for n in tmap['nodes']:
        node_pose_list[n['node']['name']] = n['node']['pose']['position']

    edge_list = set()
    for n in tmap['nodes']:
        for e in n['node']['edges']:
            l = [e['node'],n['node']['name']]
            l.sort()
            edge_list.add(f"{l[0]}_{l[1]}")

    klm = template_start % f"{place_id}_tmap_klm_autogen"
    klm += template_style % ("a1",  line_col, line_width, fill_col)
    klm += template_style % ("a2", line_col, line_width, fill_col)
    klm += template_style_join % ("a", "a1", "a2")

    from math import cos
    def add_to_gps(latitude, longitude, node_pose_list, node):
        x_offset = node_pose_list[node]['x'] / (cos(latitude) * 111111)
        y_offset = node_pose_list[node]['y'] / (111111)
        return latitude + y_offset, longitude - (x_offset)

    for i,e in enumerate(edge_list):
        s, t = e.split('_')

        # Get latlon corrections
        lat_s, lon_s = add_to_gps(lat, lon, node_pose_list, s)
        lat_t, lon_t = add_to_gps(lat, lon, node_pose_list, t)

        fence = f"{lon_s},{lat_s},0 {lon_t},{lat_t},0"
        klm += template_placemark % ("a0", e, lon_s, lat_s, 0, "a", fence)

    fence = f"{lon},{lat-0.00001},0 {lon},{lat},0"
    klm += template_placemark % ("a0", "center", lon, lat, 0, "a", fence)
    klm += template_end

    klm_path = os.path.join(args['src'], 'config', 'topological', 'tmap2.klm')
    with open(klm_path, 'w') as f:
        f.write(klm)


if __name__ == '__main__':
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    location_name = 'riseholme_polytunnel'
    main({'src': src, 'location_name':location_name, 'line_col':'40ffffff', 'line_width':'24', 'fill_col':'40ffffff'})

