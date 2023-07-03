import sys, os
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import xml.etree.ElementTree as ET
from pprint import pprint


class KmlTemplates:
    opening = """<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:gx="http://www.google.com/kml/ext/2.2" xmlns:kml="http://www.opengis.net/kml/2.2" xmlns:atom="http://www.w3.org/2005/Atom">
<Document>
    <name>%s</name>"""

    closing = """
</Document>
</kml>"""

    placemark = """
    <Placemark id="%s">
        <name>%s</name>
        <description><![CDATA[<div>%s</div>]]></description>
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
    eg = '-0.5265231,53.2769151,39.0252056 -0.5254612,53.2684,37.95586'


    style = """
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


    style_join = """
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


    @classmethod
    def styler(cls, id, line_col, line_width, fill_col):
        style = cls.style % (id, line_col, line_width, fill_col)
        style_join = cls.style_join % (id+"_map", id, id)
        return style + style_join

class KmlDraw:

    @classmethod
    def coord_list_list_to_polyline_str(cls, coords):
        polyline_str = ' '.join([f"{lo},{la},{el}" for lo,la,el in coords+[coords[0]]])
        edging = '\n\t\t\t\t%s \n\t\t\t'
        return edging%polyline_str

    @classmethod
    def draw_point(cls, gnss, shape='diamond', style='a', size=0.00005):
        lo, la, el = gnss['longitude'], gnss['latitude'], gnss['elevation']
        name, sz = gnss['name'], size
        if shape == 'cross': pass
        elif shape == 'plus': pass
        elif shape == 'dot': pass
        elif shape == 'square': pass
        elif shape == 'diamond':
            coords = [[lo+sz,la,el], [lo,la+sz,el], [lo-sz,la,el], [lo,la-sz,el]]
            polyline = cls.coord_list_list_to_polyline_str(coords)
        return KmlTemplates.placemark % (name, name, name, lo, la, el, style, polyline)

    @classmethod
    def draw_nodes(cls, gnss_dict_list, shape, style, size):
        placemarks = ''.join([cls.draw_point(gnss, shape, style, size) for gnss in gnss_dict_list])
        return placemarks


    @classmethod
    def draw_line(cls, gnss1, gnss2, style='a'):
        lo1, la1, el1, name1 = gnss1['longitude'], gnss1['latitude'], gnss1['elevation'], gnss1['name']
        lo2, la2, el2, name2 = gnss2['longitude'], gnss2['latitude'], gnss2['elevation'], gnss2['name']
        lo, la, el, name = (lo1+lo2)/2, (la1+la2)/2, (el1+el2)/2, f"{name1}_{name2}"

        coords = [[lo1, la1, el1], [lo2, la2, el2]]
        polyline = cls.coord_list_list_to_polyline_str(coords)
        return KmlTemplates.placemark % (name, name, name, lo, la, el, style, polyline)

    @classmethod
    def draw_edges(cls, gnss_dict_list, style):
        gdd = {gnss['name']:gnss for gnss in gnss_dict_list}
        edge_list = sum([[tuple(sorted([n['name'],e])) for e in n['connections']] for n in gnss_dict_list],[])
        return ''.join([cls.draw_line(gdd[edge[0]], gdd[edge[1]], style) for edge in edge_list])


#now we need to make this identify any potential connections?
#we label each node first, then we label all their connections
#then when we mark nodes to clear, we add their connections to the keep one

class KmlRead:

    @classmethod
    def polyline_to_dictlist(cls, polyline_str, name, tagtype, circuit=False):
        pls = polyline_str.replace('\n','').replace('\t','').split(' ')[:-1]
        coords = [g.split(',') for g in pls]
        dictlist = [{'longitude':round(float(gnss[0]),6),
                     'latitude': round(float(gnss[1]),6),
                     'elevation':round(float(gnss[2]),6),
                     'raw_name': f"{name} {i}",
                     'raw_connections': []} for i,gnss in enumerate(coords)]

        if tagtype in ['LineString','Polygon']:
            for i in range(0,len(dictlist)-1):
                dictlist[i]['raw_connections'] += [dictlist[i+1]['raw_name']]
            for i in range(1,len(dictlist)):
                dictlist[i]['raw_connections'] += [dictlist[i-1]['raw_name']]

        if tagtype == 'Polygon':
            dictlist[0]['raw_connections'] += [dictlist[-1]['raw_name']]
            dictlist[-1]['raw_connections'] += [dictlist[0]['raw_name']]

        return dictlist


    @classmethod
    def get_coords(cls, root):
        details = dict()
        for i, base in enumerate(root[0]):
            if 'Placemark' in base.tag:
                name, coords = '', ''
                tags = {field.tag.split('}')[-1]:field for field in base}
                name = tags['name'].text
                if 'LineString' in tags:
                    coords = tags['LineString'][0].text
                    tagtype = 'LineString'
                elif 'Polygon' in tags:
                    coords = tags['Polygon'][0][0][0].text
                    tagtype = 'Polygon'
                elif 'Point' in tags:
                    coords = tags['Point'][0].text
                    tagtype = 'Point'
                if name in details:
                    name += len(details)
                details[name] = cls.polyline_to_dictlist(coords, name, tagtype)
        return details
