# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import os, subprocess
import ee

def main(args=None):
    pass
if True:
    # Initialize credentials
    cred_id = os.getenv('GCLOUD_SERVICE_ID')
    cred_key_file = os.getenv('GCLOUD_SERVICE_KEY')
    credentials = ee.ServiceAccountCredentials(cred_id, cred_key_file)
    ee.Initialize(credentials)

    # Specify GPS location
    u_lat = -70.892
    #53.26851890186007
    u_lon = 41.6555
    #-0.5240515126879408
    u_poi = ee.Geometry.Point(u_lon, u_lat)
    roi = u_poi.buffer(1e6)

    # Specify search meta
    i_date = '2014-01-01'
    f_date = '2016-01-01'

    #https://developers.google.com/earth-engine/tutorials/community/intro-to-python-api

    # Specify image dataset
    imgc = ee.ImageCollection('SKYSAT/GEN-A/PUBLIC/ORTHO/RGB')
    #https://developers.google.com/earth-engine/datasets/catalog/SKYSAT_GEN-A_PUBLIC_ORTHO_RGB


    # Filter the collection to our requirements
    #imgc = imgc.filterDate(i_date, f_date)
    r = imgc.select('R')
    g = imgc.select('G')
    b = imgc.select('B')
    img = r.first()

    #imgc = ee.ImageCollection('MODIS/006/MOD11A1')
    #lst = imgc.select('LST_Day_1km', 'QC_Day').filterDate(i_date, f_date)
    #lst_img = lst.mean()
    #lst_img = lst_img.select('LST_Day_1km').multiply(0.02)
    #lst_img = lst_img.select('LST_Day_1km').add(-273.15)
    #img = lst_img

    # Generate URL link
    url = img.getThumbUrl({'min': 11, 'max': 190, 'dimensions': 512, 'region': roi})
    print(url)


    # Download file from Link
    #subprocess.run(f'wget {link}')


if __name__ == '__main__':
    main(args={})

