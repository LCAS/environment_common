#chatGPT-3.5's attempt
from math import radians, sin, cos, sqrt, atan2, degrees

def calculate_displacement(lat1in, lon1in, lat2in, lon2in):
    # Convert coordinates to radians
    lat1, lon1 = radians(lat1in), radians(lon1in)
    lat2, lon2 = radians(lat2in), radians(lon2in)

    # Earth radius in meters
    earth_radius = 6371000

    # Haversine formula
    delta_lat = lat2 - lat1
    delta_lon = lon2 - lon1

    a = sin(delta_lat/2)**2 + cos(lat1) * cos(lat2) * sin(delta_lon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))

    # Calculate displacement in meters
    displacement_y = earth_radius * c
    displacement_x = earth_radius * c * cos(lat1)

    #displacement_y = displacement_y if lat1in < lat2in else -displacement_y
    #displacement_x = displacement_x if lon1in < lon2in else -displacement_x

    print(f'Lat: {round(lat1in,5)},{round(lat2in,5)},{round(displacement_y,1)} \t| Lon: {round(lon1in,5)},{round(lon2in,5)},{round(displacement_x,1)}')
    return displacement_y, displacement_x



def calculate_distance_changes(lat1, lon1, lat2, lon2):
    # Convert coordinates to radians
    lat1 = radians(lat1)
    lon1 = radians(lon1)
    lat2 = radians(lat2)
    lon2 = radians(lon2)

    # Earth radius in meters
    earth_radius = 111111 #6371000  # meters

    # Haversine formula
    delta_lat = lat2 - lat1
    delta_lon = lon2 - lon1

    # Calculate distance changes in latitude and longitude
    distance_change_lat = degrees(delta_lat) * earth_radius
    distance_change_lon = degrees(delta_lon) * earth_radius * cos(lat1)

    return round(distance_change_lat,2), round(distance_change_lon,2)



def calculate_coordinates(lat, lon, dx, dy):
    # Earth radius in meters
    earth_radius = 6371000

    # Convert latitude and longitude to radians
    lat = radians(lat)
    lon = radians(lon)

    # Calculate displacement in radians
    displacement_lat = dx / earth_radius
    displacement_lon = dy / (earth_radius * cos(lat))

    # Calculate new latitude and longitude
    new_lat = lat + displacement_lat
    new_lon = lon + displacement_lon

    # Convert back to degrees
    new_lat = degrees(new_lat)
    new_lon = degrees(new_lon)

    # Return gps corrected by metric input
    return new_lat, new_lon



#def add_to_gps(latitude, longitude, node_pose_list, node):
#    x_offset = (node_pose_list[node]['x']) / (cos(latitude) * 111111)
#    y_offset = (node_pose_list[node]['y']) / (111111)
#    return latitude + (y_offset*0.95), longitude + (-x_offset*1.65)












#### IDEAL SETUP
def get_datumrelative_metric_from_gps(datum, gnss):
    x = (datum['latitude'] - gnss['latitude']) / (111111)
    y = (datum['longitude'] - gnss['longitude']) / (cos(datum['latitude']) * 111111)
    z = datum['elevation'] - gnss['elevation']
    return {'x':x, 'y':y, 'z':z}

def get_gps_from_datumrelative_metric(datum, xyz):
    lat = degrees( radians(datum['latitude']) + ( xyz['x'] / 6371000 ))
    lon = degrees( radians(datum['longitude']) + ( xyz['y'] / (6371000 * cos(radians(datum['latitude']))) ))
    return {'latitude': lat, 'longitude': lon, 'elevation': datum['elevation']}

def displace_gps_by_metric_relative_to_datum(datum, gnss, xyz):
    #print('displace_gps_by_metric_relative_to_datum')
    #print(datum)
    #print(gnss)
    #print(xyz)
    #print('\n')

    metric = get_datumrelative_metric_from_gps(datum, gnss)
    #print(metric)
    #print('\n')

    new_xyz = {'x':metric['x']+xyz['x'], 'y':metric['y']+xyz['y'], 'z':metric['z']+xyz['z']}
    #print(new_xyz)
    #print('\n')

    new_gnss = get_gps_from_datumrelative_metric(datum, new_xyz)
    #print(new_gnss)

    return new_gnss












#def displace_metric_by_gps_relative_to_datum(datum, xyz, gnss):
#    return new_xyz




def metric(datum_latitude, datum_longitude, datum_elevation, latitude, longitude, elevation):
    x = (datum_latitude - latitude) / (111111)
    y = (datum_longitude - longitude) / (cos(latitude) * 111111)
    z = datum_elevation - elevation
    return x, y, z

def gps(datum_latitude, datum_longitude, datum_elevation, x, y, z):
    latitude = datum_latitude + (x * 111111)
    longitude = datum_longitude + (y * cos(latitude) * 111111)
    elevation = datum_elevation + z
    return latitude, longitude, elevation

def add_metric_to_gps(datum_latitude, datum_longitude, datum_elevation, lat, lon, ele, x, y, z):
     xLat, yLon, zEle = metric(datum_latitude, datum_longitude, datum_elevation, lat, lon, ele)
     return gps(datum_latitude, datum_longitude, datum_elevation, xLat+x, yLon+y, zEle+z)




#datum: lat lon
#node: lat, lon
#tmap = metric(node) - metric(datum)
#tmap = metric(node-datum)
def get_relative_metric(datum, node):
    return metric(node)-metric(datum)




#datum: lat lon
#tmap: xd, yd
#node = gps(metric(datum)+tmap)
#node = datum+gps(tmap)
def get_relative_metric(datum, tmap):
    return datum+metric(node)



