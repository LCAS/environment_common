#chatGPT-3.5's attempt
from math import radians, sin, cos, sqrt, atan2, degrees

def calculate_displacement(lat1, lon1, lat2, lon2):
    # Convert coordinates to radians
    lat1, lon1 = radians(lat1), radians(lon1)
    lat2, lon2 = radians(lat2), radians(lon2)

    # Earth radius in meters
    earth_radius = 6371000

    # Haversine formula
    delta_lat = lat2 - lat1
    delta_lon = lon2 - lon1

    a = sin(delta_lat/2)**2 + cos(lat1) * cos(lat2) * sin(delta_lon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))

    # Calculate displacement in meters
    displacement_lat = earth_radius * c
    displacement_lon = earth_radius * c * cos(lat1)

    return displacement_lat, displacement_lon



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

    return new_lat, new_lon




#from math import cos
#
#
#def add_to_gps(latitude, longitude, node_pose_list, node):
#    x_offset = (node_pose_list[node]['x']) / (cos(latitude) * 111111)
#    y_offset = (node_pose_list[node]['y']) / (111111)
#    return latitude + (y_offset*0.95), longitude + (-x_offset*1.65)
#
#
#def gps_to_metric(datum_latlon, latitude, longitude, elevation):
#    x = (datum_latlon['latitude'] - latitude) / (cos(latitude) * 111111)
#    y = (datum_latlon['longitude'] - longitude) / (111111)
#    z = elevation
#    return x, y, z



