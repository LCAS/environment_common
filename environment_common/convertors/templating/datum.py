
class DatumTemplates:

    template = """# Define site datum - best to be in the center of the field of interest
datum_latitude: &datum_latitude %s
datum_longitude: &datum_longitude %s


# Define GNSS Fence coordinates [lat, lon]
gnss_fence:
  gnss_fence_coords: %s

# Define map size (x east, y north) in meters from the datum
gmapping:
  xmin: %s
  xmax: %s
  ymin: %s
  ymax: %s

mapviz_initialize_origin:
  local_xy_origins:
    - {
    name: dont_change_this,
    latitude: *datum_latitude,
    longitude: *datum_longitude,
    altitude: 100.0,
    heading: 0.0
    }

navsat_transform_node:
  magnetic_declination_radians: 0.0
  yaw_offset: 0.0
  zero_altitude: true
  broadcast_utm_transform: false
  broadcast_utm_transform_as_parent_frame: false
  publish_filtered_gps: true
  use_odometry_yaw: false
  wait_for_datum: true
  datum: [ *datum_latitude, *datum_longitude, 0.0]

"""
