from PIL import Image
import requests
from io import BytesIO

from environment_common.convertors.tools.gps import get_datumrelative_metric_from_gps
from pprint import pprint

#zoom = 16.31
#rotation = 270
#url = f"https://api.mapbox.com/styles/v1/mapbox/satellite-v9/static/{lon},{lat},{zoom},{rotation}/{size}@2x?access_token={TOKEN}"

def get_image(fence, TOKEN):
    lats = [l[0] for l in fence]
    lons = [l[1] for l in fence]

    north, south = max(lats), min(lats)
    east, west = max(lons), min(lons)
    bounds = f"[{west},{south},{east},{north}]"

    ne = {'latitude':north, 'longitude':east, 'elevation':0}
    sw = {'latitude':south, 'longitude':west, 'elevation':0}
    print(ne, sw)
    xyz = get_datumrelative_metric_from_gps(sw, ne)
    pprint(xyz)
    size = f"{abs(int(xyz['x']))}x{abs(int(xyz['y']))}"

    url = f"https://api.mapbox.com/styles/v1/mapbox/satellite-v9/static/{bounds}/{size}@2x?access_token={TOKEN}"
    print(url)
    response = requests.get(url)
    im = Image.open(BytesIO(response.content))
    return im



