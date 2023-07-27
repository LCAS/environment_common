from PIL import Image
import requests
from io import BytesIO

from environment_common.convertors.tools.gps import get_bounds, get_range


#zoom = 16.31
#rotation = 270
#url = f"https://api.mapbox.com/styles/v1/mapbox/satellite-v9/static/{lon},{lat},{zoom},{rotation}/{size}@2x?access_token={TOKEN}"


def get_image(fence, TOKEN):
    bounds = get_bounds(fence)
    bounds_str = f"[{bounds['west']},{bounds['south']},{bounds['east']},{bounds['north']}]"

    x, y = get_range(bounds)
    scale = 1280/max([abs(x), abs(y)])
    X = int(abs(x)*scale)
    Y = int(abs(y)*scale)
    print(' distances', abs(x), abs(y))
    print('   scaller', scale)
    print('dimensions', X, Y)
    size = f"{Y}x{X}"

    url = f"https://api.mapbox.com/styles/v1/mapbox/satellite-v9/static/{bounds_str}/{size}@2x?access_token={TOKEN}"
    print('\nurl', url)
    response = requests.get(url)
    if response.content != b'{"message":"Not Found"}':
        im = Image.open(BytesIO(response.content))
        return im
    print('\nresponse.content', response.content)
    exit()

