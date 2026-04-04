import sys, os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import yaml
from PIL import Image
from pprint import pprint

from pdf2image import convert_from_path


def run(args=None):
    # Convert the pdf to a series of images
    land_register_path = os.path.join(args['src'], 'config', 'location', 'land_register.pdf')
    images = convert_from_path('Land_at_the_Hainton_Estate___CA_7_1_244.pdf')
    #for i in range(len(images)):
    #    images[i].save('page'+str(i)+'.jpg', 'JPEG')

    # Load the image
    page_number = input('Select page number of interest [1-{len(images)}]: ')
    
    """
%{
I = imread('page2.jpg');

r=I(:,:,1);
g=I(:,:,2);
b=I(:,:,3);

bin = imbinarize(r-g);
ope = imopen(bin, strel('disk',2));
clo = imclose(ope, strel('disk',20));
fil = imfill(clo, 'holes');
ero = imerode(fil, strel('disk',5));

I(ero) = 0;
imagesc(I); axis image
%}

I = imread('page2.jpg');

r=I(:,:,1);
g=I(:,:,2);
b=I(:,:,3);
"""




def main(args=None):
    e = 'environment_template'
    src = '/'.join(get_package_prefix(e).split('/')[:-2]) + f'/src/{e}'
    location_name = 'riseholme_polytunnel'
    run({'src': src, 'location_name':location_name})

if __name__ == '__main__':
    main()
