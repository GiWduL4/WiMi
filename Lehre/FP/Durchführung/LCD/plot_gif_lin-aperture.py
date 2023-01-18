# -*- coding: utf-8 -*-
"""
Created on Mon Jan 16 18:49:43 2023

@author: T. Preuschoff/ L. Lind @APQ @IAP @TUDa
"""

from PIL import Image, ImageDraw

images = []

width = 1024
height = 768
color_1 = (0, 0, 0)
color_2 = (255, 255, 255)
step = 1

"""
horizontal
"""

for i in range(0, width, step):
    im = Image.new('RGB', (width, height), color_1)
    draw = ImageDraw.Draw(im)
    draw.rectangle((0,0,i*step,height-1), fill=color_2)
    images.append(im)

images[0].save('aperture_x.gif', format = "GIF",
               save_all=True, append_images=images[1:], optimize=False, duration=1, loop=0)

"""
vertical
"""

for i in range(0, height, step):
    im = Image.new('RGB', (width, height), color_1)
    draw = ImageDraw.Draw(im)
    draw.rectangle((0,0,width-1,i*step), fill=color_2)
    images.append(im)

images[0].save('aperture_y.gif',
               save_all=True, append_images=images[1:], optimize=False, duration=1, loop=0)