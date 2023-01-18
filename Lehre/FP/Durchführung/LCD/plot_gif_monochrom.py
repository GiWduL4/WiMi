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
dur = 1

im = Image.new('RGB', (width, height), color_1)
draw = ImageDraw.Draw(im)
images.append(im)

im = Image.new('RGB', (width, height), color_2)
draw = ImageDraw.Draw(im)
images.append(im)

images[0].save('img/monochrom%d.gif'%(dur), format = "GIF",
               save_all=True, append_images=images[1:], optimize=False, duration=dur, loop=0)

