# -*- coding: utf-8 -*-
"""
Created on Mon Jan 16 18:48:00 2023

@author: T. Preuschoff/ L. Lind @APQ @IAP @TUDa
"""

from PIL import Image, ImageDraw

images = []

width = 1024
height = 768
center_x = 488 #width // 2
center_y = 356#height // 2
color_1 = (0, 0, 0)
color_2 = (255, 255, 255)
max_radius = height // 2
step = 3

for i in range(0, max_radius, step):
    im = Image.new('RGB', (width, height), color_1)
    draw = ImageDraw.Draw(im)
    draw.ellipse((center_x - i, center_y - i, center_x + i, center_y + i), fill=color_2)
    images.append(im)


images[0].save('img/aperture.gif',
               save_all=True, append_images=images[1:], optimize=False, duration=1, loop=0)