# -*- coding: utf-8 -*-
"""
Created on Mon Jan 16 18:46:00 2023

@author: T. Preuschoff/ L. Lind @APQ @IAP @TUDa
"""

from PIL import Image, ImageDraw

width = 1024
height = 768
center_x = 331 #width // 2
center_y = 356#height // 2
color_2 = (0, 0, 0)
color_1 = (255, 255, 255)
rad = 10

im = Image.new('RGB', (width, height), color_1)
draw = ImageDraw.Draw(im)
draw.ellipse((center_x - rad, center_y - rad, center_x + rad, center_y + rad), fill=color_2)

im.save('img/blende_1_%r.bmp'%rad)
