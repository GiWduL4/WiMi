# -*- coding: utf-8 -*-
"""
Created on Mon Jan 16 18:52:42 2023

@author: T. Preuschoff/ L. Lind @APQ @IAP @TUDa
"""

from PIL import Image, ImageDraw

width = 1024
height = 768
center_x = 331 #width // 2
center_y = 356#height // 2
color_2 = (0, 0, 0)
color_1 = (255, 255, 255)

im = Image.new('RGB', (width, height), color_1)
draw = ImageDraw.Draw(im)

im.save('img/white.bmp')