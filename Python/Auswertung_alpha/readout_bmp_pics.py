# -*- coding: utf-8 -*-
"""
Created on Wed Sep 16 10:37:35 2020
@authors: Ludwig, Daniel
"""

import numpy as np
import os
from PIL import Image

import matplotlib.pyplot as plt

# Allows to switch the use from brain42 to local
working_on_brain42 = False

exp_code = "220930_Lightsheet_"
date, code = exp_code.split("_")[:-1]
year = "20"+date[:2]
cor_folder = "cor_"+exp_code[:-1]

path = ""
if working_on_brain42:
    path = "//brain43/groups/Atomics/Data/" +year+ "/" +date+ "/" +code+ "/" 
else:
    path = "C:/Users/ludwi/OneDrive/Dokumente/WiMi/Projekte/Lightsheet/Messungen/"
    # C:\Users\ludwi\OneDrive\Dokumente\WiMi\Projekte\Lightsheet\Messungen

n_measuring_points = 1
n_shots_per_point  = 1
n_shots_overall = 1
SeqRun_bool = True

target_shape = () #(5,8,7,220,220)

list_of_waste = []

# list_of_waste = [i for i in range(48,96)]
# good = [i for i in range(0, 33)]
# print(good)

# for i in good:
#     list_of_waste.remove(i)

# list_of_waste = [i for i in range(75, 685)]
# list_of_waste.extend([i for i in range(0, 66)])
# list_of_waste.extend([69, 70])

# list_of_waste = [i for i in range(140, 685)]
# list_of_waste = [5,6,7,8,9,55,68,69,70,71,115,116,124,123,122]
# list_of_waste.extend([197,196,195,198,199])
# list_of_waste.extend([270,271,272,273,274,325,326,327,328,329])
# list_of_waste.extend([480,481,482,483,484])
# list_of_waste.extend([540,541,542,543,544,580,581,582,583,584])
# list_of_waste.extend([i for i in range(595,610)])

list_data  = []
for j in range(n_shots_overall):
    if j not in list_of_waste:
        # filename = path + "pos" + str(j+1) +".bmp"
        filename = path + "ohne_BE_Ende_Board.bmp"
        img = Image.open(filename)
        shot = np.array(img)
        list_data.append(shot)

(n_y, n_x) = np.shape(shot)
data = np.array(list_data)

if target_shape == ():
    if SeqRun_bool:
        data = np.reshape(data, (n_shots_per_point, n_measuring_points, n_y, n_x))
        data = np.swapaxes(data, 0, 1)
    else:
        data = np.reshape(data, (n_measuring_points, n_shots_per_point, n_y, n_x))

else:
    data = np.reshape(data, target_shape)

print(np.shape(data))

# Save to subfolder data
if not os.path.exists(path):
    os.makedirs(path)
np.save(os.path.join(path, exp_code+""), data)

import matplotlib.pyplot as plt
# ref = np.load("//brain42/groups/Atomics/Data/2021/211221/60131/cor_211221_60131/211221_60131_250_Hz.npy")
# np.sum(data[0][i]-ref[16][i])
