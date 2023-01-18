# -*- coding: utf-8 -*-
"""
Created on Wed Sep 16

@author: ludwig
"""

# Python modules
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import scipy.optimize as opt
from PIL import Image

# Defining pathes for importing Homebuild modules
import sys
# sys.path.insert(1, "../basic-modules/")
sys.path.insert(1, "//192.168.1.2/groups/Atomics/ATOMICS-python-modules/basic-modules")
# sys.path.insert(1, "C:/Users\ludwi\OneDrive\Dokumente\Python Scripts\Modules")
# sys.path.insert(1, "//192.168.1.2/groups/Atomics/_Python-Scripte/Modules/")


# Homebuild modules
import graphical_analysis as ga
import camera_constants as cc
import display_data as dd
import atom_numbers as an

import os

# Graphical SetUps
mpl.rc("figure",figsize=(12,9))
mpl.rc("xtick", labelsize = 18)
mpl.rc("ytick", labelsize = 18)
mpl.rc("axes", labelsize = 20)

# German or English style for decimal numbers in plot
import locale
locale.setlocale(locale.LC_NUMERIC, 'deu_deu')
plt.rcParams['axes.formatter.use_locale'] = True


# =============================================================================
# Setting path
# =============================================================================
# Allows to switch the use from brain42 to local
working_on_brain42 = True

exp_code = "220601_Cooler_"
date, code = exp_code.split("_")[:-1]
year = "20"+date[:2]
cor_folder = "cor_"+exp_code[:-1]

path = ""
if working_on_brain42:
    path = "//brain42/groups/Atomics/Data/" +year+ "/" +date+ "/" +code+ "/" 
else:
    path = "C:/Users/Daniel/Desktop/ATOMICS/55796/" 

# =============================================================================
# Camera
# =============================================================================
camera = "top"
camera_atomnumber_constant = float("nan")
camera_px_to_m_constant = float("nan")
if camera == "top":
    camera_atomnumber_constant = cc.c102
    camera_px_to_m_constant = cc.c52
else:
    camera_atomnumber_constant = cc.c101
    camera_px_to_m_constant = cc.c51

# =============================================================================
# Read in data using exp_code
# =============================================================================
file = path + exp_code+ ".npy"
data = np.load(file)

data[np.isnan(data)] = 0
data[np.isinf(data)] = 0

(n_mp, n_s, n_y, n_x) = np.shape(data)

data = data*camera_atomnumber_constant

data_m = np.mean(data, axis = 1)#[:,57:-54,48:-47]

# =============================================================================
# Axes and measuring points
# =============================================================================
times = np.array([10,20,30,50,80])#*1e-3
tof_times = np.linspace(10, 30, 6)

x = np.linspace(0, (n_x-1), n_x) * camera_px_to_m_constant * 1e6
y = np.linspace(0, (n_y-1), n_y) * camera_px_to_m_constant * 1e6

# =============================================================================
# Full Data Plot
# =============================================================================
label = [f"({time:.0f}, {tof_time:.0f}) ms".replace(".", ",") for time in times for tof_time in tof_times]
# for j in range(len(time)):
#     label.append(str(round(time[j])) + ' ms')

label = None # [f"(20, {r_outer:.1f}) µm" for r_outer in np.linspace(50,35,7)]

# data_m = np.swapaxes(data_m, 1,2)
# data_m = np.flip(data_m, 1)
# data_m = np.flip(data_m, 0)
savepath = file[:-4]+"full_data"
dd.display_data(data=data_m, n_vert=1, n_hor=2, norm_bool=True, label = label, kk=camera_px_to_m_constant, savepath=savepath)


# [atom_number_m, atom_number_std] = an.gaussian_fit_2D(x,y,data, x0_initial = 600e-6, y0_initial = 700e-6)