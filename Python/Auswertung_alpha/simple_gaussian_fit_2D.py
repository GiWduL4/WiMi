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

exp_code = "220817_52922_"
date, code = exp_code.split("_")[:-1]
year = "20"+date[:2]
cor_folder = "cor_"+exp_code[:-1]

path = ""
if working_on_brain42:
    path = "//brain43/groups/Atomics/Data/" +year+ "/" +date+ "/" +code+ "/" 
else:
    path = "C:/Users/Daniel/Desktop/ATOMICS/55796/" 

# =============================================================================
# Camera
# =============================================================================

camera_px_to_m_constant = 1e-6

# =============================================================================
# Read in data using exp_code
# =============================================================================
file = path + exp_code+ ".npy"
data = np.load(file)

data[np.isnan(data)] = 0
data[np.isinf(data)] = 0

(n_mp, n_s, n_y, n_x) = np.shape(data)

data_m = np.mean(data, axis = 1)#[:,57:-54,48:-47]

# =============================================================================
# Axes and measuring points
# =============================================================================
times = np.array([10,20,30,50,80])#*1e-3
tof_times = np.linspace(10, 30, 6)

x = np.linspace(0, (n_x-1), n_x) * camera_px_to_m_constant 
y = np.linspace(0, (n_y-1), n_y) * camera_px_to_m_constant 

# =============================================================================
# Full Data Plot
# =============================================================================
# for j in range(len(time)):
#     label.append(str(round(time[j])) + ' ms')

label = None # [f"(20, {r_outer:.1f}) µm" for r_outer in np.linspace(50,35,7)]

# data_m = np.swapaxes(data_m, 1,2)
# data_m = np.flip(data_m, 1)
# data_m = np.flip(data_m, 0)
savepath = None
# dd.display_data(data=data_m, n_vert=1, n_hor=2, norm_bool=True, label = label, kk=camera_px_to_m_constant, savepath=savepath)


plot_bool = True
colorbar_bool  = False
force_initial  = False

def gauss2D(data, ampl, offset, x0, y0, wx, wy):
    (x, y) = data
    A = ampl * np.exp(-(x-x0)**2/(2*wx**2)) * np.exp(-(y-y0)**2/(2*wy**2)) + offset
    return(A.ravel())

    
dx = x[1]-x[0]
dy = y[1]-y[0]

n_atoms_m = []
n_atoms_std = []

wx_m = []
wx_std = []

wy_m = []
wy_std = []

offset_m = []
offset_std = []

x0_m = []
x0_std = []

y0_m = []
y0_std = []

    # Iterate over measure points
for i in range(n_mp):
    print(n_mp)
    n_atoms_run = []
    offset_run = []
    wx_run = []
    wy_run = []
    x0_run = []
    y0_run = []
    
    # Iterate over shots per measure point
    for j in range(n_s):
        if plot_bool:
            ga.reel_2D(x*1e6,y*1e6,data[i,j], vmin = np.min(data[i,j]), vmax = np.max(data[i,j]), colorbar_bool = colorbar_bool, axratio = 240/368)
        
        if not force_initial:
            # "Guess" good x- and y-start position for fit via summation and finding the maximum
            x0_initial = x[np.argmax(np.sum(data[i,j], axis=0))]
            y0_initial = y[np.argmax(np.sum(data[i,j], axis=1))]
        
        popt, perr = ga.fit_2D(gauss2D, x*1e6, y*1e6, data[i,j], [np.max(data[i,j]), 0, x0_initial*1e6, y0_initial*1e6, 100, 100], plot_bool=plot_bool, colorbar_bool=colorbar_bool)
        # print(popt)
        
        N_atoms = 2 * np.pi * popt[0] * np.abs(popt[4] * popt[5])
        #print(N_atoms)
        n_atoms_run.append(N_atoms/(dx*dy))
        wx_run.append(np.abs(popt[4]))
        wy_run.append(np.abs(popt[5]))
        offset_run.append(popt[1]/popt[0])
        x0_run.append(popt[2])
        y0_run.append(popt[3])
        
    n_atoms_m.append(np.mean(n_atoms_run))
    n_atoms_std.append(np.std(n_atoms_run, ddof=1))
    
    wx_m.append(np.mean(wx_run))
    wx_std.append(np.std(wx_run, ddof=1))
    
    wy_m.append(np.mean(wy_run))
    wy_std.append(np.std(wy_run, ddof=1))
    
    offset_m.append(np.mean(offset_run))
    offset_std.append(np.std(offset_run, ddof=1))
    
    x0_m.append(np.mean(x0_run))
    x0_std.append(np.std(x0_run, ddof=1))
    
    y0_m.append(np.mean(y0_run))
    y0_std.append(np.std(y0_run, ddof=1))
    
n_atoms_m = np.array(n_atoms_m)
n_atoms_std = np.array(n_atoms_std)

wx_m = np.array(wx_m)
wx_std = np.array(wx_std)

wy_m = np.array(wy_m)
wy_std = np.array(wy_std)

offset_m = np.array(offset_m)
offset_std = np.array(offset_std)

x0_m = np.array(x0_m)
x0_std = np.array(x0_std)

y0_m = np.array(y0_m)
y0_std = np.array(y0_std)

ga.error_1D(np.linspace(0, n_mp-1, n_mp), 100*offset_m, da = 100*offset_std, title = 'Überprüfung der 2D-Fits', xlabel = 'Messpunkt', ylabel = 'Offset des 2D-Fits relativ zum Maximum [%]', style = 'bo--', label = 'Graph')


















