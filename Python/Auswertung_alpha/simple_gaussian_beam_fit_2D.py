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
# sys.path.insert(1, "//192.168.1.3/groups/Atomics/ATOMICS-python-modules/basic-modules")
sys.path.insert(1, "C:/Users\ludwi\OneDrive\Dokumente\GitKraken\ATOMICS-python-modules\basic-modules")

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
working_on_brain43 = False

exp_code = "220930_Lightsheet_"
date, code = exp_code.split("_")[:-1]
year = "20"+date[:2]
cor_folder = "cor_"+exp_code[:-1]


path = ""
if working_on_brain43:
    path = "//brain43/groups/Atomics/Data/" +year+ "/" +date+ "/" +code+ "/" 
else:
    path = "C:/Users/ludwi/OneDrive/Dokumente/WiMi/Projekte/Lightsheet/Messungen/" 

# =============================================================================
# Camera
# =============================================================================

camera_px_to_m_constant = 2.74e-6

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

def gauss2D(data, ampl, offset, x0, y0, wx, wy): #light
    (x, y) = data
    A = ampl * np.exp(-2*(x-x0)**2/(wx**2)) * np.exp(-2*(y-y0)**2/(wy**2)) + offset
    return(A.ravel())

def gauss(x, ampl, x0, w, offset):
    A = ampl * np.exp(-2*(x-x0)**2/(w**2)) + offset
    return(A)

def gaussian_fit_1D_advanced(x, y, data, fitting_axis = 'x', p0_initial = 350, w0_initial = 20):
    (n_s, n_mp, n_y, n_x) = np.shape(data)
    if fitting_axis == 'x':
        data_1D = np.sum(data, axis = 2)
        axis = x
    if fitting_axis == 'y':
        data_1D = np.sum(data, axis = 3)
        axis = y
    dt = axis[1] - axis[0]
    n_atoms_m = []
    n_atoms_std = []
    w_m = []
    w_std = []
    for i in range(n_mp):
        n_atoms_run = []
        w_run = []
        for j in range(n_s):
            popt, perr = ga.fit_1D(gauss, axis, data_1D[j,i], initial_guess = [max(data_1D[j,i]), p0_initial, w0_initial, 0],  bounds=([0, -np.inf, 40e-6, -np.inf], [2*max(data_1D[j,i]), np.inf, 160e-6, np.inf]))
            # popt = ga.fit_1D(gauss, axis, data_1D[i,j], initial_guess = [max(data_1D[i,j]), p0_initial, w0_initial, 0])[0]
            # print(popt)
            # print(n_mp)
            # ga.reel_1D(axis, data_1D[j,i])
            # ga.reel_1D(axis, gauss(axis, *popt))
            N_atoms =  np.sqrt(2*np.pi) * popt[0] * popt[2]
            n_atoms_run.append(N_atoms/(dt))
            w_run.append(np.abs(popt[2]))
        n_atoms_m.append(np.mean(n_atoms_run))
        n_atoms_std.append(np.std(n_atoms_run))
        w_m.append(np.mean(w_run))
        w_std.append(np.std(w_run))
    n_atoms_m = np.array(n_atoms_m)
    n_atoms_std = np.array(n_atoms_std)
    w_m = np.array(w_m)
    w_std = np.array(w_std)
    return([n_atoms_m, n_atoms_std, w_m, w_std])

[n_atoms_x_m, n_atoms_x_std, wx_m, wx_std] = an.parabola_fit_1D_advanced(x, y, data, fitting_axis='x', p0_initial=590e-6, w0_initial=100e-6)
[n_atoms_y_m, n_atoms_y_std, wy_m, wy_std] = an.parabola_fit_1D_advanced(x, y, data, fitting_axis='y', p0_initial=590e-6, w0_initial=100e-6)
















