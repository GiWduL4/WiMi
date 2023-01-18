# -*- coding: utf-8 -*-
"""
Created on Tue Jun 22 11:03:33 2021

@author: atomi
"""

import numpy as np
from PIL import Image as im
import matplotlib.pyplot as plt
import matplotlib as mpl
import pickle
from scipy.odr import ODR, Model, RealData
from scipy.optimize import curve_fit as c
from scipy import constants as sc
import scipy.special as sp
from scipy.stats import moment as mom
import scipy.integrate as inte

import sys
sys.path.insert(1, "C:/Users/ludwi/OneDrive/Dokumente/GitKraken/ATOMICS-python-modules/basic-modules/")

from dipole_potentials import u0,s0,recoil,harm
# import Auswertungsmethoden as a
from kamera_konstanten import c101,c102,c51,c52
import display_data as dd
# from Conical_Refraction import vect_cr_intensity,omz_b,omr_b

mpl.rc("figure",figsize=(10,5))


def lighsheet(x,y,z,y0,z0,w0y,w0z,P,lam):
    
    wy = w0y * np.sqrt(1 + (x * lam/(np.pi * w0y**2))**2)
    wz = w0z * np.sqrt(1 + (x * lam/(np.pi * w0z**2))**2)
    
    intensity = 2 * P / (np.pi * wy * wz) * np.exp(-2 * ((y-y0)/wy)**2 -2 * ((z-z0)/wz)**2)
    
    return(intensity)

def trap_frequencies(P,lam,w0y,w0z):
    xRy = np.pi * w0y**2/lam
    xRz = np.pi * w0z**2/lam
    
    omx = np.sqrt((2*abs(u0(lam))*P)/(np.pi * 87 * sc.u * w0y * w0z) * (1/xRy**2 + 1/xRz**2))
    omy = np.sqrt((8*abs(u0(lam))*P)/(np.pi * 87 * sc.u * w0y**3 * w0z))
    omz = np.sqrt((8*abs(u0(lam))*P)/(np.pi * 87 * sc.u * w0y * w0z**3))
    
    return(omx,omy,omz)

"""
Some Spatial Grids
"""
X,Y = np.meshgrid(np.linspace(-0.3,0.3,201)* 1e-3,np.linspace(-0.3,0.3,201)* 1e-3)
Y2,Z = np.meshgrid(np.linspace(-3,3,201)* 1e-3,np.linspace(-100,100,201)* 1e-6)
"""
Current Lightsheet Parameters (T.lauber Dissertation)
"""

w0z_c = 20e-6#26.2e-6
w0y_c = 3244e-6#3643e-6
lam_c = 783.55e-9
P_c = 100e-3#2/3*137.5e-3

print(np.array(trap_frequencies(P_c, lam_c, w0y_c, w0z_c))/(2*np.pi))
potential = (lighsheet(0, 0, np.linspace(-50,50,201)*1e-6, 0,0, w0y_c, w0z_c, P_c, lam_c) * u0(lam_c) + np.linspace(-50,50,201)*1e-6 * 9.81 * 87*sc.u )/recoil(780.241e-9)
print(max(potential[:100])-min(potential[50:100]))

# plt.imshow(lighsheet(X, Y, 0, 0, 0, w0y_c, w0z_c, P_c, lam_c) * u0(lam_c),cmap='jet',extent=[-3,3,-3,3])
# plt.xlabel('x [mm] (Strahlrichtung)', fontsize=15)
# plt.ylabel('y [mm]', fontsize=15)
# plt.tight_layout()
# # plt.savefig('Current/Lightfield_c.pdf',bbox_inches='tight')
# plt.show()

# plt.plot(np.linspace(-50,50,201),(lighsheet(0, 0, np.linspace(-50,50,201)*1e-6, 0,0, w0y_c, w0z_c, P_c, lam_c) * u0(lam_c) + np.linspace(-50,50,201)*1e-6 * 9.81 * 87*sc.u )/recoil(780.241e-9))
# plt.xlabel('z [µm]', fontsize=15)
# plt.ylabel('Potential [$E_R$]', fontsize=15)
# plt.grid(True)
# plt.tight_layout()
# # plt.savefig('Current/Potential_c.pdf',bbox_inches='tight')
# plt.show()
"""
Parameters Lighsheet Fibrelaser (Manual)
"""

w0z = 20.8e-6
w0y = 4900e-6
lam = 1070e-9
P = 5000e-3

"""
Parameters Lighsheet Injectionlock (MSc Felix Sommer)
"""

# w0z = 26.2e-6
# w0y = 3907e-6
# lam = 803.7e-9
# P = 1000e-3

"""
Parameters Lighsheet 808nm TA (Tilman)
"""

# w0z = 26.2e-6
# w0y = 3912.0e-6
# lam = 808.0e-9
# P = 900e-3

"""
Parameters Lighsheet 798nm TA (Tilman)
"""
# lam = 798.0e-9
# w0y = 5000e-6#3878e-6
# w0z = 20e-6#lam*400e-3/(np.pi*w0y)#26.2e-6
# P = 173e-3

"""
Parameters Lighsheet 798nm TA (Tilman)
"""
# lam = 798e-9
# w0y = 3244e-6#3643e-6
# w0z = 20e-6#lam*400e-3/(np.pi*w0y)#26.2e-6
# P = 0.25*380e-3

print(np.array(trap_frequencies(P, lam, w0y, w0z))/(2*np.pi))
potential = (lighsheet(0, 0, np.linspace(-50,50,201)*1e-6, 0,0, w0y, w0z, P, lam) * u0(lam) + np.linspace(-50,50,201)*1e-6 * 9.81 * 87*sc.u )/recoil(780.241e-9)
print(max(potential[:100])-min(potential[50:100]))


# plt.imshow(lighsheet(X, Y, 0, 0, 0, w0y, w0z, P, lam) * u0(lam),cmap='jet',extent=[-3,3,-3,3])
# plt.xlabel('x [mm] (Strahlrichtung)', fontsize=15)
# plt.ylabel('y [mm]', fontsize=15)
# plt.tight_layout()
# # plt.savefig(str(int(lam*1e9)) + 'nm/Lightfield.pdf',bbox_inches='tight')
# plt.show()

plt.plot(np.linspace(-50,50,201),(lighsheet(0, 0, np.linspace(-50,50,201)*1e-6, 0,0, w0y, w0z, P, lam) * u0(lam) + np.linspace(-50,50,201)*1e-6 * 9.81 * 87*sc.u )/recoil(780.241e-9),label='New Lightsheet')
plt.plot(np.linspace(-50,50,201),(lighsheet(0, 0, np.linspace(-50,50,201)*1e-6, 0,0, w0y_c, w0z_c, P_c, lam_c) * u0(lam_c) + np.linspace(-50,50,201)*1e-6 * 9.81 * 87*sc.u )/recoil(780.241e-9),label='Current Lightsheet')
plt.xlabel('z [µm]', fontsize=15)
plt.ylabel('Potential [$E_R$]', fontsize=15)
plt.grid(True)
plt.tight_layout()
plt.legend(loc='upper left',prop={'size':15})
# plt.savefig(str(int(lam*1e9)) + 'nm/Potential.pdf',bbox_inches='tight')
plt.show()

k = 2*np.pi/lam_c

plt.plot(np.linspace(-50,50,201),lighsheet(0, 0, np.linspace(-50,50,201)*1e-6, 0,0, w0y, w0z, P, lam) * s0(lam),label='New Lightsheet')
plt.plot(np.linspace(-50,50,201),lighsheet(0, 0, np.linspace(-50,50,201)*1e-6, 0,0, w0y_c, w0z_c, P_c, lam_c) * s0(lam_c),label='Current Lightsheet')
# plt.plot(np.linspace(-50,50,201),lighsheet(0, 0, np.linspace(-50,50,201)*1e-6, 0,0, w0y_c, w0z_c, P_c, lam_c) * s0(lam_c)*sc.hbar*k/(87*sc.u),label='Current Lightsheet')
plt.xlabel('z [µm]', fontsize=15)
# plt.ylabel(r'Acceleration $\vec{a} [\frac{m}{s^2}$]', fontsize=15)
plt.ylabel(r'Scatteringrate $\Gamma_{Sc} [\frac{1}{s}$]', fontsize=15)
plt.grid(True)
plt.tight_layout()
plt.legend(loc='upper left',prop={'size':15})
# plt.savefig(str(int(lam*1e9)) + 'nm/Scattering.pdf',bbox_inches='tight')
plt.show()
