# -*- coding: utf-8 -*-
"""
Created on Mon May 17 19:51:50 2021

@author: ludwig
"""

# Python Modules
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import scipy.constants as const
from PIL import Image
import locale
locale.setlocale(locale.LC_NUMERIC, "de_DE")

# Defining path for importing own modules
import sys
sys.path.insert(1, "//brain43/groups/Atomics/ATOMICS-python-modules/basic-modules")

#Graphical SetUps
plt.rcParams['axes.formatter.use_locale'] = True
mpl.rc("figure",figsize=(5,5))
mpl.rc("xtick", labelsize = 18)
mpl.rc("ytick", labelsize = 18)
mpl.rc("axes", labelsize = 20)

#Homebuild Modules
import funcs_to_create_DMD_surface as cs
import graphical_analysis as ga
import dipole_potentials as dp

working_on_brain43 = False

path = ""
if working_on_brain43:
    test_path_1 = "//brain43/groups/Atomics/DMD-masks/read_me.txt"
    test_path_2 = "//192.168.1.3/groups/Atomics/DMD-masks/read_me.txt"

    if os.path.exists(test_path_1):
        path = "//brain43/groups/Atomics/DMD-masks/"

    elif os.path.exists(test_path_2):
        path = "//192.168.1.3/groups/Atomics/DMD-masks/"

    else:
        print("Wo arbeitest du?")
else:
    # path = "C:/Users/Daniel/Desktop/ATOMICS/DMD-masks"
    path = "C:/Users/Ludwig/OneDrive/Dokumente/GitKraken/WiMi/Lehre/FP/Durchführung/DMD/Masken/"

V = 1
waist = 0.999 * 1e-3 # 0.975 * 1e-3

m = const.u * 86.909180529 # https://www-nds.iaea.org/amdc/ bzw. https://www-nds.iaea.org/amdc/ame2020/mass_1.mas20.txt (07.02.2022)
E_rec = dp.recoil(780.241*1e-9)

x = np.linspace(-608,608,1216, endpoint = False) * 5.4 *1e-6 * V
y = np.linspace(-342,342,684, endpoint = False) * 5.4 *1e-6 * V
(xm,ym) = np.meshgrid(x,y)
r = np.sqrt(xm**2+ym**2)
phi = np.arctan2(ym,xm)

def create_masks(func, it, params, filenames, R_max, sig, bool_variable_Power=False, bool_show_info = True):
    """
    Parameters
    ----------
    func : function
        defining the potential
        variables are it (as a list) and params as single values
    it : list
        the specific parameter that vary
    params : list
        fixed parameters of func
    filenames : list of str
        filenames as which the masks are saved
    R_max : float
        maximal radius we want to achieve, definies edge of flat topping area
    sig : float
        defines the Gaussian filtering. 0 gives back the original. Values around 2-3 seem to be suitable.
    bool_show_info : bool, optional
        Decides wether information regarding every created mask are printed. The default is True.

    Returns
    -------
    M : list of arrays(1216x684)
        list of the created masks as np.array
    U : float
        PD Voltage for which the masks distribute the input potential function within the adressable area
    R_max_min : float
        The minimum over all masks of maximally adressable radiuses for each mask.

    """
    N = len(it)
    pot = np.zeros((N,684,1216))
    mask = np.zeros((N,684,1216))
    mask_final = np.zeros((N,684,608))
    U = np.zeros(N)
    R_max_list = np.zeros(N)
    for j, p in enumerate(it):
        # print(f'Progress: {100*(j+1)/len(2*it):.2f} %')
        pot[j] = func(p, *params)
        cs.showpic(pot[j], str(it[j]))
        P = np.flip(pot[j, 200:-200, 466:-466], axis = 0)
        P = np.flip(P, axis = 1)
        ga.reel_2D(x[466:-466]*1e6, y[200:-200]*1e6, P, title = 'HO_ring_pot', xlabel = 'Längenskala [µm]', ylabel = 'Längenskala [µm]', colorbar_bool=False)
        mask[j], U[j], R_max_list[j] = cs.pot2mask(pot[j], R_max, V, w0 = waist, P_1V = 6.8e-3, plot_bool = False, bool_U_fixed = False, bool_show_info = bool_show_info)

        if bool_variable_Power:#Case of variable TiSa Power per DMD mask
            mask_final[j] = cs.mask2DMD(mask[j], sig = sig, filename = filenames[j], plot_bool = False)
            cs.showpic(mask_final[j], str(it[j]))

    U_max = np.max(U)
    R_max_min = np.min(R_max_list)
    if bool_variable_Power:
        np.savetxt(filenames[0]+'_Power.txt',U,fmt='%.2f,',newline='')
        return(mask_final, U_max, R_max_min, pot)

    print('------------------------------- \n')
    if not bool_variable_Power:#Case of fixed TiSa Power for every DMD mask
        for j, p in enumerate(it):
            # print(f'Progress: {100*(j+1+ len(it))/len(2*it):.2f} %')
            mask[j], U[j], R_max_list[j] = cs.pot2mask(pot[j], R_max_min, V, w0 = waist, P_1V = 6.8e-3, plot_bool = False, U_max = U_max, bool_U_fixed = True, bool_show_info = bool_show_info)
            mask_final[j] = cs.mask2DMD(mask[j], sig = sig, filename = filenames[j], plot_bool = False)
            cs.showpic(mask_final[j], str(it[j]))
        np.savetxt(filenames[0]+'_Power.txt',U,fmt='%.2f,',newline='')
        return(mask_final, U_max, R_max_min, pot)

#################################################
# Potential creating functions
#################################################

def curtain(parameters):
    axis,step = parameters[0],parameters[1]
    pot = np.zeros_like(r)
    shape = np.shape(pot)

    if axis==1:
        boundary = int(shape[1]*step)
        pot[:,:boundary] = 3
        return(pot)
    else:
        boundary = int(shape[0]*step)
        pot[:boundary,:] = 3
        return(pot)


def gaussian_dark(w, x0, y0, A):
    return(A*(1-np.exp(-2*(xm-x0)**2/w**2)*np.exp(-2*(ym-y0)**2/w**2)))

def gaussian_ring(wp, r0, phi0, wr, A):
    phase_azi = np.exp(-2*(phi-phi0)**2/wp**2)+np.exp(-2*(phi-phi0+2*np.pi)**2/wp**2)+np.exp(-2*(phi-phi0-2*np.pi)**2/wp**2)
    e_azi = phase_azi/np.max(phase_azi)
    return(A*(1-np.exp(-2*(r-r0)**2/wr**2)*e_azi))

def trafo(alpha):
    A = gaussian_ring(60e-6,0,15e-6,20,8)
    B = gaussian_dark(30e-6,0,0,8)
    return(alpha*A + (1-alpha)*B)

def potfrompic(N, pot_max):
    img = Image.open(load[N] + '.bmp') #Lade DMD-Anzeige in in 1216x684 Bild
    A = np.array(img, dtype = float)*pot_max
    # A += ((xm*np.sqrt(2)+ym*np.sqrt(2))/2+175e-6) * 0.05*m/E_rec
    A[A>6] = 6
    return(A)

def HO_2(limit,om): #U_dip in E_rec
    A = m/2*om**2*r**2*(1/E_rec)
    A[A>limit] = limit
    return(A)

def HO_transfer(x0, om, limit): #U_dip in E_rec
    A = m/2*om**2*((ym)**2+(xm-x0)**2)*(1/E_rec)
    A[A>limit] = limit
    return(A)

def HO_radial_trafo(r0,om,limit):
    pot = (m/2 * om**2 * (r-r0)**2)/E_rec
    pot[pot>limit] = limit
    return(pot)

def HO_splitting(x0, om, limit):
    pot = (m/2 * om**2 * ((np.abs(xm)-x0)**2 + ym**2))/E_rec
    pot[pot>limit] = limit
    return(pot)

def HO_ring(limit,r0,om):
    pot = (m/2 * om**2 * (r-r0)**2)/E_rec
    pot[pot>limit] = limit
    return(pot)


def HO_racetrack(om, r0, l, limit):
    pot = (m/2 * om**2 * (np.abs(ym)-r0)**2)/E_rec
    pot[xm>l] = ((m/2 * om**2 * (np.sqrt((xm-l)**2+ym**2)-r0)**2)/E_rec)[xm>l]
    pot[xm<-l] = ((m/2 * om**2 * (np.sqrt((xm+l)**2+ym**2)-r0)**2)/E_rec)[xm<-l]
    pot[pot>limit] = limit
    return(pot)

def HO_square(l, om, limit):
    b1 = (ym >= -xm) * (ym <= xm)
    b2 = (ym <= -xm) * (ym >= xm)
    b3 = (xm >= -ym) * (xm <= ym)
    b4 = (xm <= -ym) * (xm >= ym)
    pot = np.zeros((684, 1216))
    pot[b1]  = ((m/2 * om**2 * (np.abs(xm)-l)**2)/E_rec)[b1]
    pot[b2]  = ((m/2 * om**2 * (np.abs(xm)-l)**2)/E_rec)[b2]
    pot[b3]  = ((m/2 * om**2 * (np.abs(ym)-l)**2)/E_rec)[b3]
    pot[b4]  = ((m/2 * om**2 * (np.abs(ym)-l)**2)/E_rec)[b4]
    pot[pot>limit] = limit
    return(pot)

def circle(R, limit):
    A = np.ones((684, 1216))
    A[r < R] = 0
    return(A*limit)

def HO_rotation(phi0, om, R, limit):
    x0 = np.cos(phi0) * R
    y0 = np.sin(phi0) * R
    A = m/2*om**2*((ym-y0)**2+(xm-x0)**2)*(1/E_rec)
    A[A>limit] = limit
    return(A)

def pot_ramp(grad,x0):
    # grad - Potential Steepness in Joule/m
    pot = ((xm*np.sqrt(2)-ym*np.sqrt(2))/2-x0) * grad/E_rec
    limit = abs(2*x0*grad/E_rec)

    pot[pot<0] = 0
    pot[pot>limit] = limit
    return(pot)

def pot_ramp_2(gradx,grady,x0,y0):

    x = xm*np.sqrt(2)/2 + ym*np.sqrt(2)/2
    y = -xm*np.sqrt(2)/2 + ym*np.sqrt(2)/2

    # grad - Potential Steepness in Joule/m
    pot = ((x-x0)*gradx + (y-y0)*grady)/E_rec
    limit = abs(2*x0*gradx/E_rec) + abs(2*y0*grady/E_rec)

    pot[pot<0] = 0
    pot[pot>limit] = limit
    return(pot)

def barrier2(am,r0,w,grad):
    pot = am * np.exp(-(r-r0)**2/(2*w**2))
    pot += ((xm*np.sqrt(2)-ym*np.sqrt(2))/2-r0) * grad/E_rec
    pot[pot<0] = 0
    return(pot)

def HO_radial_trafo_2(r0,om,limit,rmax,gradx):
    x = xm*np.sqrt(2)/2 + ym*np.sqrt(2)/2
    y = -xm*np.sqrt(2)/2 + ym*np.sqrt(2)/2
    # R = np.sqrt(x**2 + y**2

    pot = (x+rmax)*gradx/E_rec #0.048*m/E_rec

    pot += (m/2 * om**2 * (r-r0)**2)/E_rec

    lim = limit + 2 * rmax * gradx/E_rec
    pot[pot>lim] = lim
    pot[pot<0] = 0
    return(pot)

##############################################################################

def HO_ring_new(params):
    limit,r0,om = params[0], params[1], params[2]
    pot = (m/2 * om**2 * (r-r0)**2)/E_rec
    pot[pot>limit] = limit
    return(pot)

def HO(parameter): #U_dip in E_rec
    om = parameter[0]
    limit = parameter[1]
    A = m/2*om**2*r**2*(1/E_rec)
    A[A>limit] = limit
    return(A)


def HO_radial_trafo_3(parameters):
    # r0,om,limit,rmax,gradx
    r0,om,limit,rmax,gradx,Phase_imprint_bool = parameters[0],parameters[1],parameters[2],parameters[3],parameters[4],parameters[5]

    x = xm*np.sqrt(2)/2 + ym*np.sqrt(2)/2
    y = -xm*np.sqrt(2)/2 + ym*np.sqrt(2)/2
    # R = np.sqrt(x**2 + y**2

    pot = (x+rmax)*gradx/E_rec #0.048*m/E_rec

    pot += (m/2 * om**2 * (r-r0)**2)/E_rec

    if Phase_imprint_bool:

        mask = np.copy(r)
        mask[r0-15e-6 <= r] = 1
        mask[r <= r0+15e-6] = 1
        mask[r < r0-15e-6] = 0
        mask[r0+15e-6 < r ] = 0

        pot += mask * (phi/np.pi + 1) * 0.68

    lim = limit + abs(2 * rmax * gradx/E_rec)
    pot[pot>lim] = lim
    pot[pot<0] = 0
    return(pot)

def HO_squeezed(ratio, om, limit):
    A = m/2*(om**2*xm**2+(ratio*om)**2*ym**2)*(1/E_rec)
    A[A>limit] = limit
    return(A)

def HO_squeezed_param(parameters):
    ratio, om, limit = parameters[0], parameters[1], parameters[2]
    A = m/2*(om**2*xm**2+(ratio*om)**2*ym**2)*(1/E_rec)
    A[A>limit] = limit
    return(A)

def Double_Ring(parameters):

    r1,r2,om1,om2,rmax,gradx,limit = parameters[0],parameters[1],parameters[2],parameters[3],parameters[4],parameters[5],parameters[6]

    x = xm*np.sqrt(2)/2 + ym*np.sqrt(2)/2
    y = -xm*np.sqrt(2)/2 + ym*np.sqrt(2)/2

    rm = helper_Double_Ring(om1,r1,0,om2,r2,0)

    part1 = np.copy(r)
    part2 = np.copy(r)

    part1[part1>rm]=r1
    part2[part2<=rm]=r2

    pot = m/2 * (om1**2 * (part1-r1)**2 + om2**2 * (part2-r2)**2) * (1/E_rec)
    grav = (x+rmax)*gradx/E_rec
    pot += grav
    # pot[pot>=limit]=limit
    pot = np.where(pot >= grav + limit, grav + limit, pot)

    return(pot)

def Penta_Ring(parameters):

    r1,r2,r3,r4,r5,om1,om2,om3,om4,om5,rmax,gradx,limit = parameters[0],parameters[1],parameters[2],parameters[3],parameters[4],parameters[5],parameters[6],parameters[7],parameters[8],parameters[9],parameters[10],parameters[11],parameters[12]

    x = xm*np.sqrt(2)/2 + ym*np.sqrt(2)/2
    y = -xm*np.sqrt(2)/2 + ym*np.sqrt(2)/2

    rm12 = helper_Double_Ring(om1,r1,0,om2,r2,0)
    rm23 = helper_Double_Ring(om2,r2,0,om3,r3,0)
    rm34 = helper_Double_Ring(om3,r3,0,om4,r4,0)
    rm45 = helper_Double_Ring(om4,r4,0,om5,r5,0)

    part1 = np.copy(r)
    part2 = np.copy(r)
    part3 = np.copy(r)
    part4 = np.copy(r)
    part5 = np.copy(r)

    part1[part1>rm12]=r1

    part2[part2<=rm12]=r2
    part2[rm23<=part2]=r2

    part3[part3<=rm23]=r3
    part3[rm34<=part3]=r3

    part4[part4<=rm34]=r4
    part4[rm45<=part4]=r4

    part5[part5<=rm45]=r5


    pot = m/2 * (om1**2 * (part1-r1)**2 + om2**2 * (part2-r2)**2 + om3**2 * (part3-r3)**2
                 + om4**2 * (part4-r4)**2 + om5**2 * (part5-r5)**2) * (1/E_rec)
    pot += (x+rmax)*gradx/E_rec
    pot[pot>=limit]=limit

    return(pot)

def Triple_Ring(parameters):

    r1,r2,r3,om1,om2,om3,rmax,gradx,limit = parameters[0],parameters[1],parameters[2],parameters[3],parameters[4],parameters[5],parameters[6],parameters[7],parameters[8]

    x = xm*np.sqrt(2)/2 + ym*np.sqrt(2)/2
    y = -xm*np.sqrt(2)/2 + ym*np.sqrt(2)/2

    rm12 = helper_Double_Ring(om1,r1,0,om2,r2,0)
    rm23 = helper_Double_Ring(om2,r2,0,om3,r3,0)


    part1 = np.copy(r)
    part2 = np.copy(r)
    part3 = np.copy(r)

    part1[part1>rm12]=r1

    part2[part2<=rm12]=r2
    part2[rm23<=part2]=r2

    part3[part3<=rm23]=r3


    pot = m/2 * (om1**2 * (part1-r1)**2 + om2**2 * (part2-r2)**2 + om3**2 * (part3-r3)**2) * (1/E_rec)
    grav = (x+rmax)*gradx/E_rec
    pot += grav
    # pot[pot>=grav + limit]=grav + limit
    pot = np.where(pot>=grav + limit,grav + limit,pot)
    return(pot)

def Asymmetric_Double_Rings(parameters):

    r1,r2,om_in,om_out,limit = parameters[0],parameters[1],parameters[2],parameters[3],parameters[4]


    rm = np.mean((r1,r2))

    part1 = np.copy(r)
    part2 = np.copy(r)
    part3 = np.copy(r)
    part4 = np.copy(r)

    part1[part1>r1]=r1

    part2[part2<=r1]=r1
    part2[rm<=part2]=r1

    part3[part3<=rm]=r2
    part3[r2<=part3]=r2

    part4[part4<=r2]=r2

    pot = m/2 * (om_out**2 * ((part1-r1)**2 +(part4-r2)**2) + om_in**2 * ((part2-r1)**2 + (part3-r2)**2)) * (1/E_rec)

    pot = np.where(pot>= limit,limit,pot)
    return(pot)

def phase_imprinter(parameters):
    r0 = parameters[0]
    w = parameters[1]
    mask = np.copy(r)
    mask[r0-w/2 <= r] = 1
    mask[r <= r0+w/2] = 1
    mask[r < r0-w/2] = 0
    mask[r0+w/2 < r ] = 0
    return(mask * (phi/np.pi + 1))

def double_phase_imprinter(parameters):
    r0 = parameters[0]
    w = parameters[1]

    mask0 = np.copy(r)
    mask0[r0-w/2 <= r] = 1
    mask0[r <= r0+w/2] = 1
    mask0[r < r0-w/2] = 0
    mask0[r0+w/2 < r ] = 0

    r1 = parameters[2]
    w1 = parameters[3]


    mask1 = np.copy(r)
    mask1[r1-w1/2 <= r] = 1
    mask1[r <= r1+w1/2] = 1
    mask1[r < r1-w1/2] = 0
    mask1[r1+w1/2 < r ] = 0
    return(mask0 * (phi/np.pi + 1) + mask1 * (-phi/np.pi + 1))


def stirrer(params):
    x0 = params[0]
    y0 = params[1]
    wx = params[2]
    wy = params[3]
    am = params[4]
    cos = params[5]
    sin = params[6]
    pot = 0

    for i,j in enumerate(x0):
        X = xm*cos[i] + ym * sin[i]
        Y = -xm*sin[i] + ym * cos[i]
        pot += am[i] * np.exp(-(X-j)**2/(2*wx[i]**2) - (Y-y0[i])**2/(2*wy[i]**2))

    return(pot)

def barrier(params):
    am = params[0]
    r0 = params[1]
    w = params[2]
    off = params[3]
    return(am * np.exp(-(r-r0)**2/(2*w**2)) + off)

def ring_trafo_jan(params):
    om,am,r0,w,limit = params[0],params[1],params[2],params[3],params[4]
    
    pot = (m/2 * om**2 * r**2)/E_rec + am*np.exp(-(r-r0)**2/(2*w**2))
    pot = np.where(pot>= limit,limit,pot)
    
    return(pot)

def cos_pot(params):
    om,am,limit,r_in,r_out,sigma,P= params[0],params[1], params[2],params[3],params[4],params[5],params[6]
    r0 = np.mean([r_in,r_out])
    pot = am/2 * (1 + np.cos((phi+np.pi)*om)) * np.exp(-((r-r0)**2/(2*sigma**2))**P)
    pot = np.where(pot>= limit,limit,pot)
    pot = np.where(r > r_out,0,pot)
    pot = np.where(r_in > r,0,pot)
    
    return(pot)


##################################################
# Iterating list functions
##################################################

def blackman2(r0,t):
    #t should be an array from -pi to pi for a 0 to 1 transfer
    bl2 = (21*(np.pi+t) + 25*np.sin(t) + 2*np.sin(2*t))/(42*np.pi)#Integrated Standard Blackman Pulse
    return(r0 * bl2)

def scaling_law(U0, tau, beta, t):
    return U0*1/(1+t/tau)**beta

##################################################
# Helper-functions
##################################################

def helper_Double_Ring(w1, r1, c1, w2, r2, c2):
    """
    Find location of intersection of two different harmonic potentials without
    any offset, but possibly different omegas.

    Parameters
    ----------
    w1 : TYPE
        DESCRIPTION.
    r1 : TYPE
        DESCRIPTION.
    w2 : TYPE
        DESCRIPTION.
    r2 : TYPE
        DESCRIPTION.

    Returns
    -------
    float
        radial coordinate of intersection

    """
    if w1 == w2:
        return (r1+r2)/2

    p1 = (r1*w1 - r2*w2)/(w1 - w2)
    p2 = np.sqrt(w1*w2*(r1-r2)**2 - (c1-c2)*(w1-w2))/abs(w1 - w2)

    s1 = p1 + p2
    s2 = p1 - p2

    if r1 < r2:
        temp = min(s1, s2)
        if temp > 0:
            return temp
        return max(s1, s2)

    else:
        temp = max(s1, s2)
        if temp > 0:
            return temp
        return min(s1, s2)

def dual_radius(r1,r2):
    return(list(zip(r1,r2)))

def U_new(t, tau, beta, U_init):
    return U_init/((1+t/tau)**beta)


##################################################
# Setting iterable variable and params
##################################################

### Additional Stuff
load = [path + 'Loading/white']
# load = [path + 'Loading/Dog_2']
# load = [path + 'Loading/club', path + 'Loading/spade', path + 'Loading/heart', path + 'Loading/diamond']

### define function

func = HO_radial_trafo_3

### Iterating variable

# n = range(len(load)) #potfrompic
# wp_list = np.linspace(30, 360, 4, endpoint=True) * np.pi/180 #Spot2Circle
# om_list = np.linspace(110, 110,1, endpoint = True) * 2*np.pi #HO
# r0_list = (40 + 40*np.sin(np.linspace(-np.pi,np.pi,48,endpoint=True)/2)) * 1e-6#Radial Sine Trafo
# r0_list = np.linspace(0, 80, 48)*1e-6 #HO_radial_trafo
# x0_list = np.linspace(0, 20, 48) *1e-6 #HO_splitting
# r0_list = blackman2(35, np.linspace(-np.pi,np.pi,48)) * 1e-6#Blackman Trafo_1
# r0_list = blackman2(15, np.linspace(-np.pi,np.pi,48)) * 1e-6#Blackman Trafo_2
# x0_list = np.linspace(0,15,2, endpoint=True) *1e-6 #HO_Transfer
# limit_list = np.linspace(6,6,1)
# barrier_list = [10]
# R_list = np.linspace(60,60,1) *1e-6 # circle
# phi0_list = np.linspace(0, 2*np.pi, 48) #HO_rotation lin
# phi0_list = np.pi/2+np.pi/2*np.sin(np.linspace(-np.pi/2, np.pi/2, 47)) #HO_rotation sinusoidal
# l_list = [10]
# gradient_list = [0.0474*m]
# ratio_list = np.linspace(1,1,1) # squeezed
# l_list = np.linspace(0,100,48)*1e-6

# r_outer_list = np.linspace(0, 60, 48)*1e-6
# r_inner_list = np.empty(48)
# r_inner_list[:16] = r_outer_list[:16]
# r_inner_list[16:] = 20e-6
# r0_list = dual_radius(r_inner_list, r_outer_list)#Double_Ring


"""HO(om, limit)"""
# val = np.zeros((1,2));
# val[:,0] = 2*np.pi*170;
# val[:,1] = 10;

""" HO_ring(limit,r0,om)"""
# limit_list = [10]#[i for i in range(1,11)]

"""HO_ring_new (limit,r0,om)"""
# val = np.zeros((96, 3))
# val[:, 0] = 2  # np.linspace(10, 1, 24)
# val[:, 1] = blackman2(30, np.linspace(-np.pi,np.pi,96)) * 1e-6
# val[:, 2] = 2*np.pi*120
# for radius in np.linspace(0,20,11)*1e-6:
#     val[:,1] = radius
    # val[:,2] = 2*np.pi*110

"""Ringevaporation Test HO_radial_trafo3 (r0,omega,limit,rmax,gradx,Phase_bool)"""
val = np.zeros((1,6))
val[:,0] = 40e-6#blackman2(100, np.linspace(-np.pi,np.pi,96)) * 1e-6#radius
val[:,1] = 2*np.pi*110
val[:,2] = 1#[2]*95 + [0.75]#U_new(np.linspace(0,0.2,48), 0.5, 6.84, 10)
val[:,3] = 30e-6
val[:,4] = -m*0.0
val[:,5] = False#[False]*47 + [True]


"""Double-Ring (r1,r2,om1,om2,rmax,gradx,limit)"""
# val = np.zeros((48,7))

# for r_outer in np.linspace(35,40,3):
# r_diff = r_outer - 15

# val[:,0] = list(blackman2(-r_diff, np.linspace(-np.pi,np.pi,24)) * 1e-6 + r_outer*1e-6) + [15e-6]*24# + [20e-6] * 72 #list((blackman2(17.5, np.linspace(np.pi,-np.pi,24)) +20) * 1e-6)#radius1
# val[:,1] = [r_outer*1e-6]*48# + [55e-6] * 72 #list((blackman2(17.5, np.linspace(-np.pi,np.pi,24)) +37.5) * 1e-6)#radius2
# val[:,2] = 2*np.pi*110#150
# val[:,3] = 2*np.pi*110#150
# val[:,4] = 55e-6#40*1e-6
# val[:,5] = 0#m * 0.05
# val[:,6] = [10]*24 + list(np.linspace(10,1,24)) #[10]*24 + list(np.linspace(10,1,72))

# raise SystemExit(0)

"""Triple-Ring (r1,r2,r3,om1,om2,om3,rmax,gradx,limit)"""
# val = np.zeros((96,13))
# val[:,0] = list(blackman2(-35, np.linspace(-np.pi,np.pi,48)) * 1e-6 + 55e-6) + [20e-6]*48#list(blackman2(37.5, np.linspace(-np.pi,np.pi,24)) * 1e-6) +list((blackman2(17.5, np.linspace(np.pi,-np.pi,24)) +20) * 1e-6)#radius1
# val[:,1] = 55e-6#list(blackman2(37.5, np.linspace(-np.pi,np.pi,24)) * 1e-6) +list((blackman2(17.5, np.linspace(-np.pi,np.pi,24)) +37.5) * 1e-6)#radius2
# val[:,2] = list(blackman2(35, np.linspace(-np.pi,np.pi,48)) * 1e-6 + 55e-6) + [90e-6]*48
# val[:,3] = list(2*np.pi*np.linspace(50,110,48)) + [2*np.pi*110] * 48
# val[:,4] = list(2*np.pi*np.linspace(50,110,48)) + [2*np.pi*110] * 48
# val[:,5] = list(2*np.pi*np.linspace(50,110,48)) + [2*np.pi*110] * 48
# val[:,6] = 90*1e-6
# val[:,7] = m * 0.05
# val[:,8] = [10]*48 + list(np.linspace(10, 1,48))


"""Penta-Ring (r1,r2,r3,r4,r5,om1,om2,om3,om4,om5,rmax,gradx,limit)"""
# val = np.zeros((96,13))
# val[:,0] = list(blackman2(20, np.linspace(-np.pi,np.pi,32))* 1e-6) + [20e-6]*64#list(blackman2(37.5, np.linspace(-np.pi,np.pi,24)) * 1e-6) +list((blackman2(17.5, np.linspace(np.pi,-np.pi,24)) +20) * 1e-6)#radius1
# val[:,1] = list(blackman2(20, np.linspace(-np.pi,np.pi,32))* 1e-6)  + list(blackman2(35, np.linspace(-np.pi,np.pi,32))* 1e-6+20e-6) + [55e-6]*32#list(blackman2(37.5, np.linspace(-np.pi,np.pi,24)) * 1e-6) +list((blackman2(17.5, np.linspace(-np.pi,np.pi,24)) +37.5) * 1e-6)#radius2
# val[:,2] = list(blackman2(20, np.linspace(-np.pi,np.pi,32))* 1e-6) + list(blackman2(35, np.linspace(-np.pi,np.pi,32))* 1e-6+20e-6) + list(blackman2(35, np.linspace(-np.pi,np.pi,32))* 1e-6+55e-6)
# val[:,3] = 0#blackman2(125, np.linspace(-np.pi,np.pi,48)) * 1e-6
# val[:,4] = 0#blackman2(160, np.linspace(-np.pi,np.pi,48)) * 1e-6
# val[:,5] = 2*np.pi*110
# val[:,6] = 2*np.pi*110
# val[:,7] = 2*np.pi*110
# val[:,8] = 2*np.pi*110
# val[:,9] = 2*np.pi*110
# val[:,10] = 90*1e-6
# val[:,11] = m * 0.05
# val[:,12] = 3


""" Phase_imprinter"""
# val = np.zeros((1,4))
# val[:,0] = 50e-6
# val[:,1] = 30e-6

# val[:,2] = 90e-6
# val[:,3] = 30e-6

"""Boundary"""
# val = np.zeros((1,2))
# val[:,:] = 1
# val[:,1] = 1#np.linspace(0.25,0.75,96)

"""barrier(params)"""
# val = np.zeros((6,4))
# val[:,0] = 10
# val[:,1] = 100e-6#np.linspace(25, 125,5)*1e-6
# val[:,2] = 10e-6
# val[:,3] = [0,1,2,3,4,5]

"""HO_squeezed_param"""
# ratio, om, limit = parameters[0], parameters[1], parameters[2]
# t_evap = 0

# for n_masks in [11,28,46,60,80,96]:
    # for U_f in [8,6,4,2]:
# n_masks = 1
# U_f = 10
# t_evap = (n_masks-1)*2e-3
# val = np.zeros((n_masks, 3))

# for i in [10]:#[10,20,30,40,52]:

#     n_masks = i
#     t_evap = n_masks*1e-3


# for beta in [1.5]:
#     for tau in [0.1]:
# for freq in [125, 135, 145, 153, 162, 170]:
#     for t_evap in np.array([10,20,30,40,50])*1e-3:
#         beta, tau = 2.3, 2.1
#         U_i, U_f = 10, 2
#         t_evap = ((U_i/U_f)**(1/beta)-1)*tau
#         t_evap_list = np.linspace(0, t_evap, np.shape(val)[0])

# for U_f in [3,7]:
# # tau = 9e-3/((2)**(1/beta)-1)
# t_evap_list = np.linspace(0,9,10)*1e-3

# val[:, 0] = 0.3
# val[:, 1] = 2*np.pi * 135#91.8
# val[:, 2] = np.linspace(10, U_f, n_masks)#np.linspace(10,U_f,10)#scaling_law(U0=10, tau=tau, beta=beta, t=t_evap_list)#np.linspace(0, t_evap, 24)*(-9)/t_evap+10#scaling_law(U0=U_i, tau=tau, beta=beta, t=t_evap_list)

# raise SystemExit(0)


"""Asymmetric_Double_Rings, r1,r2,om_in,om_out,limit"""
# val = np.zeros((11, 5))
# val[:, 0] = 20e-6
# val[:, 1] = 45e-6
# val[:, 2] = 2*np.pi * np.linspace(10,110,11)
# val[:, 3] = 2*np.pi * 110
# val[:, 4] = 5

""" ring_trafo_jan om,am,r0,w,limit"""
# val = np.zeros((96,5))
# val[:,0] = 2*np.pi*40
# val[:,1] = blackman2(3, np.linspace(-np.pi,np.pi,96))#Höhe des Gauß in E_Rec
# val[:,2] = 0
# val[:,3] = 10e-6
# val[:,4] = 3

"""Cos-pot om,am,limit,r_in,r_out,sigma,P"""
# val = np.zeros((1,7))
# val[:,0] = 15
# val[:,1] = 2
# val[:,2] = 2
# val[:,3] = 35e-6
# val[:,4] = 65e-6
# val[:,5] = 8e-6
# val[:,6] = 3

### fixed parameters

# params = [3] #potfrompic
# params = [50e-6, 0, 20e-6, 12] #Spot2Circle
# params = [2*np.pi*120,10] #HO 120Hz
# params = [2*np.pi*93,10] #HO 90Hz
# params = [100*2*np.pi, 6] #HO Transfer
# params = [120*2*np.pi, 10] #HO Rapid Switch
# params = [150*2*np.pi, 3] #HO Blackman 30mu,3E_R
# params = [50e-6,100*2*np.pi]
# params = [2*np.pi*100, 6] # radial trafo
# params = [100*2*np.pi, 2]
# params = [120e-6,10e-6]
# params = [120*2*np.pi, 50e-6, 10] #HO_rotation
# params = [100e-6]#,10e-6,-0.017*m]#pot_ramp
# params = [0.0*m,-100e-6,-100e-6]
# params = [2*np.pi*150,3,20e-6,0.00*m]#HO_radial_Trafo_2
# params = [2*np.pi*120, 6] # squeezed
# params = [60e-6, 2*np.pi*100]
# params = [40e-6, 30e-6, 6]
# params = [2*np.pi*100,60e-6,0.05*m,3]
# params = [45e-6, 2*np.pi*160] # HO_ring
# params = [2*np.pi*110]
# params = [150e-6, 10e-6, 0]
params = []

### Set it_list to iterating variable
it_list = val

filenames = []
folder = '230109/HO_radial_trafo_3/'
# if "t_evap" in globals():
#     folder = f"220822/HO_squeezed_evap_10Er_{U_f:d}Er_135Hz_0.68_{t_evap*1e3:.1f}ms_richtig/"#"_{beta:.2f}(beta)_{tau:.4f}(tau)/"

os.makedirs(path+folder, exist_ok=True)

for i in range(len(it_list)):
    i+=0
    # if i < 10:
    #     s = '0' + str(i)
    # else:
    #     s = str(i)
    s = format(i,'02d')
    filenames.append(path + folder +'Input_' + s)
#     # filenames.append(path + "Blenden_HO/210624_HO_" + str(round(it_list[i]/(2*np.pi))) + " Hz")


(M, U, R_max_min, pot) = create_masks(func, it_list, params, filenames, 60 * 1e-6, 0, bool_variable_Power=True, bool_show_info = True)

np.savetxt(path+folder+'Iterationparameters.txt',it_list)
# if "t_evap" in globals():
#     np.savetxt(path+folder+'Singlemasktimes.txt', t_evap_list)
#     with open(path+folder+'Evaporationparameters.txt', "w") as file:
#         file.write(f"beta: {beta:.10f}\n")
#         file.write(f"tau: {tau:.10f}\n")
#         file.write(f"U_i: {U_i:.1f} E_R\n")
#         file.write(f"U_f: {U_f:.1f} E_R\n")
#         file.write(f"t_evap: {t_evap*1e3:.6f} ms\n")

print('------------------------------- \n')
print('We have to use U = ' + str(round(U,2)) + ' V as PD Voltage.')
print('The maximally adressable area has a radius of ' + str(round(R_max_min*1e6,2)) + ' µm.')
print('Outside of this radius the full height of the potential cannot be reached even though the pixels are set to 1/white.')

# x = np.linspace(-608,608,608, endpoint = False) * 5.4 *1e-6 * V
# y = np.linspace(-342,342,684, endpoint = False) * 5.4 *1e-6 * V
# (xm,ym) = np.meshgrid(x,y)
# dx = x[1] - x[0]
# dy = y[1] - y[0]

# w = waist * V
# wh = w * 1/np.cos(26*np.pi/180)

# G = np.exp(-2*(ym**2)/w**2)*np.exp(-2*(xm**2)/wh**2)
# I = G * (2*U*6.4e-3)/(np.pi*w*wh)
# E = np.sqrt(I)*M[0]/255

# E_m = np.zeros((684, 608))
# d = 1
# for j in np.linspace(0+d, 684-d-1, 684-2*d-1, dtype = int):
#     for i in np.linspace(0+d, 608-d-1, 608-2*d-1, dtype = int):
#         E_m[j,i] = np.mean(E[j-d:j+d+1, i-d:i+d+1])

# P = np.sum(E_m**2) * dx * dy

# print(P * 1e3)
