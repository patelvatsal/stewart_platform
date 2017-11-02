
from SPRKControl import *
import numpy as np
import time
pi = 3.14159

p = PosObj()
w = WaveObj()

def platform():
    # To set a particular position
    # p.x = 0	# set x position - y and z below
    # p.y = 0
    p.z = 0.2
    # p.xr = 0	# set x-rotation in deg - yr and zr below
    # p.yr = 0
    # p.zr = -5 # set z rotation to 5 degrees
    
    r.position(p.request) # time to target, position request
    
    # To call for wave (sinWave / mixWave / sinBreath) motion
    # w.x = [.2,.5]				# set [amplitude, frequency] of oscillations about x-axis (or y, z, xr, yr, zr below)
    # w.y = [.2,.5]
    # w.z = [.2,.5]
    # w.xr = [5,.5]
    # w.yr = [5,.5]
    # w.zr = [10,.5]
    # r.sinWave(w.request)			# Use r.sinWave(), r.mixWave, or r.sinBreath()
    return

r = SPRKControl()

r.home()							# Uncomment this line when setting the platform to home

# platform()						# Uncomment this line when calling a position or a wave motion




