#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# Paintrobot
# Copyright (C) 2019  Awesome Technologies Innovationslabor GmbH
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import numpy as np
import math3d as m3d
import math
import sys
import urx
from time import sleep
from svgpathtools import svg2paths, Arc, Line, QuadraticBezier, CubicBezier

np.set_printoptions(precision=3)
r = urx.Robot("192.168.0.2", use_rt=True)
r.set_tcp((0, 0, 0.15, 0, 0, 0))
r.set_payload(0.1)
sleep(0.2)

a = 0.1 # max acceleration m/s^2
v = 0.3 # max velocity m/s
curve_interp_step = 0.01 # steps in m for arc and bezier interpolation
hover = 0.02 # hover over canvas while not painting
feed = 0.003 # brush feed while painting in m/m
offset = 0.008 # initial brush when starting to paint in m

paint_depth = 0.100     # depth of color pot in m
in_paint_duration = 0.5 # duration for brush in paint pot in s
drop_off_duration = 1   # duration to stay over paint pot in s

r.set_csys(m3d.Transform())

# Joint configurations:
# Make sure that a free path exists between any of those!
j_home         = ( 0    , -math.pi/2,  0, -math.pi/2, 0, 0)
j_paint_above  = (-1.257, -1.332, -2.315, -1.065, 1.571,  0.313)
j_canvas_above = (-0.671, -1.464, -1.975,  0.026, 2.302, -0.169)
j_brush_change = (0.0, -0.725, -2.153, -1.570, 0, 0)

# Canvas coordinates (base csys):
# p0 ---------> px
# |
# |
# py                   
p0 = m3d.Transform(( 0.542,  0.241,  0.677, -1.497,  1.332, -1.134))
px = m3d.Transform(( 0.543, -0.509,  0.668, -1.497,  1.332, -1.134))
py = m3d.Transform(( 0.437,  0.245,  0.137, -1.497,  1.333, -1.134))

dx = px.pos - p0.pos
dy = py.pos - p0.pos
canvas_coordinates = m3d.Transform.new_from_xyp(dx, dy, p0.pos)

# Paint pot coordinates:
paint = {"red":    (-0.12, -0.280, 0.08, 0, np.pi, 0),
         "yellow": (-0.04, -0.280, 0.08, 0, np.pi, 0),
         "blue":   ( 0.04, -0.280, 0.08, 0, np.pi, 0),
         "black":  ( 0.12, -0.285, 0.08, 0, np.pi, 0)}

# Paint drop removal coordinates:
mesh  = {"red":    m3d.Vector(-0.12, -0.40, 0.057),
         "yellow": m3d.Vector(-0.04, -0.40, 0.055),
         "blue":   m3d.Vector( 0.04, -0.40, 0.054),
         "black":  m3d.Vector( 0.12, -0.40, 0.053)}

# Brush calibration point
brush_calib_above = (-0.168, -0.315, 0.080, 0, np.pi, 0)
brush_calib_down  = (-0.168, -0.315, 0.027, 0, np.pi, 0)

def brush_transform(index, angle, length):
    rot = m3d.Orientation.new_rot_z(index * np.pi / 2)
    rot.rotate_x(angle)
    vec = m3d.Transform(rot, (0, 0, 0)) * m3d.Vector(0, 0, length)
    return m3d.Transform(rot, vec)

# Brush calibration parameters
brush = {"red":    brush_transform(-1, 34 * np.pi / 180, 0.143),
         "yellow": brush_transform( 0, 31 * np.pi / 180, 0.148),
         "blue":   brush_transform( 1, 30 * np.pi / 180, 0.144),
         "black":  brush_transform( 2, 31 * np.pi / 180, 0.144)}

def move_home():
    print "Move to home"
    r.movej(j_home, acc=1.0, vel=v)

def move_to_canvas():
    print "Move to canvas"
    j = r.getj() # Keep orientation of last joint
    r.movej(j_canvas_above[:5] + (j[5],), acc=a, vel=v)

def move_to_paint():
    print "Move to paint"
    r.movej(j_paint_above, acc=a, vel=v)
    #j = r.getj() # Keep orientation of last joint
    #r.movej(j_paint_above[:5] + (j[5],), acc=a, vel=v)
    
def move_to_brush_change():
    print "Move to brush change"
    r.movej(j_brush_change, acc=a, vel=v)

def move_to_brush_calibration(stroke):
    print "Set base coordiante system"
    r.set_csys(m3d.Transform())

    print "Calibrate brush"
    #   Move with no brush selected to avoid extreme rotations of last joint
    r.set_tcp((0, 0, 0.15, 0, 0, 0))
    print "  Move over calibration point"
    r.movel(brush_calib_above, acc=a, vel=v)
    #   Select brush
    r.set_tcp(brush[stroke])
    r.movel(brush_calib_above, acc=a, vel=v)
    #   Move into color
    print "  Move to calibration point"
    r.movel(brush_calib_down, acc=a/2, vel=v/4)

def calibrate_brush(stroke):
    move_to_brush_calibration(stroke)
    raw_input("Measure brush length and press enter to continue...")
    r.movel(brush_calib_above, acc=a, vel=v)

def move_to_canvas_origin(stroke):
    print "Set canvas coordinate system"
    r.set_csys(canvas_coordinates)
    r.set_tcp(brush[stroke])
    r.movel((0, 0, -hover, 0, 0, 0), acc=a, vel=v)

def move_to_canvas_xaxis(stroke):
    print "Set canvas coordinate system"
    r.set_csys(canvas_coordinates)
    r.set_tcp(brush[stroke])
    r.movel((0.75, 0, -hover, 0, 0, 0), acc=a, vel=v)

def move_to_canvas_yaxis(stroke):
    print "Set canvas coordinate system"
    r.set_csys(canvas_coordinates)
    r.set_tcp(brush[stroke])
    r.movel((0, 0.55, -hover, 0, 0, 0), acc=a, vel=v)


def get_paint(stroke):
    print "Set base coordiante system"
    r.set_csys(m3d.Transform())

    # TODO: check current position
    print "  Distance to pots:", r._get_joints_dist(j_paint_above)

    print "Get new paint"
    #   Move with no brush selected to avoid extreme rotations of last joint
    r.set_tcp((0, 0, 0.15, 0, 0, 0))
    print "  Move over color pot"
    r.movel(paint[stroke], acc=a, vel=v)
    # TODO:  Measure color depth
    
    #   Select brush
    r.set_tcp(brush[stroke])
    r.movel(paint[stroke], acc=a, vel=v)
    #   Move into color
    print "  Move into color pot"
    r.down(z=paint_depth, acc=a/2, vel=v/3)
    sleep(in_paint_duration)

    print "  Move over color pot"
    r.movel(paint[stroke], acc=a, vel=v)
    print "  Wait for color to drop off"
    sleep(drop_off_duration)
    
    print "  Remove paint from tip of brush"
    radius = 0.018
    circle = [m3d.Transform(m3d.Orientation.new_rot_z(i * np.pi / 6), (0, 0, 0)) * m3d.Vector(-radius, -radius, 0) for i in range(8)]
    circle = [m3d.Transform((0, np.pi, 0), mesh[stroke] + c) for c in circle]
    circle.append(m3d.Transform(paint[stroke]))
    r.movels(circle, acc=a, vel=v/4)

def paint_path(path):
    print "Set canvas coordinate system"
    r.set_csys(canvas_coordinates)

    # TODO: check current position
    print "  Distance to canvas:", r._get_joints_dist(j_canvas_above)

    print "Paint path"
    for sub in path.continuous_subpaths():
        print "  Paint continuous sub path with length %smm" % (round(sub.length()))
        r.movel((sub.start.real / 1e3, sub.start.imag / 1e3, -hover, 0, 0, 0), acc=a, vel=v)
        poses = []
        acc_dist = 0
        for seg in sub:
            if isinstance(seg, Line):
                #print "    ", seg, "length:", seg.length()
                poses.append((seg.start.real / 1e3, seg.start.imag / 1e3, offset + feed * acc_dist / 1e3, 0, 0, 0))
            elif isinstance(seg, Arc) or isinstance(seg, QuadraticBezier) or isinstance(seg, CubicBezier):
                # one point every curve_interp_step, but at least two points
                step = min(curve_interp_step * 1e3 / seg.length(), 0.5)
                points = [seg.point(t) for t in np.arange(0, 1, step)]
                # TODO acc_dist should be incremented from point to point:
                poses.extend([(p.real / 1e3, p.imag / 1e3, offset + feed * acc_dist / 1e3, 0, 0, 0) for p in points])
            acc_dist += seg.length()
        poses.append((sub.end.real / 1e3, sub.end.imag / 1e3, offset, 0, 0, 0))
        poses.append((sub.end.real / 1e3, sub.end.imag / 1e3, -hover, 0, 0, 0))
        r.movels(poses, acc=a, vel=v/4, threshold=0.001)
    # If we are on left side of canvas move to save position first
    r.movel((0.6, 0.3, -hover, 0, 0, 0), acc=a, vel=v)

def paint_svg(paths, attributes):
    i = 0
    for (path, attr) in zip(paths, attributes):
        stroke = attr['stroke']
        print "Path", i, "with color", stroke, "of length", round(path.length())

        move_to_paint()
        try:
            get_paint(stroke)
            move_to_canvas()
            paint_path(path)
        except Exception as e:
            print "ERROR:", e
            raw_input("Press enter to continue... ")

        i += 1

if __name__ == '__main__':
    try:
        move_home()
        paths, attributes = svg2paths(sys.argv[1])
        paint_svg(paths, attributes)
    except Exception as e:
        print "ERROR:", e
        raw_input("Press enter to continue... ")
    
    
    move_to_canvas()
    move_home()

    r.stopj()
    p = r.getl()
    print "Tool pose is: ", np.array(p)
    print "Robot joints: ", r.getj()
    
    r.secmon.close()
    r.rtmon.close()
    r = None
