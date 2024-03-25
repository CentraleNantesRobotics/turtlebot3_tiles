#!/usr/bin/env python3

import os
import sys
import pylab as pl
import argparse

this_dir = os.path.dirname(os.path.realpath(__file__))

model_path = os.path.abspath(f'{this_dir}/../model/tiles.cao')

parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.description = 'Generate CAO file for tiles'

parser.add_argument('-s', '--size', type=float, help='Tile size in meters',default=0.1)
parser.add_argument('-d', '--dim', type=int, help='Number of tiles to track',default=4)

args = parser.parse_args()
n = args.dim
size = args.size

# build a nxn grid with given size
points = []
for x in range(n):
    for y in range(n):
        points.append((x*size, y*size, 0.))

# build (n-1)x(n-1) tiles
squares = []
for x in range(n-1):
    for y in range(n-1):
        idx = n*x + y
        squares.append((idx, idx+1, idx+n+1, idx+n))
        #squares.append(reversed(squares[-1]))



# test...
'''
pl.close('all')
P = pl.array(points)[:,:2]

for i,p in enumerate(P):
    pl.plot(p[0],p[1],'bd')
    pl.text(p[0],p[1], str(i))

for sq in squares:
    p = P[list(sq) + [sq[0]]].T
    m = pl.mean(p, 1)
    p = (0.95*p.T + 0.05*m).T
    pl.plot(p[0], p[1])

pl.show()
'''


'''
V1
# 3D Points
8                  # Number of points
0     0      0     # Point 0: X Y Z
0     0     -0.033
0.12 0     -0.033
0.12 0      0
0.12 0.075  0
0.12 0.075 -0.033
0     0.075 -0.033
0     0.075  0     # Point 7
# 3D Lines
0                  # Number of lines
# Faces from 3D lines
0                  # Number of faces
# Faces from 3D points
6                  # Number of faces
4 0 1 2 3          # Face 0: [number of points] [index of the 3D points]...
4 1 6 5 2
4 4 5 6 7
4 0 3 4 7
4 5 4 3 2
4 0 7 6 1          # Face 5
# 3D cylinders
0                  # Number of cylinders
# 3D circles
0                  # Number of circles
'''



model = ['V1']

model.append('# points')
model.append(str(n*n))
for p in points:
    model.append(f'{p[0]} {p[1]} {p[2]}')

model.append('# lines\n0')
model.append('# faces from lines\n0')

# faces from points
model.append('# faces from points')
model.append(str(len(squares)))
for i1,i2,i3,i4 in squares:
    model.append(f'4 {i1} {i2} {i3} {i4}')

model.append('# cylinders\n0')
model.append('# circles\n0')

with open(model_path, 'w') as f:
    f.write('\n'.join(model) + '\n')

print(f'Wrote a {n}x{n} grid of size {size} @ {model_path}')
