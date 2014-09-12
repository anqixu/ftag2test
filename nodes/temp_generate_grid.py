#! /usr/bin/env python
import random
import math
import pickle

def dist(p0, p1):
  d = [ (p0[i]-p1[i])**2 for i in range(len(p0)) ]
  return math.sqrt(sum(d))

def dist3(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2 + (p0[2] - p1[2])**2)


#######################################################
#######################################################

listx = []
listy = []
for i in range(10):
  listx.append(random.randrange(10))
  listy.append(random.randrange(10))

listxy = zip(listx,listy)
print 'listxy: ', listxy
last = listxy.pop(0)
sorted_list = [last]
while len(listxy) > 0:
  i = 0
  print 'Last: ', last
  mind = 99999
  min_idx = None
  for current in listxy:
    print 'Checking ', current, ' vs ', last
    print 'dist( ', current, ', ', last, ' )' 
    d = dist( current, last )
    if d < mind :
      print 'i = ', i
      print 'd = ', d
      mind = d
      new_min = current
      min_idx = i
    i += 1

  print 'Chosen: idx = ', min_idx, '\t Elem: ', new_min
  sorted_list.append(new_min)
  last = listxy.pop(min_idx)

  print 'Old list: ', listxy
  print 'New list: ', sorted_list, '\n\n'

print '\n\n'
print '\n\n'


#######################################################
#######################################################

def chunks(l, n):
    """ Yield successive n-sized chunks from l.
    """
    for i in xrange(0, len(l), n):
      yield l[i:i+n]

def linear_approx(z_new, corners):
  new_corners = []
  levels = list(chunks(corners, 4))
  for cor in range(0,4):
    z1 = levels[0][0][2]
    z2 = levels[1][0][2]
    l1x = levels[0][cor][0]
    l2x = levels[1][cor][0]
    l1y = levels[0][cor][1]
    l2y = levels[1][cor][1]
    l_new_x = l1x + (l2x-l1x) * (z_new-z1)/(z2-z1)
    l_new_y = l1y + (l2y-l1y) * (z_new-z1)/(z2-z1)
    new_corners.append([l_new_x, l_new_y, z_new])
  return new_corners

grid = []
listxy = []
listx = []
listy = []
sorted_list = []
positions_z = [0.2, 0.4, 0.6]
infile = open( "calib.p", "rb" )
corners = pickle.load(infile)

    
num_xy_points_per_z = 5

for z_new in positions_z:
  print 'z: ', z_new
  new_corners = linear_approx(z_new, corners)
  min_x = new_corners[0][0]
  max_x = new_corners[0][0]
  min_y = new_corners[0][1]
  max_y = new_corners[0][1]

  for corner in new_corners:
    if corner[0] < min_x:
      min_x = corner[0]
    if corner[0] > max_x:
      max_x = corner[0]
    if corner[1] < min_y:
      min_y = corner[1]
    if corner[1] > max_y:
      max_y = corner[1]

  for i in range(num_xy_points_per_z):
    listx.append(random.uniform(min_x, max_x))
    listy.append(random.uniform(min_y, max_y))  

  listxy = zip(listx,listy)
  last = listxy.pop(0)
  sorted_list = [[last[0],last[1],z_new]]
  while len(listxy) > 0:
    i = 0
    mind = 99999
    min_idx = None
    for current in listxy:
      d = dist( current, last )
      if d < mind :
        mind = d
        new_min = current
        min_idx = i
      i += 1

    sorted_list.append([new_min[0], new_min[1], z_new])
    last = listxy.pop(min_idx)
  grid.extend( sorted_list )  

  print 'Pos grid: ', grid, '\n\n'








#######################################################
#######################################################







PositionGrid = []
listxyz = []
listx = []
listy = []
listz = []
sorted_list = []
positions_z = [0.2, 0.4, 0.6]
infile = open( "calib.p", "rb" )
corners = pickle.load(infile)

min_z = 0.05
max_z = 0.8
    
num_xyz_points = 1000
for i in range(num_xyz_points):
  z_new = random.uniform(min_z, max_z)
  #print 'z: ', z_new
  new_corners = linear_approx(z_new, corners)
  min_x = new_corners[0][0]
  max_x = new_corners[0][0]
  min_y = new_corners[0][1]
  max_y = new_corners[0][1]

  for corner in new_corners:
    if corner[0] < min_x:
      min_x = corner[0]
    if corner[0] > max_x:
      max_x = corner[0]
    if corner[1] < min_y:
      min_y = corner[1]
    if corner[1] > max_y:
      max_y = corner[1]

  listx.append(random.uniform(min_x, max_x))
  listy.append(random.uniform(min_y, max_y))  
  listz.append(z_new)

listxyz = zip(listx,listy,listz)
last = listxyz.pop(0)
sorted_list = [[last[0], last[1], last[2]]]
cont = 0
while len(listxyz) > 0:
  cont += 1
  if cont % 100 == 0:
    print 'Cont = ', cont
  i = 0
  mind = 99999
  min_idx = None
  for current in listxyz:
    d = dist( current, last )
    if d < mind :
      mind = d
      new_min = current
      min_idx = i
    i += 1

  sorted_list.append([new_min[0], new_min[1], new_min[2]])
  last = listxyz.pop(min_idx)

PositionGrid = sorted_list

#for i in range(len(PositionGrid)):
#  print PositionGrid[i]
print 'Num. points: ', len(PositionGrid)












##########################################################
##     OLD POSITIONGRID
##########################################################

    positions_z = [0.2, 0.4, 0.6]
    num_samp_x = 3
    num_samp_y = 5
    infile = open( "calib.p", "rb" )
    corners = pickle.load(infile)
    
    self.PositionGrid = []
    for z_new in positions_z:
      new_corners = self.linear_approx(z_new, corners)
      min_x = new_corners[0][0]
      max_x = new_corners[0][0]
      min_y = new_corners[0][1]
      max_y = new_corners[0][1]
      for corner in new_corners:
        if corner[0] < min_x:
          min_x = corner[0]
        if corner[0] > max_x:
          max_x = corner[0]
        if corner[1] < min_y:
          min_y = corner[1]
        if corner[1] > max_y:
          max_y = corner[1]
      xs = list(numpy.linspace(min_x, max_x, num_samp_x))
      ys = list(numpy.linspace(min_y, max_y, num_samp_y)) 
      self.PositionGrid.extend( list( itertools.product( xs, ys, [z_new] ) ) )  
