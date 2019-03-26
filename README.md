# Trajectory Generation and Control of a Quadrotor

## Introduction
We will autonomously control a simulated quadrotor through a 3D environment with obstacles.

## Quadrotor
The simulated quadrotor is assumed to be a cylinder with radius of 0.15 m and height of 0.1 m.

## OccMap Class
This class loads an environment stored in text (txt) format. Each txt file contains the size of the environment, as well as the size and position of each obstacle. The OccMap converts this into an occupancy map, stored as a 3D matrix, where map.occgrid(i,j,k) is 0 if index (i,j,k) in the map is free, and 1 otherwise.  
**map.occgrid** - The occupancy grid stored inside the class object.  
**map.res_xyz** - The size of each cell in the occupancy map in each dimension (in meters).  
**map.margin** - The minimum distance a point can be from an obstacle before it is considered to be inside
"occupied" space. A larger margin typically means a safer (but potentially longer) trajectory.  
**map = load_map(filename, res_xy, res_z, margin)** - loads a map stored in txt format with xy resolution res_xy, z resolution res_z and margin.  
**collides = map.collide(xyz)** - checks if the points in xyz (Nx3) collide with any obstacles in the map.
Note that this will crash on points outside the map, which need to be handled before using this function.  
**xyz = map.subToXYZ(i,j,k)** - converts the subs in i,j,k (Nx1 each) to metric positions xyz, where map.occgrid(i, j, k) corresponds to the cell containing point xyz.  
**[i,j,k] = map.xyzToSub(xyz)** - does the inverse of subToXYZ, takes metric positions xyz (Nx3) and returns subs i, j and k.  
**xyz = map.indToXYZ(ind)** - converts the 1D index ind of a point inside the map to metric coordinates xyz.  
**ind = map.xyzToInd(xyz)** - does the inverse of indToXYZ, takes metric coordinates xyz inside the map and returns the 1D index ind for the corresponding point inside the occgrid.  

## Trajectory Generation
The optimal path output from the implementation of Dijkstra or A* usually contains many sharp turns due to the voxel grid based discretization of the environment. To improve the performance, trajectory smoothing techniques are used to convert sharp turns into smooth trajectories that the robot can track.  
Note that we need to decide on a velocity profile to turn the path from Dijkstra or A* into a
trajectory. The speed of the robot does not need to be a constant.

## Controller
We implemented a controller in file **controller.m** that makes sure that the quadrotor follows the
desired trajectory. The controller takes in a cell struct ***qd***, current time ***t***, current quadrotor number ***qn*** and quadrotor parameters ***params*** and outputs thrust ***F***, moments ***M***, desired thrust, roll, pitch and yaw ***trpy*** and derivatives of desired roll, pitch and yaw ***drpy***.

## Collision
The quadrotor should fly as fast as possible. However, a real quadrotor is not allowed to collide with
anything. Therefore, we have zero tolerance towards collision.


