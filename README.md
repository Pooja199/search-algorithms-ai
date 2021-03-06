# Project description

In this project, we twist the problem of path planning a little bit just to give you the opportunity to deepen your understanding of search algorithms by modifying search techniques to fit the criteria of a realistic problem. To give you a realistic context for expanding your ideas about search algorithms, we invite you to take part in a Mars exploration mission. The goal of this mission is to send a sophisticated mobile lab to Mars to study the surface of the planet more closely. We are invited to develop an algorithm tofind the optimal path for navigation of the rover based on a particular objective. 

The input of our program includes a topographical map of the mission site, plus some information about intended landing site and target locations and some other quantities that control the quality of the solution. The surface of the planet can be imagined as a surface in a 3-dimensional space. A popular way to represent a surface in 3D space is using a mesh-grid with a Z value assigned to each cell that identifies the elevation of the planet at the location of the cell. At each cell, the rover can move to each of 8 possibleneighborcells: North, North-East, East, South-East, South, South-West, West, and North-West. Actions are assumed to be deterministic and error-free (the rover will always end up atthe intended neighborcell).


The rover is not designed to climb across steep hills and thus moving to a neighboring cell which requires the rover to climb up or down a surface which is steeper than a particular threshold value is not allowed. This maximum slope (expressed as a difference in Z elevation between adjacent cells) will be given as aninput along with the topographical map.


# Search for the optimal paths

Our task is to move the rover from its landing site to one of the target sites for experiments and soil sampling. For an ideal rover that can cross every place, usually the shortest geometrical path is defined as the optimal path;however,since in this project we have some operational concerns, our objective is first to avoid steep areas and thuswe want to minimize the path from A to Bunder those constraints. Thus, ourgoal is,roughly, finding the shortest path among the safe paths. What defines the safety of a path is the maximum slope between any two adjacent cells along that path.

# Problem definition details

You will write a program that will take an input file that describes the terrain map, landing site, target  sites,  and  characteristics  of  the  robot.  For  each  target  site,  you  should  find  the  optimal (shortest) safe path from the landing site to that target. A path is composed of a sequence of elementary moves. Each elementary move consists of moving the rover to one of its 8 neighbors. To find the solution you will use the following algorithms:
- Breadth-first search (BFS)
-Uniform-costsearch (UCS)
-A* search(A*)

Your  algorithm  should  return  an optimal path,  that  is,  with  shortest  possible operational path length. Operational  path  length  is  further  described  belowand  is  not  equal  to  geometric  path length.  If  an  optimal  path  cannot  be  found,  your  algorithm  should  return  “FAIL”  as  further described below.

# Terrain map

We assume a terrain map that is specified as follows:A matrix with H rows (where H is a strictly positive integer) and W columns (W is also a strictly positive  integer)  will  be  given,  with a Z  elevation  value  (an  integer number,  to  avoid  rounding problems) specified in everycell of the WxH map.For example:10 20 3012 13 14is a map with W=3 columns and H=2 rows, and each cell contains a Z value (in arbitrary units).By convention,  we  will  use  North  (N),  East  (E),  South  (S),  West  (W)  as  shown  above  to  describe motions from one cell toanother. In the above example, Z elevation in the North West corner of the map is 10, and Z elevation in the South East corner is 14.To  help  us  distinguish  between  your  three  algorithm  implementations,  you  must  follow  the following conventions for computing operational path length:-

# Breadth-first search (BFS)

In BFS, each move from one cell to any of its 8 neighbors counts for a unit path cost of 1. You do not need to worry about elevation differences (except that you still need to ensure that they are allowable and not too steep for your rover),or about the fact that moving diagonally (e.g., North-East) actually is a bit longer than moving along the Northto Southor Eastto Westdirections.So, any allowed move from one cell to an adjacent cell costs 

# Uniform-cost search (UCS)

When  running  UCS,  you  should  compute  unit  path  costs  in  2D. Assume  that  cells’center coordinates projected to the 2D groundplane are spaced by a 2D distance of 10 North-South and East-West. That is, a North or South or East or Westmove from a cell to one of its 4-connected neighbors incurs a unit path cost of 10, while a diagonal move to a neighbor incurs a unit path cost of 14 as an approximation to10√ퟐwhen running UCS.

# A* search (A*)

When running A*,you should compute an approximate integerunit path costof each move in 3D, by summing the horizontal move distance as in the UCS case (unit cost of 10 when moving North to South or East to West, and unit cost of 14 when moving diagonally), plus the absolute difference in elevation between the two cells. For example, moving diagonally from one cell with Z=20 to adjacent North-East cell with elevation Z=18 would cost 14+|20-18|=16. Moving from a cell with Z=-23 to adjacent cell to the West with Z=-30 would cost 10+|-23+30|=17.You need to design an admissible heuristic for A* for this problem
