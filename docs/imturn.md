# IM Turn Algorithm

[Back to Landing Page](/README.md)

_Note: this algorithm prototype is provided as [MATLAB m-code](https://github.com/mitre/im_sample_algorithm/blob/master/IMTurn/main_closedLoopTurn_pathShortening.m). We do not provide any compilable connection between the MATLAB code and the C++ code base._

## Entry Points

The code is run by calling `main_closedLoopTurn_pathShortening.m`. The function takes a cell array of airports, a vector of initial spacing values between Ownship and Traffic (in NM), a plotting flag (1= on, 0=off), and a string variable called routeType which can be either “short” or “long”. The easiest method to get up and running is to load one of the results .mat files from the Data folder and then call `main_closedLoopTurn_pathShortening.m`. 

Loading one of these files and calling main is done as follows:

```matlab
load('Data\LongRouteResults.mat')
[totalIterations, validity, initError_s, waypointSequence, turnPoint] = main_closedLoopTurn_pathShortening(airports,initSpacing,plotting,routeType)
```

The main function will loop through the airports and initial spacing values, gathering any needed IFPI from the .txt files. To simulate IM Turn at airports which aren’t already in the database, a route must be added to the route .txt files and the locations of each waypoint along with altitude and speed constraints must be added to wptDatabase.txt. IM Turn is a tricky beast, new routes may not run without modification. The algorithm is also sensitive to the initial spacing, so some work may be needed to identify a working initial spacing between Ownship and Traffic for a new route.

## Returned Data

The main function returns the total number of iterations, the results of the validity check, the initial error in seconds, the (x,y) values of the route waypoints in NM with and without the IM Turn Point, and the (x,y) location of the IM Turn Point in NM. When plotting is turned on, the main function also generates plots of the route with and without the IM Turn Point, as well as the location of the iterative attempts to find the IM Turn Point.
