# Minimum Jerk Trajectory Generation For UAV Navigation in Indoor Dense Environment

In this project, we proposed an efficient and low computation cost online motion planning framework for UAV navigation in indoor dense environment.  we adopt A* to find a collision-free path and utilize $Ramer–Douglas–Peucker(RDP)$ algorithm to simplify the path into certain critical waypoints. Thanks to the $Optimality\ Conditions$ of minimum jerk  polynomial trajectory, we can solve the trajectory optimization without the need of cost function. The simulation results show the average time consuming of planning module is less than 10ms and the maximum velocity of UAV can reach 1.2m/s.

## 1 .$Main\ Contribution$

- Implementing $RDP$ algorithm to simplify the front-end path to certain critical waypoints for trajectory generation. (see/Astar_searcher.cpp/pathSimplify())
- Based on $Optimality\ Conditions$ of minimum jerk polynomial trajectory, we solve the unconstrained trajectory generation problem without cost function with linear time and space complexity. (see trajectory_generator_waypoint.cpp/minimumJerkTrajGen())
- Implementing $reoptimize$ strategy to regenerate trajectory if there are collision segments of the whole trajectory. (see/trajectory_generator_node.cpp/trajOptimization() ).

## 2. Simulation Result

![min_jerk1](https://github.com/Lkaho/Minimum-Jerk-Trajectory-Generation-For-UAV-Navigation-in-Indoor-Dense-Environment/blob/main/min_jerk1.gif)

![min_jerk2](https://github.com/Lkaho/Minimum-Jerk-Trajectory-Generation-For-UAV-Navigation-in-Indoor-Dense-Environment/blob/main/min_jerk2.gif)
