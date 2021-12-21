# social-rrt-demo

## Contribution defined

In general, the contribution is related to include social cues into a sampling based planning method. In this case, the main planning method is RRT*. As a basis a modified risk related RRT* modification from Juan Hernandez is used.

The only social cue in which we will focus will be social comfort. Meaning that proxemics will be considered, leading to the following objective:

- Implementing a modified version of RRT\* capable of creating feasible dynamic paths for robot navigation around moving people.

Several stages will be carried on, step by step to accomplish the objective.

### Stage 1

For this part the following will be considered:

1. Only static people will be considered in simulation.
2. A comfort zone will be defined to interact around each people.
3. The planned solution path will have to consider path cost affected by comfort zone.

#### How to do it?

1. Static people has already been implemented in simulation using pedsim_ros. This is done by defining just a single waypoint.

2. The interaction for static and dynamic agents.

![](https://i.imgur.com/kaL4ZpR.png)

![](https://i.imgur.com/8D9VzFD.png)

3.
