# Two-arms-pick-place-planning of moving-objects

This repository contains the source code accompanying this research:

![Robot setup picture missing](images/F15a.png)

# Installation
Download the repository and make sure all dependencies are satisfied:

    pip install -r requirements

# Introduction

This article tackles the challenge of automating the pick-and-place operation of multiple moving objects by a dual-arm robot while avoiding collisions between manipulators and minimising the execution time. The system is able to generate each robot’s trajectory to complete the task in the minimum time. This application is addressed from the mathematical framework of Markov Decision Processes (MDP). To do so, the workspace has been discretized to get a finite set of states connected by a set of actions. This work is focused on automation of industrial production and uses Reinforcement Learning to achieve high level of performance and flexibility. In this study, the model is tailored to suit a deterministic setup, where a single agent oversees the entire operation and enjoys full visibility into the system. The result is a methodology for using MDPs in time-changing processes which, when applied to bimanual manipulation, provides the following advantages: It devises optimal trajectories for completing tasks satisfactorily; Accounts for collision avoidance between manipulators and; Automatically generates robot code compatible with different manufacturers. The outcome fulfils the outlined objectives, yielding a robust solution.

# Execution
Both Value Iteration and BFS approaches are implemented in python.

You can test it by running:

    python3 validation.py

Note: this will run several iterations of the problem, with different initial conditions (number of pieces, ...)

PDDL implementations requires a PDDL solver to find a solution. This implementation has been tested with Fast Downward PDDL planner (https://www.fast-downward.org/).

# Change the initial conditions
If you intend to try with different initial conditions, you'll need to edit the code for that.

In the case of value iteration or BFS, you need to specify the new input of the problem (check validation.py). That involves the initial location of the robotos, the intial and final location of the pieces, the action mode and the number of planes in the workspace.

Example:

    # EEs initial position
    armsGridPos = [[6, 4, 0], [8, 0, 0]]
    
    # Pieces (config)
    pieces_cfg = [
        {'start': [ 250, 300, 180],'end'  : [-450, 400, 180],},
        {'start': [-250, 300, 180],'end'  : [ 450, 400, 180],},
        {'start': [  50, 600, 180],'end'  : [-350, 200, 180],},
        {'start': [ -50, 600, 180],'end'  : [ 350, 200, 180],},
        {'start': [-450, 200, 180],'end'  : [ 250, 500, 180],},
        {'start': [ 450, 200, 180],'end'  : [-250, 500, 180],},
        {'start': [-350, 600, 180],'end'  : [ 150, 300, 180],},
        {'start': [ 350, 600, 180],'end'  : [-150, 300, 180],},
        {'start': [ 250, 400, 180],'end'  : [ 450, 500, 180],},
        {'start': [-250, 400, 180],'end'  : [ 350, 300, 180],}
    ]

    # Only orthogonal moves in the horizontal plane are allowed
    num_layers = 1
    action_mode = Cfg.ACTIONS_ORTHO_2D

In the case of PDDL, there is a file definition per use (depending on the action mode and the number of pieces). You need to edit the appropriate one and update initial location of the robots and initial and fina location of the pieces.
