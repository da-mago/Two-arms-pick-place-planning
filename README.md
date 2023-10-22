# Two-arms-pick-place-planning

This repository contains the source code accompanying this paper:

   xxxxxx not yet available xxxxxx

![Robot setup picture missing](images/setup2.jpg)

# Installation
Download the repository and make sure all dependencies are satisfied:

    pip install -r requirements

# Introduction
The problem at hand is a multi-robot pick&place task. This project purpose is to find a way to automate the generation of a solution to this problem.
This includes managing high level concepts such as trajectories, collision avoidance, task time minimization, ...

Note: the source code is intended to be generic on the number of robots and pieces, but currently it is supporting only two robots use case.

This repository implements three different algorithms to solve the problem: Value Iteration, BFS and PDDL. In the first case, the problem is modeled as Markov Decision Process. In the second case, it is modeled as a graph and in the latter one, in PPDL language.

Note: BFS algorithm is implemented in C and later imported in the python project as a module
Example (10 pieces use-case):

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
