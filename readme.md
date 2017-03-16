Introduction
===========
- Midterm Project for ENPM808x
- Tested on  Ubuntu 14.4 with GCC 4.8.4 and OpenGL 4.3.0

[![Build Status](https://travis-ci.org/banuprathap/PotentialFieldPathPlanning.svg?branch=master)](https://travis-ci.org/banuprathap/PotentialFieldPathPlanning)

[![Coverage Status](https://coveralls.io/repos/github/banuprathap/PotentialFieldPathPlanning/badge.svg?branch=master)](https://coveralls.io/github/banuprathap/PotentialFieldPathPlanning?branch=master)

Overview
=======
This is a simple path planner  implemetation in C++ .  


The project 's objective is to implement a simple path planner that interfaces with the Robot through the code API provided by the simulator and reaches the goal region with minimal computations. The main advantage of this technique can be attributed to the fact that this can be extended to any robot by simply changing  the simulator.
This project implements path planner using Artificial Potential Fields or Virtual Potential Fields in a known environment due to its mathematical simplicity. 


Algorithm:
- Gradient descent algorithm

Potential Fields
=============
- In this method, the robot is represented by a point in C-space and treated like a particle under the influence of an artificial potential field (U). U is constructed to reflect (locally) the structure of the free C-space (hence called ’local’ methods)

- This method was originally proposed by Khatib for on-line collision avoidance for a robot with proximity sensors, applicable when the robot does not have a prior model of the obstacle, but senses then during motion execution.

	 Suppose the goal is a point g∈ ℜ<sup>2</sup> and the robot is a point r ∈ ℜ<sup>2</sup>
 
- The goal region is filled with positive potential (attractive) and the obstacles are filled with high negative potential (repulsive), thus the robot gets attracted to the goal region and repelled by the obstacles.
 
- To put this in mathematical terms,

<p align="center"> U(q) = U<sub>goal</sub>(q) + ∑U<sub>obstacles</sub>(q) </p >


Now the robot tries to move towards the goal region by avoiding the obstacles. See the image below for a pictorial representation of artificial force fields
<br><br>
<br><br>
<p align="center">
<img src="http://www.cs.mcgill.ca/~hsafad/robotics/report_files/image008.jpg" alt="Artificial Potential Fields"/>
</p>

<br><br>

This project also implements a simple unit test program which utilize google test framework to 
perform tests on module function.  Test covers setting module parameters, computing shortest path,
correctness of shortest path, and robustness of computing shortest path.


Finally, the implementation process of this project follows SIP(Solo Iterative Process) model in software engineering.  
STATUS
=======
- ~~Proposal Pending Approval~~
- ~~Created Skeleton and added preliminary UML diagram~~
- ~~Implement other Callback functions~~
- ~~Change Projection for easier transformations~~
- ~~GUI with Robot and Goal position visible and updated UML~~
- Full implementaion of path planner works good

TODO
===
- ~~Learn Coverity~~ Change Travis to include coverity
- ~~Look up quality assurance tools~~ 
- ~~made _circles public  (investigate other options)~~
- ~~Implement heuristic cost function with euclidean distance~~
- ~~Implement MoveRobot()~~
Future Work
------------
The potential field path planner naturally suffers from the curse of local minima. This project will be extended in future to avoid such traps by one of the following methods.
1. Implement hybrid potential fields such that there is only one minima
2. Implement an advanced path planning algorithm like wave propagation or brush-fire algorithm on top when the robot realizes that it is stuck in the local minima. 



SIP
===
1. Implementation of GUI and simulator for the robot
	- This phase involved developing a Graphical user interface to interact with the end user. The program entirely runs on GUI and allows the user to change the goal location and add/change existing obstacles. Also, the simulator APIs were implemented during this stage, which can be accessed by the planner as and when needed.
2. Implementation of  Unit Tests and planner
    - This stage involved creating unit tests for the GUI and simulator and implementing the actual path planner itself. More unit tests were written to cover all possible cases
3. Code Optimization & Documentation
   - During this stage, all the compiler warnings were addressed and all the program blocks were commented using Doxygen syntax. This ensures the overall quality of the software product.

Backlogs can be found  [here][backlogs].

[backlogs]: (https://docs.google.com/a/terpmail.umd.edu/spreadsheets/d/1iUfZD_q6N5AdZn11GtuUNW_BCMHTRgVdXwcLb2_1gZw/edit?usp=sharing)

UML class/activity diagrams can be found [here](https://github.com/banuprathap/PotentialFieldPathPlanning/tree/master/UML%20Diagrams).



License
===

This program is under MIT License. A copy of the license can be obtained from [here](https://github.com/banuprathap/PotentialFieldPathPlanning/blob/master/LICENSE)

Copyright (c) 2017 Banuprathap Anandan
```bash
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```


 Dependencies
=======

Run the following code to install the dependencies for this project
```bash
sudo apt-get update 
sudo apt-get install build-essential
sudo apt-get install freeglut3-dev
```

Build Instructions
========

- Checkout the repo (and submodules)
```bash
$ git clone --recursive https://github.com/banuprathap/PotentialFieldPathPlanning.git
cd PotentialFieldPathPlanning
mkdir -p build && cd build
cmake ..
make
```

Running the demo
=============


- To start the program, in your build directory

```bash
./app/demo
```

- Example of demo output
- Please click [here](https://youtu.be/gD7_fiDB1RM) to view sample output



Running the unit tests
=============

- In your build directory

```bash
./test/cpp-test
```


Generating doxygen documentation
====

- In your git home directory

```bash
doxygen Doxygen
```

- Doxygen files will be generated to /docs folder