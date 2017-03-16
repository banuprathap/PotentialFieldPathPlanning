Introduction
===========
- Midterm Project for ENPM808x
- Tested on  Ubuntu 14.4 with GCC 4.8.4 and OpenGL 4.3.0

[![Build Status](https://travis-ci.org/banuprathap/PotentialFieldPathPlanning.svg?branch=master)](https://travis-ci.org/banuprathap/PotentialFieldPathPlanning)

[![Coverage Status](https://coveralls.io/repos/github/banuprathap/PotentialFieldPathPlanning/badge.svg?branch=master)](https://coveralls.io/github/banuprathap/PotentialFieldPathPlanning?branch=master)


STATUS
=======
- ~~Proposal Pending Approval~~
- ~~Created Skeleton and added preliminary UML diagram~~
- ~~Implement other Callback functions~~
- ~~Change Projection for easier transformations~~
- GUI with Robot and Goal position visible and updated UML

TODO
===
- ~~Learn Coverity~~ Change Travis to include coverity
- Look up quality assurance tools 
- made _circles public  (investigate other options)
- Implement heuristic cost function with euclidean distance
- Implement MoveRobot()

## Overview


## License

This program is under MIT License.

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


## Dependencies
Run the following code to install the dependencies for this project
```bash
sudo apt-get update 
sudo apt-get install build-essential
sudo apt-get install freeglut3-dev
```

## How to build

- Checkout the repo (and submodules)
```bash
$ git clone --recursive https://github.com/banuprathap/PotentialFieldPathPlanning.git
cd PotentialFieldPathPlanning
mkdir -p build && cd build
cmake ..
make
```

## How to run demo


- To start the program, in your ./build directory

```bash
./app/shell-app
```




- Example of demo output




## How to run unit tests

- In your ./build directory

```bash
./test/cpp-test
```


## How to generate doxygen documentation

- In your . directory

```bash
doxygen ./Doxygen
```

- Doxygen files will be generated to ./docs folder