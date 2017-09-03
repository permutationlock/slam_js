# DP SLAM in JavaScript
 A JavaScript implementation of the basic
 [DP Slam](https://users.cs.duke.edu/~parr/dpslam/) algorithm.
 
## Features

### Modular Design
 A "modular" design that allows different map, motion, and sensor models. We
 first provide an implementation of a basic particle filter. We then use this
 particle filter to construct a generic implementation of the DP SLAM algorithm
 as a JavaScript class.
 
 Abstract concepts are described for map, motion, and sensor model objects. The
 DP SLAM object may then be instantiated with any parameters satisfying the
 necessary concepts.
 
 Concrete map, motion, and sensor model classes are provided that implement the
 ideas in the
 [DP SLAM 1.0 paper](http://people.ee.duke.edu/~lcarin/Lihan4.21.06a.pdf).
 
### Simulation
 A very basic simulation of a robot with a laser range sensor is provided in
 the main.html file. Walls may be added by clicking twice on the canvas. The
 simulation may be started and paused by pressing space. A particle is sampled
 from the DP SLAM particle filter at fixed time intervals. The grid shows what
 the current sampled particle predicts the environment to be. The green line shows the
 actual position and orientation of the robot. The blue line shows the
 what the current sampled particle predicts the robot's position and orientation
 to be.


