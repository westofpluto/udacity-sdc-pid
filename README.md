# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

Marc Ilgen, January 6, 2018

This repo represents my development of a PID controller in C++ for the SDC Term 2 PID controller project. The objective of the projct was to implement a simple PID controller for a car driving around a track and trying to maintain its position in the middle of the road (I assume, or at least to a specified road position as defined in the simulator) as closely as possible.

My implementation consisted of writing code inside the PID.cpp file and (to a lesser extent) the main.cpp file. I found it strange that the PID class did not have a method called something like "computeSteering" since that is the whole purpose of the controller. Later I realized that the "TotalError" method was supposed to be used for that purpose. I believe it is always best to name methods and functions according to what they actually do, and I find "computeSteering" to be a far better and more descriptive name. Therefore, I created a method a "computeSteering" method and called that from the main.cpp file.

A major portion of the effort of this project was to tune the control gains for the PID controller. Typically in control systm design, one has equations of motion that if linear allow one to use a range of analytical control gain design techniques (specifying gain and/or phase margin, use optimal control techniques with a Riccati equation, etc). In this project, it was most effective to simply tune the parameters using the following procedure:

1. Begin by tuning the combination of Kp and Kd. The proportional gain Kp has the effect of causingthe car to oscillate back and forth across the desired trajectory. Using only a Kd term would result in a marginally stable controller in the best of circumstances, and it would be very difficult to get the car to drive very far around the simulated track using just that term. The Kd term responds to the the lateral "velocity" (actually since the delta time in the controller was approximately constant, it was easiest to just absorb this into the Kd and Ki gains and multiply Kd by just the difference between current and previous cross-track errors). The Kd term has a damping and stabilizing effect on the controller, allowing it (in an ideal case with no secular drift) to stabilize quickly on the desired trajectory. So, I used a bit of manual adjustment (also called trial and error or manual gradient descent) to adjust the Kp and Kd parameters until I got near the performance I wanted to see.
2. Next I started increasing the Ki term. Ki is used to offset any integrated errors (such as secular drift). The downside of a Ki term is that it adds lag to the system which reduces system stabilty. One must be careful in choosing a Ki - too large a value and the controller becomes unstable. So, I gradually increased the Ki term until the overall performance was good.
3. Next, now that I had a reasobale set of values, I did more tweaking/trial and error runs to adjust the parameters to further improve the performance
4. Finally, I added a velocity-dependent component to Kd. I theorized that increasing the Kd term at higher velocities would do a better job of damping out cross-track oscillations which are more dangerous at higher velocities. I tunes the magnitude of this final hyperparameter using trial and error and came up with a set of values that led to good performance.

My final set of values were as follow:

* Kp = 0.15
* Ki = 0.002
* Kd = 30.0
* Actual Kd gain used (Kdtmp in the code) was (Kd + 0.2*speed)

The submission includes a video file of the car driving around the track using my PID controller. Note that I bumped the throttle up from 0.3 to 0.5, yielding a top speed of around 50 MPH.

The original README file contents are listed below and provide more information about setting up the project.


---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

