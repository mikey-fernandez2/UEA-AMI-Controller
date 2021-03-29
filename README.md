# UEA-AMI-Controller
Controller and setup for Haptix virtual limb environment. Go to the [Haptix website](http://gazebosim.org/haptix) for installation instructions.

## Compiling controller
To compile, go to "handsim/build/src" and run
```
cmake ../ && make
```

## Running controller
Open two terminal windows in "handsim/build/src".

In the first, run
```
gazebo --verbose worlds/arat.world
```
for the MPL arm or
```
gazebo --verbose worlds/luke_hand.world
```
for the LUKE arm.


In the second, run one of:
```
./hx_controller 0 0
./hx_controller 1 0
./hx_controller 0 1
```
for hardcoded trajectory, EMG trajectory, or Polhemus tracking tracjectory, respectively.
