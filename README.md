# UEA-AMI-Controller
Controller and setup for Haptix virtual limb environment. Go to the [Haptix website](http://gazebosim.org/haptix) for installation instructions.

## Compiling controller
To compile, go to "handsim/build/src" and run
```
cmake ../ && make
```

## Running controller
Open two terminal windows.

In the first, go to "/handsim/worlds/" and run
```
gazebo --verbose ./arat.world
```
for the MPL arm or
```
gazebo --verbose ./luke_hand.world
```
for the LUKE arm.


In the second, go to "/handsim/build/src" and run one of:
```
./hx_controller 0 0
./hx_controller 1 0
./hx_controller 0 1
```
for hardcoded trajectory, EMG trajectory, or Polhemus tracking trajectory, respectively.

Optionally, include a title for the log file, which will be saved in "/logs/". The log file saves the robot parameters, the motor commands, the motor positions, and the joint positions. If using EMG, the normalization factors and bounds are saved as well as the raw and processed EMG values. If using the Polhemus tracker, the poses of the IMUs are saved.