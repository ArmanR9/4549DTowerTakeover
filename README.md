# 4549DTowerTakeover

4549D's code for the Tower Takeover 2019-2020 season.

Created with PROS using C++

- `src/auto_drive` Motion Algorithims
- `src/controller_printing` Printing uitility for the controller to update data relating to the Robot
- `src/gui` Custom GUI made with LVGL
- `src/intake` Async thread for the intake subsytem
- `src/joystick` Custom joystick algorithim with custom curves and modes.
- `src/lcd` Old autonomous selector made in LLEMU
- `src/lift` Async thread for the lift subsystem
- `src/logo` C-Array of custom 4549 Robotics logo (for LVGL)
- `src/main` Handles intialization routine, autonomous routine, and opcontrol routine
- `src/motors` Intialized global motor objects and related functions
- `src/odometry` Position tracking code and utilities (using localisation technique known as Odometry)
- `src/PID` PID class that includes methods for settling utilities, and calculating output for PID loop
- `src/sensors` Intialized global sensor objects and related functions
- `src/tilter` Async thread for the tilter subsystem
- `src/utilities` Mischalenous math utilities
