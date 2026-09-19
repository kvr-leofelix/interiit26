###flow plan
like i want to roll to 90*-->attitude control-->rotate at this rate -->rate controller pat done by task B-->mixer which manage the torque
-----------------
ideal workflow:
Position error
V
Position controller
V
Desired velocity
V
Velocity controller
V
Desired roll/pitch
V
Attitude controller
V
Desired angular rates
V
Rate controller
V
Torque
-------------
FOR ALTITUDE :
Desired Z
V
Vertical control
V
Total thrust
------------------------
m = 1.2 kg
g ≈ 9.81 m/s²

Hover thrust ≈ 11.77 N total

Per rotor ≈ 2.94 N
Maximum total = 32 N
Hover required ≈ 11.77 N
----------------------
GIVEN ABOUT Each loop freq in ps
Rate      500 Hz
Attitude  250 Hz
Velocity  100 Hz
Position   50 Hz