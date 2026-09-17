##1. Introduction
understand the chain of loop to iterate to manuvre the drone. controlling position (see the roll and pitch to be at that altitude) command ,altitude command (see angular rate ),rate controller (what I need to send to the 4 motar I have)
note :inner loop must be faster than parent loop to give the proper time for outer loop to address that command

theory:
PWM = Pulse Width Modulation.
A digital pin switches ON and OFF very fast.
The device sees the average effect of that ON/OFF signal
Period = 1 / frequency
Duty cycle = ON time / Total period × 100% (what percentage of once cycle is ON)
Resolution =how finely you can change the duty cycle.
ow finely you can change the duty cycle.
A timer is basically a counter
Then it resets back to 0.
PWM is made using this counting.

counter =1000 -->That means one full PWM cycle has 1000 counts.

Timer clock->How fast the timer counts
Prescaler->Slows down timer clock
Auto-reload / TOP value->Maximum count before reset
Compare value->Decides duty cycle

PWM frequency = Timer clock / total counts
PWM frequency = Timer clock / ((Prescaler + 1) × (TOP + 1)) [accurate]
Duty cycle = Compare value / TOP value

example:
Let timer clock = 1 MHz.
This means timer counts 1,000,000 times per second.if TOP =999
PWM frequency = 1,000,000 / 1000 = 1000 Hz


Switching Loss:Every time the motor driver turns ON or OFF, it loses some energy as heat.
high PWM frequency = More switching per second = more heat in driver

Current Ripple
Motor current does not become smooth instantly. It rises and falls.
At higher PWM = Motor current does not become smooth instantly. It rises and falls.
So motor current becomes smoother.
Higher frequency -> smoother current -> more heat
Lower frequency -> more jerky current

motor PWM
Duty cycle controls average power/voltage
20% duty -> low power
80% duty -> high power

Servo motor
Pulse width controls angle , Pulse repeats every 20 ms
1 ms   → one side
1.5 ms → middle
2 ms   → other side

Resolution vs frequency
A timer has a fixed clock speed.
If you want higher PWM frequency, the timer has less time to count in each cycle.
Less counts means lower resolution.
ex.Timer clock = 1 MHz,Timer clock = 1 MHz
1,000,000 × 0.001 = 1000 counts so 1000 level of duty level

Higher PWM frequency → less audible noise, smoother current, but lower resolution and more driver heat
Lower PWM frequency → better resolution, less driver switching heat, but more motor noise and current ripple

Each motor gives upward thrust.
Changing motor speeds creates roll, pitch, yaw.
Roll = tilt left/right.
Pitch = tilt forward/backward.
Yaw = rotate like turning head left/right.
--------------------------------------------
###Structure :

Position control (where the drone is)
↓
Velocity control(how fast)
↓
Attitude control(tilt)
↓
Rate control(how fast drone rotate)
↓
Motor commands(convert control command into 4 motor speed)
----------------------------------------------
2. Quadcopter model used

-----------------------------------------------
3. Mixer design

Convert desired thrust, roll, pitch, yaw into 4 motor commands
for going up -> all motar speed increase
for roll right -> left motors increase and right motors decrease

Input:
Total thrust
Roll torque
Pitch torque
Yaw torque
        ( MIXER )
Output:
Motor 1 thrust
Motor 2 thrust
Motor 3 thrust
Motor 4 thrust


saturation:(threshold)
Motor allowed range = 0 to 8 N

Requested 6 N  → output 6 N
Requested 10 N → output 8 N
Requested -2 N → output 0 N
--------------------------------------------------
4. Rate loop tuning

this is the innermost loop and fastest loop
how fast drone rotates
ex .roll rate = 30 deg/s
--------------------------------------------------
5. Attitude loop tuning

roll rate = 30 deg/s
It gives command to rate loop.
-----------------------------------------------
6. Velocity loop tuning

drone speed in x, y, z
It gives desired tilt to attitude loop.
------------------------------------------------

7. Position loop tuning
---------------------------------------------------
8. Bandwidth separation test

Rate loop fastest
Attitude loop slower
Velocity loop slower
Position loop slowest

If outer loop is too fast, it gives commands before inner loop can follow.
Then drone becomes unstable.
-------------------------------------------------
9. Realistic effects: motor lag, saturation, noise, delay
10. Final trajectory
11. Conclusion