##BASIC
A motor converts electrical energy into mechanical rotational energy. The two most important mechanical output quantities are torque and angular speed
power=Tw
T=torque
w=angular velocity
------------------------
##Brushed DC motor:
have a ring , fixed magnet called stator and moving coil called rotating armature
Brushes and a mechanical commutator automatically reverse the armature current as the rotor turns so that the produced torque remains in the required direction.

input =voltage and current
for controlling the speed and direction we use motor driver which use PWM as input from the MCU

torque prop to current
back emf more if it spin faster.to becuse to oppose the applied voltage
τ = K(V - Keω)/R
Kew is back emf and V is supplied voltage
k=torque contant
At nearly zero load, speed approaches the no-load speed and current becomes relatively small.TODO:
at w=0 the current is ideally V/R so very big
Stall: maximum torque, maximum current, zero speed
No load: maximum speed, low torque/current
Between them: usable operating region
using motor driver increasing PWM duty cycle increases the effective applied voltage
--------------------------------------------
##BLDC MOTOR:
no mechanical contact between ring and mech .brush
magnet at rotator rnd winding is stator
esc use to do the switching of current
Rotor position may be tell by hall senor,encoder , back emf
at no load rpm =kv * voltage (real with load is less)
torque =k *I
mcu to esc communicate with =pwm , can,uart,analog or torque and current command (interface)
-----------------------------------------------
##STEPPER MOTOR:

moves in steps.like T660 moves 200 step per rev and with the stepper driver using the dip we can change it with power of 2**n)*200 this is microstepping

communication -->mcu-->driver [step pulses (steps move)/dir logic level(direction)]
Without feedback, the controller cannot directly know that happened.why stepper lose steps: Excessive load torque
Excessive acceleration:TODO:
High speed
Insufficient phase current
Supply-voltage limitations at high speed
Resonance
Mechanical binding
Driver overheating/current limiting

benefit of microsteping:
Smoother movement
Reduced vibration/resonance
Finer command resolution
Quieter operation

A closed-loop stepper adds an encoder and controller that measures actual rotor position and corrects errors.open loop have no encoder so we don't know if he stepper has moved or not


--------------------------------
##SERVO:
dc motor+position sensor+internal feedback controller+driver+GEARBOX
note:pwm in servo dc brush motor control avg voltage which change the rpm,but in servo pwm target position
---------------------------------
##stepper vs servo:
servo is closed loop,adjust postion and speed
Stepper motors provide high torque at low speeds, but torque drops significantly as speed increases. Servo motors maintain consistent, high torque across a wide speed range
Servo motors correct errors on the fly and never lose synchronization


stepper move in discrete increment via open loop or closed loop
Stepper motors provide high torque at low speeds, but torque drops significantly as speed increases
Stepper motors can skip steps if overloaded, leading to errors
--------------------------------------
##differnce table

Brushed DC:
Simple and inexpensive
Excellent for basic wheel drives
Requires H-bridge
Encoder needed for accurate closed-loop speed/position

BLDC:
Efficient and high power density
Requires esc/foc
Excellent for high-performance drives
More complex controller/ESC

Stepper:
Excellent for repeatable incremental positioning
Can operate open-loop
Strong low-speed/holding behavior
Can silently lose steps
Torque drops with speed

Servo:
Integrated closed-loop position system
Easy position control
Common for relatively compact joints
Range, torque and precision depend strongly on servo/geartrain
----------------------------------
##NOTE

Rated/continuous current:
A current the motor is designed to handle continuously under specified thermal conditions.
Peak current:
A larger current allowed for only a limited time.
Stall current:
Current when the motor cannot rotate while full voltage is applied

Rated torque:
Torque sustainable under specified continuous conditions.
Stall torque:
Torque produced at zero speed at the stated electrical condition.
Holding torque for a stepper:
Maximum stationary torque under specified energization conditions.

Motor temperature also depends on thermal resistance, cooling, ambient temperature, duty cycle, iron losses and other factors.
--------------------------------
##GEARBOX THROEY
G=INPUT RPM/OUTPUT RPM
OUTPUT TORQUQ=INPUT TORQUE*G
also you can do the inverse

A gearbox also affects:
Backlash (lost motion btw input and output when direction reverse.If gears have clearance, the motor can rotate slightly after reversal before the output starts moving)
Back-drivability
Mechanical compliance
Efficiency
Reflected inertia
Effective encoder resolution
-----------------------------------
##FOR DEIMOS REMEBER:
TO SELET THE DRIVER SELECTION 
CHECK:
Battery/supply voltage
Maximum bus voltage
Continuous motor current
Peak/stall/transient current
Thermal resistance/cooling conditions
PWM capability
Protection behavior
Regenerative energy behavior

ALSO READ AND IMPLEMENT :Overcurrent protection
Overtemperature shutdown
Undervoltage lockout
Short-circuit protection
----------------------------
##TO PREVENT THE REGENRATION ENERGY WHEN BRAKE ARE APPLY IT ASCT AS BATTERY:
Freewheel/flyback diodes
MOSFET body diodes
Synchronous MOSFET switching
Snubbers/clamps
-----------------------------------------
#BACK EMF AND FLYBACK
Back-EMF is generated by motor rotation:E = Keω
Flyback/inductive kick is caused by forcing winding current to change rapidly:V = L di/dt
-----------------------------
Two cables running parallel for a long distance have substantial coupling length.

Crossing them approximately at right angles greatly reduces the distance over which strong coupling occurs.
-------------------------------
Twisting also causes successive sections of the pair to experience interference with alternating geometry, helping unwanted coupling cancel.

For an encoder, pairing a signal with its return is usually much better than routing the signal separately from its return.
----------------------------
#NOTE FROM ME
(NEED MORE TIME TO UNDER BETTER ABOUT ELECTRICAL NOISE AND EMI)
Sources of Motor Noise
Capacitive Coupling
Inductive Coupling
Grounding and Ground Loops
Twisted Pair
Shielding
Ferrites and Filtering
Decoupling and Bulk Capacitance
Cable Routing
Differential Signalling