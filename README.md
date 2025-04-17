Self-Stabilizing Platform using 9-Axis IMU (BNO-055)

1. Introduction
The 9-axis Inertial Measurement Unit (IMU) system is a versatile and widely used control system in robotics, aerospace, and automotive applications. It provides precise orientation and motion tracking by combining data from three sensors: an accelerometer, a gyroscope, and a magnetometer. This project focuses on implementing a control system using the Adafruit BNO055, a highly integrated sensor module that simplifies sensor fusion and orientation estimation.

2. Components Identification
The components used in this project are as follows:
Component	Specification	Role in System
Arduino Uno	ATmega328P, 16MHz	Main microcontroller for signal processing and control algorithm implementation
Adafruit BNO055	9-axis absolute orientation sensor	Provides accelerometer (±2g/±4g/±8g/±16g), gyroscope (±125°/s to ±2000°/s), and magnetometer (±1300μT) data
Power Supply	5V regulated	Powers Arduino and sensor module
Servo Motors	SG90 	Actuators for self-levelling platform

3. Mathematical Model Development
Sensor Dynamics
The BNO055 incorporates three separate sensor systems whose outputs must be processed and fused:

Accelerometer Model:
ameasured = agravity + alinear + nacc

Gyroscope Model:
ωmeasured = ωtrue + bgyro + nacc

Magnetometer Model:
mmeasured  =R(mearth)+dhard+Dsoftmearth+nmag 

5. Control Loop Design
![image](https://github.com/user-attachments/assets/969b2632-afdc-4fb1-87bb-f693e6aae97a) 

Variable Definitions
	Input: Raw sensor data from accelerometer, gyroscope, and magnetometer
	Output: Rotation angles (roll, pitch, yaw) or quaternion
	Disturbances:
	Magnetic interference (±5° heading error)
	Sensor drift (0.05°/s typical)
	Linear acceleration effects (±0.2° tilt error per g)
	Parameters:
	Proportional gain: Kp=2.5
	Integral gain: Ki=0.8
	Derivative gain: Kd=0.2

Sensor Fusion Algorithm
BNO055 internal fusion (when using NDOF mode):
qestimated=ffusion(qgyro,qacc,qmag)
For custom implementation (complementary filter):
θestimated=α(θprev+ωgyroΔt)+(1−α)
PID Control Equation
For platform stabilization:
u(t) = 2.5e(t) + 0.8∫0te(τ)dτ + 0.2de(t)dt

5. Stability Analysis
Method 1: Routh-Hurwitz Criterion
For a platform stabilization system with the BNO055, the characteristic equation:
Gcl(s) = (3.1s^2+2.2s+0.8) / (s^3+5.2s^2+7.5s+0.8)

Routh Array:
 ![image](https://github.com/user-attachments/assets/61757a45-f8ca-48ba-a471-301d0108bed7)

No sign changes in first column → System stable

Method 2: Bode Plot Analysis
 ![image](https://github.com/user-attachments/assets/35064a99-3565-49e7-8275-8fa094585f7f)

Fig: Bode Plot

Bode Plot Analysis:
	Gain Margin: 9.4 dB
	Phase Margin: 52°
	Bandwidth: 2.8 rad/s

Method 3: Nyquist Criterion

 ![image](https://github.com/user-attachments/assets/e1c0083f-7dc7-4d89-bf97-06117ff8bf12)

Fig: Nyquist Plot

Nyquist Plot Analysis:
	No encirclement of critical point (-1,0)
	System is stable with adequate stability margins

Method 4: Step Response Analysis
The step response evaluates how the system reacts to a sudden change in input.

Graph:
![image](https://github.com/user-attachments/assets/88faa3ca-dcf9-414b-9d1a-e9c7acc4c40b)

 
Fig: Step Response Plot

Key Metrics:
	Rise Time: ~0.65 seconds
	Settling Time: ~2.1 seconds
	Overshoot: ~9.5%


6. Time and Frequency Response
Step Response Metrics
Parameter	Value
Rise Time	0.65s
Settling Time	2.1s
Overshoot Time	9.5%
Steady-State Error	<1%

Output Error Analysis
Orientation	Static Accuracy	Dynamic Accuracy
Roll/Pitch	±0.5°	±2°
Heading (Yaw)	±2°	±5°


7. System Performance
Robustness to Disturbances
The system demonstrates:
	Rejection of high-frequency noise via complementary filtering
	Compensation for gyroscope drift through sensor fusion
	Hard/soft iron calibration for magnetic distortion (via BNO055 internal algorithms)
Real-Time Performance
	Sample rate: 100Hz (10ms cycle time)
	Processing delay: <2ms
	Total control loop latency: <15ms

8. Conclusion
The 9-axis IMU system based on the BNO055 sensor demonstrates excellent stability and performance characteristics:
	Fast response time (rise time <0.7s)
	Good disturbance rejection (magnetic interference, vibration)
	Low steady-state error (<1°)
	Robust quaternion-based orientation tracking in 3D space
The implemented fusion algorithms successfully combine the complementary strengths of each sensor type, mitigating individual weaknesses such as gyroscope drift and accelerometer noise sensitivity. The control system meets all required specifications with adequate stability margins.


