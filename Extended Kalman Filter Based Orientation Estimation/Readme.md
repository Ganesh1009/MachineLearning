# Extended Kalman Filter Based Orientation Estimation Using Nordic Microcontroller.
### Abstract:
Tracking the motion of the human body in real-time plays a crucial role in a simulated environment. This report details about Orientation Estimation using the Extended Kalman Filter in real-time for IMU sensor. The filter runs on the Nordic Microcontroller processing the data interfaced from small IMU sensor module containing a 3-axis accelerometer measuring the rate of change of velocity, 3-axis magnetometer measuring the magnetic field and 3-axis gyroscope measuring the angular velocity. The final orientation estimate of the IMU sensor is represented in quaternions rather than Euler-Angles or Axis/Angles pairs. The experimental timings for preprocessing of the measurements obtained from the accelerometer, magnetometer, gyroscope and the quaternion estimates of the orientation produced by filter shows the feasibility of Nordic Microcontroller interfaced with IMU sensors in real-time.

### Keywords:
Extended Kalman Filter based Filtering, IMU sensors, Nordic Microcontroller, Orientation Estimation, Quaternion.

## Introduction
Motion tracking is a critical technology in Robotics, Human-Computer Interaction, Simulated environments that require information about human motion in real-time. Thus accurate tracking of the orientation of these IMU sensors in the 3D space is vital. Precise estimates of the 3D orientation by IMU sensing unit requires exploitation of complementary properties of gyros, accelerometers and magnetic sensors. Fusing the properties of these sensors like the aiding sensors (accelerometer and magnetometer sensor) help in mitigating the gyro bias error while in turn the gyro data can be used to smooth out the errors in aiding sensors. Different approaches are available to design sensor fusionalgorithms[2]. Since orientation estimation is an inherently non-linear problem, Extended Kalman Filter is perhaps an elegant tool for combining multisensory fusion, the motion prediction and filtering. It provides a model for predicting the aspects of time behaviour of the system (dynamic model) and model for sensors measurement (measurement model)[3]. Orientation is improved by making use of quaternions which boosts computational efficienty and avoid singularities. Additionally, the use of quaternions eliminates the need for computing trigonometric functions. This report shows
that the preprocessing of the raw information obtained from the IMU and then applying the filter to estimate the quaternion orientation is feasible on Nordic Microcontroller and also experimental results validating that filter performance on the microcontroller is adequate for orientation estimation based application. Usage of the microcontroller is well suited in body sensor network based application as they are small in size and consume less power with reliable output and performance. 

Fig. 1 shows the block diagram of the approach used to estimate the orientation of the IMU unit interfaced with Nordic Microcontroller. 
![Approach](https://github.com/Ganesh1009/MachineLearning/blob/master/Extended%20Kalman%20Filter%20Based%20Orientation%20Estimation/images/Approach.PNG)

## My approach
Currently at DFKI there exists Centralized architecture with Edison(fig.2a) and Decentralized ar-
chitecture with Nordic(fig.2b).
![Architecture Style](https://github.com/Ganesh1009/MachineLearning/blob/master/Extended%20Kalman%20Filter%20Based%20Orientation%20Estimation/images/Architecture.PNG)
In the centralized architecture(fig.2a), the IMU units send the raw measurements to the Edison
microcontroller(fig.3a).
![Hardware Used](https://github.com/Ganesh1009/MachineLearning/blob/master/Extended%20Kalman%20Filter%20Based%20Orientation%20Estimation/images/Handware.PNG)
Then the calibration and the Orientation estimation process is performed on it. The Intel Edison run on the Linux(Yocto) platform and the application (DFKI BSN Framework) is developed using C++. As per the design of the decentralized architecture, each node in the kinematic chain is a smart node. The IMU units are interfaced with the Nordic(nRF52832)(fig.3b) and calibration of the raw information and its orientation estimation using EKF is computed on every node in the chain. The estimated quaternion is then transferred to the Nordic(nRF52840)(fig:3c) BLE-USB dongle which communicates with the PC to render the graphical representation of the complete kinematic chain. There exist many challenges while implementing the complete filter which is listed below:
1. Can only be implemented in C.
2. No memory management unit.
3. Implementation of math library from scratch to perform the matrix operation.
4. No foating point display for the console logs while debugging.
5. Limited memory of 512KB flash with 64KB RAM for nRF52832 and 1MB flash with 256KBRAM in nRF52840.

Due to no memory management unit in the Nordic microcontroller defragmentation of the memory is not handled. As a result of which temporary variables and arrays need to hold the intermediate computations cannot be declared with an automatic scope. During the initial stages of the implementation, the filter exhibited abnormal behaviour like the filter automatically stopped after a few
iterations. Also, the dynamic creation of the arrays to keep the matrix library generic could not be done as the performance of the filter was degraded. To increase the performance of the filter and to avoid the memory leaks, all the variable declaration and initialization of the memory for arrays dynamically is done at global scope. Debugging the errors was also quite tedious as the floating point output on the display unit was not provided. Due to this constraint, the output from themicrocontroller is multiplied by 1000 to display the oating point precision.The core library of this filter is the Math library that performs all the matrix operation. TheMatrix data-type created as part of this implementation can be used generically to create a matrix ofany dimensions. The data-type is packaged in such a way that the data-type provides access to fetchdimensions of the matrix which play a crucial role while writing tests for the matrix operations. Thisensures that the library does not perform illegal matrix operations. Since object references cannotbe established in C, matrix operations become more difficult as pointers to the matrix data-typeshave to be explicitly mentioned. The packaging of matrix data-type into a structure makes thedebugging very easy. The use of mallocs functions also provides a precise reference to the specific matrix variable which causes issues with respect to memory availability. The library is also checkedwith cppcheck with performance and warning ag enable and the results were positive with zerowarnings and performance issues.

### Experiment
Table.2 shows the time taken in microseconds for calibration of the sensor and orientation estimtion
using both EKF1 and EKF2.
![Tabulated Result](https://github.com/Ganesh1009/MachineLearning/blob/master/Extended%20Kalman%20Filter%20Based%20Orientation%20Estimation/images/ExperimentResult.PNG)
Figure.4 shows the output of the orientaion estimation.(Output with respect to each filter is not shown as the drift is observed over period of time for EKF1).

![Rendered Result](https://github.com/Ganesh1009/MachineLearning/blob/master/Extended%20Kalman%20Filter%20Based%20Orientation%20Estimation/images/ExperimentResult.PNG)

Figure.5 shows the sample output of EKF1 and EKF2 and the time take to perform the calibration and orientation estimation.
![CMD LIVE Output](https://github.com/Ganesh1009/MachineLearning/blob/master/Extended%20Kalman%20Filter%20Based%20Orientation%20Estimation/images/CMDOutput.PNG)

### Conclusion

Estimating the orientation/pose of a rigid body plays a crucial role in many application of robotics, healthcare system etc. This project gave me an opportunity to realize the wide spectrum of the Extended Kalman Filter and implement the orientation estimation on one single unit(Nordic Microcontroller interfaced with IMU) of a complete kinematic chain. I understood the overall process flow of how EKF with sensor fusion is used in estimating the orientation of a rigid body. Working
with highly motivated individuals at DFKI has, in turn, motivated me to continue my work as a
thesis on Body Sensor Network so as to understand the algorithm in its entirety and build a novel
approach.

### Referemnces 
1. Franco Ferraris, Ugo Grimaldi, and Marco Parvis. Procedure for effortless in-field calibration of three-axis
rate gyros and accelerometers. Sensors and Materials, 7:311{311, 1995.
2. Angelo Maria Sabatini. Estimating three-dimensional orientation of human body parts by inertial/mag-
netic sensing. Sensors, 11(2):1489{1525, 2011.
3. Xiaoping Yun and Eric R Bachmann. Design, implementation, and experimental results of a quaternion-
based kalman filter for human body motion tracking. Technical report, NAVAL POSTGRADUATE
SCHOOL MONTEREY CA DEPT OF ELECTRICAL AND COMPUTER , 2006.

