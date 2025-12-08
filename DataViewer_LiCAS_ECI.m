%
% DataViewer_LiCAS_ECI.m
%
% Author: Alejandro Suarez, asuarezfm@us.es
% School of Engineering, University of Seville
%
% This script is used to plot the data logged by the LiCAS External Control Interface (ECI),
% including the Tool Center Point (TCP) Cartesian position, the joint position, speed, torque and PWM.
% Note: the servo actuators employed in the LiCAS A1 do not provide torque feedback, only PWM signal
% applied to the motor, that is, the normalized value of torque commanded to the motor.
%
% The LiCAS_DataLog.txt can be found in the build/Main folder after executing the program. 
%

data = load('LiCAS_DataLog.txt');

t = data(:,1);

pL = data(:,2:4);
pR = data(:,5:7);

qL = data(:,8:11);
qR = data(:,12:15);

dqL = data(:,16:19);
dqR = data(:,20:23);

tauL = data(:,24:27);
tauR = data(:,28:31);

pwmL = data(:,32:35);
pwmR = data(:,36:39);



figure(1)
subplot(1,2,1), plot3(pL(:,1), pL(:,2), pL(:,3), 'k', 'LineWidth', 2)
hold on
grid on
xlabel('X [cm]')
ylabel('Y [cm]')
zlabel('Z [cm]')
title('LEFT TCP POSITION')
axis square
subplot(1,2,2), plot3(pR(:,1), pR(:,2), pR(:,3), 'r', 'LineWidth', 2)
hold on
grid on
xlabel('X [cm]')
ylabel('Y [cm]')
zlabel('Z [cm]')
title('LEFT TCP POSITION')
axis square


figure(2)
subplot(3,2,1), plot(t, qL, 'LineWidth', 2)
hold on
grid on
ylabel('Joint position [deg]')
legend('q_1', 'q_2', 'q_3', 'q_4')
title('LEFT ARM JOINT POSITION')

subplot(3,2,2), plot(t, qR, 'LineWidth', 2)
hold on
grid on
ylabel('Joint position [deg]')
legend('q_1', 'q_2', 'q_3', 'q_4')
title('RIGHT ARM JOINT POSITION')

subplot(3,2,3), plot(t, dqL, 'LineWidth', 2)
hold on
grid on
ylabel('Joint speed [deg/s]')
title('LEFT ARM JOINT SPEED')

subplot(3,2,4), plot(t, dqR, 'LineWidth', 2)
hold on
grid on
ylabel('Joint speed [deg/s]')
title('RIGHT ARM JOINT SPEED')

subplot(3,2,5), plot(t, pwmL, 'LineWidth', 2)
hold on
grid on
ylabel('Joint PWM [-1, 1]')
xlabel('time [s]')
title('LEFT ARM JOINT PWM')

subplot(3,2,6), plot(t, pwmR, 'LineWidth', 2)
hold on
grid on
ylabel('Joint PWM [-1, 1]')
xlabel('time [s]')
title('RIGHT ARM JOINT PWM')

