clear all;close all;clc;
% MyRobot
MyRobotState=load('D:\\MyRobotGroup\\ya0\\Kinect\\Data\\MeasuredData.txt');%1-6 pos ;7-12 veliocity;13-18 acc

Pos = MyRobotState(:,1:6);
Pos(1,:);
robot_pos =fopen('robot_pos.txt','w');
for i = 1:8
    fprintf(robot_pos,' %1f %1f %1f %1f %1f %1f\n',Pos(i,:));
end

fclose(robot_pos);