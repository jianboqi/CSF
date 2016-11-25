
%#######################################################################################
%#                                                                                     #
%#               CSF: Airborne LiDAR filtering based on Cloth Simulation               #
%#                                                                                     #
%#  Please cite the following paper, If you use this software in your work.            #
%#                                                                                     #
%#  Zhang W, Qi J, Wan P, Wang H, Xie D, Wang X, Yan G. An Easy-to-Use Airborne LiDAR  #
%#  Data Filtering Method Based on Cloth Simulation. Remote Sensing. 2016; 8(6):501.   #
%#                                                                                     #
%# The paper can be downloaded from https://www.researchgate.net/profile/Wuming_Zhang2 #
%#                                                                                     #
%#                                     Copyright                                       #
%#               RAMM laboratory, School of Geography, Beijing Normal University       #
%#                               (http://ramm.bnu.edu.cn/)                             #
%#                                                                                     #
%#                      Wuming Zhang; Jianbo Qi; Peng Wan; Hongtao Wang                #
%#                                                                                     #
%#                      contact us: 2009zwm@gmail.com; wpqjbzwm@126.com                #
%#                                                                                     #
%#######################################################################################


%[groundIndex,nonGroundIndex]=csf_filtering(PointCloudMatrix,rigidness,isSmooth,clothResolution)
%CSF_FILTERING Filtering ground points from lidar point cloud
%   PointCloudMatrix N*3 (N*4...) matrix, each row represents a point
%   rigidness the regidness of the cloth,1 for tilted terrain, 2 for terrain with gentle slop, 3 for city areas with flat terrain
%   isSmooth is post processing 
%   clothResolution grid size of cloth

%return groundIndex index of ground points in PointCloudMatrix
%return nonGroundIndex index of nonGround points in PointCloudMatrix
%example
%[groundIndex,nonGroundIndex] = csf_filtering(pointCloud,3,true,1,0.5,500,0.65);
%groundPoints = pointCloud(groundIndex,:);
%nonGroundPoints = pointCloud(nonGroundIndex,:);


%This demo shows how to use CSF without computer vision toolbox
%read point cloud
ptCloud=dlmread('sample.txt');
%filtering operation
tic
[groundIndex,nonGroundIndex] = csf_filtering(ptCloud,3,true,1,0.5,500,0.65);
toc
%extract gound points and non-ground points
groundPoints = ptCloud(groundIndex,:);
nonGroundPoints = ptCloud(nonGroundIndex,:);
%show results
figure
plot3(groundPoints(:,1),groundPoints(:,2),groundPoints(:,3),'b.')
title('ground points')
figure
plot3(nonGroundPoints(:,1),nonGroundPoints(:,2),nonGroundPoints(:,3),'r.')
title('non-ground points, such as trees and houses')
figure
plot3(groundPoints(:,1),groundPoints(:,2),groundPoints(:,3),'b.')
hold on
plot3(nonGroundPoints(:,1),nonGroundPoints(:,2),nonGroundPoints(:,3),'r.')
legend('ground points','non-ground points, such as trees and houses')
%save gound points and non-ground points into files
dlmwrite('groundPoints.txt',groundPoints,'precision','%.6f','delimiter','\t');
dlmwrite('nonGroundPoints.txt',nonGroundPoints,'precision','%.6f','delimiter','\t');
