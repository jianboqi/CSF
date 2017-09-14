
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


%This demo shows how to use CSF with computer vision toolbox
%read point cloud
ptCloud=pcread('sample.ply');
%filtering operation
tic
[groundIndex,nonGroundIndex] = csf_filtering(ptCloud.Location,3,true,1,0.5,500,0.65);
toc
%extract gound points and non-ground points
groundPoints = pointCloud(ptCloud.Location(groundIndex,:));
nonGroundPoints = pointCloud(ptCloud.Location(nonGroundIndex,:));
%show results
figure
pcshow(groundPoints)
title('ground points')
figure;
pcshow(nonGroundPoints)
title('non-ground points, such as trees and houses')
figure
pcshowpair(groundPoints,nonGroundPoints)
title('ground points and non-ground points, such as trees and houses')
%save gound points and non-ground points into files
pcwrite(groundPoints,'groundPoints','PLYFormat','binary');
pcwrite(nonGroundPoints,'nonGroundPoints','PLYFormat','binary');