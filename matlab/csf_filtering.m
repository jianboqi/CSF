
%#######################################################################################
%#                                                                                     #
%#               CSF: Airborne LiDAR filtering based on Cloth Simulation               #
%#                                                                                     #
%#  Please cite the following paper, If you use this software in your work.            #
%#                                                                                     #
%#  Zhang W, Qi J, Wan P, Wang H, Xie D, Wang X, Yan G. An Easy-to-Use Airborne LiDAR  #
%#  Data Filtering Method Based on Cloth Simulation. Remote Sensing. 2016; 8(6):501.   #
%#                                                                                     #
%#                                     Copyright ?                                     #
%#               RAMM laboratory, School of Geography, Beijing Normal University       #
%#                               (http:%ramm.bnu.edu.cn/)                              #
%#                                                                                     #
%#                      Wuming Zhang; Jianbo Qi; Peng Wan; Hongtao Wang                #
%#                                                                                     #
%#                      contact us: 2009zwm@gmail.com; wpqjbzwm@126.com                #
%#                                                                                     #
%#######################################################################################
%Compile in the matlab with:

%mex csf_filtering.cpp csf.lib

function [groundIndex,nonGroundIndex]=csf_filtering(PointCloudMatrix,rigidness,isSmooth,clothResolution)
%CSF_FILTERING Filtering ground points from lidar point cloud
%   PointCloudMatrix N*3 (N*4...) matrix, each row represents a point
%   rigidness the regidness of the cloth,1 for tilted terrain, 2 for terrain with gentle slop, 3 for city areas with flat terrain
%   isSmooth is post processing 
%   clothResolution grid size of cloth

%return groundIndex index of ground points in PointCloudMatrix
%return nonGroundIndex index of nonGround points in PointCloudMatrix
%example
%[groundIndex,nonGroundIndex] = csf_filtering(pointCloud,3,true,1);
%groundPoints = pointCloud(groundIndex,:);
%nonGroundPoints = pointCloud(nonGroundIndex,:);

end

