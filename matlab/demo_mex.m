
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
%Compile in the matlab with:

%build from source
mex csf_filtering.cpp ../src/c2cdist.cpp ../src/Cloth.cpp ../src/CSF.cpp ../src/Particle.cpp ../src/point_cloud.cpp ../src/Rasterization.cpp ../src/XYZReader.cpp