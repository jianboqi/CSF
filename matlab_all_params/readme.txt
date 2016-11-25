
CSF: Ground filtering of point cloud based on Cloth Simulation


1.Introduction
2.Files list
3.Contacts

1.Introduction
Separating point clouds into ground and non-ground measurements is an essential step to generate digital terrain models (DTMs) from LiDAR (light detection and ranging) data. Many filtering algorithms have been developed. However, even state-of-the-art filtering algorithms need to set up a number of complicated parameters carefully to achieve high accuracy.
For the purpose of reducing the parameters users to set, and promoting the filtering algorithms, we present a new filtering method which only needs a few easy-to-set integer and Boolean parameters. This method is based on cloth simulation which is a 3D computer graphics algorithm and is used for simulating cloth within a computer program. So our filtering algorithm is called cloth simulation filtering, CSF.
More information of CSF and its parameters can be found at http://www.cloudcompare.org/doc/wiki/index.php?title=CSF_(plugin).

CSF implemented the algorithm proposed by the paper "Zhang W, Qi J, Wan P, Wang H, Xie D, Wang X, Yan G. An Easy-to-Use Airborne LiDAR Data Filtering Method Based on Cloth Simulation. Remote Sensing. 2016; 8(6):501.",which can be downloaded from https://www.researchgate.net/profile/Wuming_Zhang2. Please cite this paper, if you use this software in your work.

The usage is very simple. [groundIndex,nonGroundIndex] = csf_filtering(pointcloud,typeofscene,postprocessing,gridsize); Sometime, only the type of the scene is needed to be set by the user. More details can be found in demos.

CSF has been integrated into two free softwares for point cloud processing. If you want to use it with a graphical user interface (GUI), you can download CloudCompare from http://www.cloudcompare.org/ or Point Cloud Magic from http://lidar.radi.ac.cn (In Chinese).

2.File list
CSF.h				header file
CSF.lib				lib file
csf_filtering.cpp		cpp for compiling mex
demo_mex.m			m file for compiling mex
demo_with_toolbox.m		demo for usage with computer vision toolbox
demo_without_toolbox.m		demo for usage without computer vision toolbox
license.txt			licence
point_cloud.h			header file
readme.txt			this readme file
sample.ply			sample file used by demo_with_toolbox.m
sample.txt			sample file used by demo_without_toolbox.m

3.Contacts
               RAMM laboratory, School of Geography, Beijing Normal University       
                               (http://ramm.bnu.edu.cn/)                             
                                                                                     
                      Wuming Zhang; Jianbo Qi; Peng Wan; Hongtao Wang                
                                                                                     
                      contact us: 2009zwm@gmail.com; wpqjbzwm@126.com                