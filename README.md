# CSF
Airborne LiDAR filtering method based on Cloth Simulation.

**New feature has been implemented: **
Now, We has wrapped a Python interface for CSF with swig. It is simpler to use now. It can read point cloud from txt file, or Python 2D list. This new feature can make CSF easier to be embeded into a large project. For example, it can work with Laspy (https://github.com/laspy/laspy). 

This is the code for the article:

W. Zhang, J. Qi, P. Wan, H. Wang, D. Xie, X. Wang, and G. Yan, “An Easy-to-Use Airborne LiDAR Data Filtering Method Based on Cloth Simulation,” Remote Sens., vol. 8, no. 6, p. 501, 2016.
(http://www.mdpi.com/2072-4292/8/6/501/htm)

This source code is deveoped under windows and produces DLL, which makes it convenient to be embeded into other applications.
It is very easy to compile since no external library are needed.

Currently, we only export DLL for windows applications. For linux, it is also quite easy, we will do that soon.

For binary release version, it can be downloaded at: http://ramm.bnu.edu.cn/projects/CSF/download/

Note: This code has been changed a lot since the publication of the corresponding paper. A lot of optimizations have been made. We are still working on it, and wish it could be better.

At last, if you are interested in Cloudcompare, there is a good news. our method has been implemented as a Cloudcompare plugin, you can refer to : https://github.com/cloudcompare/trunk


