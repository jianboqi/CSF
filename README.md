![csf1](https://github.com/jianboqi/CSF/blob/master/CSFDemo/CSF1.png) ![csf2](https://github.com/jianboqi/CSF/blob/master/CSFDemo/CSF2.png)
# CSF
Airborne LiDAR filtering method based on Cloth Simulation.
This is the code for the article:

W. Zhang, J. Qi*, P. Wan, H. Wang, D. Xie, X. Wang, and G. Yan, “An Easy-to-Use Airborne LiDAR Data Filtering Method Based on Cloth Simulation,” Remote Sens., vol. 8, no. 6, p. 501, 2016.
(http://www.mdpi.com/2072-4292/8/6/501/htm)


**New feature has been implemented:**

Now, We has wrapped a Python interface for CSF with swig. It is simpler to use now. This new feature can make CSF easier to be embeded into a large project. For example, it can work with Laspy (https://github.com/laspy/laspy). What you do is just read a point cloud into a python 2D list, and pass it to CSF.
The following example shows how to use it with laspy.
```python
# coding: utf-8
import laspy
import CSF
import numpy as np

inFile = laspy.file.File(r"in.las", mode='r') # read a las file
points = inFile.points
xyz = np.vstack((inFile.x, inFile.y, inFile.z)).transpose() # extract x, y, z and put into a list

csf = CSF.CSF()

# prameter settings
csf.params.bSloopSmooth = False
csf.params.cloth_resolution = 0.5
# more details about parameter: http://ramm.bnu.edu.cn/projects/CSF/download/

csf.setPointCloud(xyz)
ground = CSF.VecInt()  # a list to indicate the index of ground points after calculation
non_ground = CSF.VecInt() # a list to indicate the index of non-ground points after calculation
csf.do_filtering(ground, non_ground) # do actual filtering.

outFile = laspy.file.File(r"ground.las",
                          mode='w', header=inFile.header)
outFile.points = points[ground] # extract ground points, and save it to a las file.
outFile.close() # do not forget this
```
### Installation
Download the source code. under python folder:
```python
python setup.py build
python setup.py install 
```

### How to use CSF in C++
This source code is deveoped under windows and produces DLL, which makes it convenient to be embeded into other applications.
It is very easy to compile since no external library are needed.

Currently, we only export DLL for windows applications. For linux, it is also quite easy, we will do that soon.

For binary release version, it can be downloaded at: http://ramm.bnu.edu.cn/projects/CSF/download/

Note: This code has been changed a lot since the publication of the corresponding paper. A lot of optimizations have been made. We are still working on it, and wish it could be better.

At last, if you are interested in Cloudcompare, there is a good news. our method has been implemented as a Cloudcompare plugin, you can refer to : https://github.com/cloudcompare/trunk

### License
CSF is maintained and developed by Jianbo　QI. It is free software and can be redistributed and modified under the terms of the GNU General Public License (Version 3) as provided by the Free Software Foundation.

