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

**Reading data from txt file:**

If the lidar data is stored in txt file (x y z for each line), it can also be imported directly.

```python
import CSF

csf = CSF.CSF()
csf.readPointsFromFile('samp52.txt')

csf.params.bSloopSmooth = False
csf.params.cloth_resolution = 0.5

ground = CSF.VecInt()  # a list to indicate the index of ground points after calculation
non_ground = CSF.VecInt() # a list to indicate the index of non-ground points after calculation
csf.do_filtering(ground, non_ground) # do actual filtering.
csf.savePoints(ground,"ground.txt")
```

### How to use CSF in Python
Download the source code. under python folder:
```python
python setup.py build
python setup.py install 
```

### How to use CSF in Matlab
see more details from file `demo_mex.m` under matlab folder.

### How to use CSF in R

Thanks to the nice work of @Jean-Romain, through the collaboration, the CSF has been made as a R package, the details can be found in the [RCSF repository](https://github.com/Jean-Romain/RCSF). This package can be used easily with the [lidR package](https://github.com/Jean-Romain/lidR):

```r
library(lidR)
las  <- readLAS("file.las")
las  <- lasground(las, csf())
```

### How to use CSF in C++
Now, CSF is built by CMake, it produces a static library, which can be used by other c++ programs.
#### linux
To build the library, run:
```bash
mkdir build #or other name
cd build
cmake ..
make
sudo make install
```
or if you want to build the library and the demo executable `csfdemo`

```bash
mkdir build #or other name
cd build
cmake -DBUILD_DEMO=ON ..
make
sudo make install

```

#### Windows
You can use CMake GUI to generate visual studio solution file.

### Binary Version
For binary release version, it can be downloaded at: http://ramm.bnu.edu.cn/projects/CSF/download/

Note: This code has been changed a lot since the publication of the corresponding paper. A lot of optimizations have been made. We are still working on it, and wish it could be better.

### Cloudcompare Pulgin
At last, if you are interested in Cloudcompare, there is a good news. our method has been implemented as a Cloudcompare plugin, you can refer to : https://github.com/cloudcompare/trunk

### License
CSF is maintained and developed by Jianbo QI. It is now released under Apache 2.0.

