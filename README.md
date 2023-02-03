# CSF Ground Segmentation
This package is a library for segmenting ground and non-ground points from a point cloud, i.e., `csf_seg`. It is different from the `ground_seg` package as the `ground_seg` package works strictly on a scan.

Please note that the source code of this package is a modified version of https://github.com/jianboqi/CSF. (Taken 2021/08/11)
## PCD ground filter
 As a sample usage for `csf_seg` library, a simple program which can separate PCD files containting ground and non-ground points is provided. Additionally, the estimated ground surface can be output by setting `-s` flag as shown below. The default config file for parameters is located at `config/csf_seg.yaml`.
```
$ ./scripts/pcd/pcd_ground_filter.sh [-s] <PCD_FILE> <CONFIG_FILE>
```

Options:
* **-s** : output ground surface to a PCD file

Some basic guildlines for parameter tuning (only those with high impact):
* `rigidness` should be **small** if the terrains are not-flat.
* `cloth_resolution` should be **small** if the map area is small.
* `class_threshold` is the maximum distance of a ground point from the estimated surface.

## Source
Airborne LiDAR filtering method based on Cloth Simulation.
This is the code for the article:

W. Zhang, J. Qi*, P. Wan, H. Wang, D. Xie, X. Wang, and G. Yan, “An Easy-to-Use Airborne LiDAR Data Filtering Method Based on Cloth Simulation,” Remote Sens., vol. 8, no. 6, p. 501, 2016.
(http://www.mdpi.com/2072-4292/8/6/501/htm)
## License
CSF is maintained and developed by Jianbo QI. It is now released under Apache 2.0.

