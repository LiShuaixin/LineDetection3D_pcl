# 3DLineDetection
A simple and efficient 3D line detection algorithm for large scale unorganized point cloud. A conference paper based on this code can be found here https://arxiv.org/abs/1901.02532

This is a modified version that applied PCL to facilitate the incorporation of ROS project. 

Prerequisites:
---
1. OpenCV > 2.4.x
2. OpenMP
3. PCL >= 1.7
3. No other libs

Usage:
---
1. build the project with Cmake
2. run the code
3. The default parameters are useful for general cases without tunning(at least for these cases in the experiences of the paper). However, you can also adjust the parameters if the result is not very good.

Performance:
---
On a computer with Intel Core i5-3550p CPU, the computing time for point clouds with 30M, 20M, 10M, 5M, 2M and 1M points is 130s, 80s, 40s, 20s, 8s and 4s, respectively.
![image](https://github.com/xiaohulugo/images/blob/master/3DLineDetection.jpg)

Feel free to correct my code, if you spotted the mistakes. You are also welcomed to Email me: fangzelu@gmail.com

Notice: Basically, this code works well for dense and accurate point clouds, while poor for noisy ones.
---

Citation:
---
Please cite the following paper if this you feel this code helpful.

```
@article{lu2019fast,
title={Fast 3D Line Segment Detection From Unorganized Point Cloud},
author={Xiaohu, Lu and Yahui, Liu and Kai, Li},
journal={arXiv preprint arXiv:1901.02532},
year={2019},
}
```

TODO:
---
This project is a point-based region growing approach which is slower than the octree-based region growing (OBRG) approches. The OBRG version will be release later
