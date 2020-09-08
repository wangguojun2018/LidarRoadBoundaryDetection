## Speed and Accuracy Tradeoffs for LiDAR Based Road Boundary Detection (IEEE/CAA 2020) [\[paper\]](http://www.ieee-jas.org/article/id/47aa185b-001a-46b9-bff4-ef7e430ab39a?pageType=en)
An speed and accuracy tradeoff method for LiDAR-based road boundary detection in structured environments is proposed. 
The proposed method consists of three main stages: 
1) a multi-feature based method is applied to extract feature points; 
2) a road-segmentation-line-based method is proposed for classifying left and right feature points; 
3) an iterative Gaussian process regression (GPR) is employed for filtering out false points and extracting boundary points. 

To demonstrate the effectiveness of the proposed method, KITTI datasets is used for comprehensive experiments, 
and the performance of our approach is tested under different road conditions. 
Comprehensive experiments show the road-segmentation-line-based method can classify left, 
and right feature points on structured curved roads, and the proposed iterative Gaussian process regression can extract road boundary points on varied road shapes and traffic conditions. Meanwhile, the proposed road boundary detection method can achieve real-time performance with an average of 70.5 ms per frame.


## Demo
[![Demo](https://github.com/wangguojun2018/LidarRoadBoundaryDetection/data/successful_case.png)](https://www.bilibili.com/video/BV12p4y197ms/)

# Introduction
![model](https://github.com/wangguojun2018/LidarRoadBoundaryDetection/data/pipeline.png)  
The proposed method consists of four main steps: **Ground Points Segmentation**,**Feature Points Extraction**,**Feature Points Classification** and **Feature Points Filtering**. It takes a frame of raw point cloud as input and outputs road boundary points.


# Installation
## 1. Install ```proto3```
   Please check [README.md](https://github.com/protocolbuffers/protobuf/blob/master/src/README.md) for ```proto3``` installation instructions.
## 2. Install ```glog``` 
   Please check [INSTALL](https://github.com/google/glog/blob/master/INSTALL) for ```glog``` installation instructions.
## 3. Install ```OpenCV 3.4.5++``` and ```PCL 1.7++```  
## 4 Install Gaussian Processes library ```limbo```   
   1. clone ``limbo``  

```
    cd LidarRoadBoundaryDetection 
    git clone https://github.com/resibots/limbo

```

   2. Build ``limbo``  
Then please check [official documentation](http://www.resibots.eu/limbo/tutorials/compilation.html) to build and intall ```limbo```

## 5. Build This Project

```bash
    cd LidarRoadBoundaryDetection 
    mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j8
```
## 6. Run Video Demo
(Please modify ``binDir`` to your own bin file directory in [boundary_detection.config](https://github.com/wangguojun2018/LidarRoadBoundaryDetection/config/boundary_detection.config))

```bash
    ./demo_video ../config/boundary_detection.config
```
## 7 Run Frame Demo
  
```bash
    ./demo_frame ../config/boundary_detection.config
```



## Citation
If you find this work useful in your research, please consider cite:

```bash
Guojun Wang, Jian Wu, Rui He and Bin Tian, "Speed and Accuracy Tradeoff for LiDAR Data Based Road Boundary Detection," IEEE/CAA J. Autom. Sinica,. 
```

