# 1. DefSLAM
**Authors:** [Jose Lamarca](http://webdiis.unizar.es/~jlamarca/), Shaifali Parashar, Adrien Bartoli and [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/).\\
Forked from [the original repo.](https://github.com/UZ-SLAMLab/DefSLAM)

DefSLAM is a real-time deformable SLAM library for **Monocular** cameras that computes the camera trajectory and a sparse 3D reconstruction in a deformable environment.

# 2. Prerequisites

* [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface.
* [OpenCV](http://opencv.org) to manipulate images and features. **Required 4.0.0**.
* [Eigen](http://eigen.tuxfamily.org) **Required at least 3.1.0**.
* [Ceres](http://ceres-solver.org) to optimize warp and to run the NRSfM.
* [PCL](https://pointclouds.org/downloads/) mainly for groundtruths.

## DBoW2 and g2o and ORBSLAM (included in Thirdparty folder)
* Modified [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition
* Modified [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations.
* Modified version of ORBSLAM.
* Modified BBS is only used for ORBSLAM_2.

# 5. Datasets
## Mandala dataset
Download the sequences with the link: 
[Mandala Dataset](https://drive.google.com/file/d/1i3i2f3Ph22DfZ6AfXKjPRb8WrGNw_41C/view?usp=sharing)

It contains the five sequences presented in the paper. The mandala deformation becomes more challenging with the number of dataset.

The dataset is organized as follows:
```
--Mandala(i)
   -- images
   -- yalm file
   -- times
```
## Hamlyn dataset  
1. Download the dataset from the webpage [Hamlyn](http://hamlyn.doc.ic.ac.uk/vision/). 

2. To downloaded in the format run in our paper [Hamlyn not official](https://drive.google.com/file/d/1fG6RNQlBqwDKZSRUuiEuoMbVosMo5u8J/view?usp=sharing)


## Run Example
Examples of scripts in Apps/rungt.sh

1. Process a sequence. 
```
./DefSLAM <ORBfile> <yaml.file> <ORBfile> <image folder>
```
If you run it without <image folder>. It takes the camera in 0. 

2. Sequences with depth image for ground truth. (Used for CT phantom dataset)
```
./DefSLAMCTGT <ORBfile> <yaml.file> <video.avi> <folder with pattern>
```

3. Sequences with stereo for ground truth.
```
./DefSLAMGT <ORBfile> <yaml.file> <left_image folder> <right_image folder> <timestamps file>
```

Script to generate this folders with this format avaiable. Ask me through <jlamarca@unizar.es>

## Code Organization
```
-- Root folder
  -- Apps. Folder with executables.

  -- Vocabulary. Folder that contains the vocabulary for the BoW.

  -- ThirdParty
    -- BBS. Bspline library adapted for C++ with eigen.
    -- DBoW2. Bag Of word library only used for ORBSLAM_2. Loop closin and relocalization.
    -- g2o. Optimization library for deformation tracking and pose estimation.
    -- ORBSLAM_2. Base clases of the rigid SLAM library modified to work in deforming 
		  environments.

  -- Modules. DefSLAM modules, it includes all the modules needed for the library.
    -- Common. It includes the keyframes, map, map points and Systems.
    -- Mapping.
    -- GroundTruth
    -- Matching
    -- Settings. Check advance settings.
    -- Template. Template tools
    -- ToolsPCL. Tools from PCL library used mainly for the ground truth.
    -- Tracking. 
    -- Viewer.
```
## Advance settings for and ORBSLAM:
To repeat the experiments and save the results you can activate the flag Viewer.SaveResults in the yalm file. The system is running by default in parallel. To change it, you must change the file set_MAC.h and comment the line #define PARALLEL.

To run ORBSLAM, you can uncomment the line 5 in set_MAC.h.

The file CC_MAC.h contain the parameters for the cross correlation matching used in the grundtruth. It is adapted for a better performance in the Mandala dataset, but it should be adapted for each dataset.

To set the number of nodes of the mesh (Initially 10x10), change line 63 and 64 of Template/TriangularMesh.cc

To set the number of nodes for the BBSpline Thirdparty/BBS/bbs_MAC.h
