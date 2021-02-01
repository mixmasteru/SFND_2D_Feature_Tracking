# SFND 2D Feature Tracking


![](images/keypoints.png)

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

## Rubic

+ [x] MP.0 Mid-Term Report
+ [x] MP.1 Data Buffer Optimization
+ [x] MP.2 Keypoint Detection
+ [x] MP.3 Keypoint Removal
+ [x] MP.4 Keypoint Descriptors
+ [x] MP.5 Descriptor Matching
+ [x] MP.6 Descriptor Distance Ratio
+ [x] MP.7 Performance Evaluation 1
+ [x] MP.8 Performance Evaluation 2
+ [x] MP.9 Performance Evaluation 3


## Run and sort output 
```bash
./2D_feature_tracking > output.csv
head -n1 output.csv && tail -n+2 output.csv | sort --field-separator=',' --key=5 
```
## MP.7 Performance Evaluation 1
In the [table](#table) below you will find all available data 
for the detector / descriptor combinations. The column "total_kp" shows the number 
of keypoints collected in the code.

## MP.8 Performance Evaluation 2
In the [table](#table) below you will find all available data
for the detector / descriptor combinations. The column "total_match" shows the number
of matched keypoints for all 10 images collected in the code.

## MP.9 Performance Evaluation 3
The column "total_time" in the [table](#table) below
shows the total time it takes for keypoint detection and descriptor extraction 
collected in the code.

### The TOP3 detector / descriptor combinations are:
* FAST-ORB
* FAST-BRIEF
* FAST-BRISK

These are recommended as the best choice for the purpose of detecting keypoints on vehicles
in this case.

## Results sort by total time 
# <a name="table"></a>
|detectorType|descriptorType|total_kp|total_match|total_time|
|------------|--------------|--------|-----------|----------|
|**FAST**    |**ORB**       |**4094**|**2061**   |**0.0351665** |
|**FAST**    |**BRIEF**     |**4094**|**2258**   |**0.0424373** |
|**FAST**    |**BRISK**     |**4094**|**1832**   |**0.0671354** |
|ORB         |BRIEF         |1150    |421        |0.101114  |
|ORB         |BRISK         |1150    |646        |0.101602  |
|SHITOMASI   |ORB           |1179    |768        |0.125969  |
|SHITOMASI   |BRIEF         |1179    |848        |0.131723  |
|ORB         |ORB           |1150    |525        |0.144936  |
|HARRIS      |ORB           |248     |145        |0.163797  |
|HARRIS      |BRISK         |248     |121        |0.168623  |
|HARRIS      |BRIEF         |248     |155        |0.176734  |
|SHITOMASI   |BRISK         |1179    |690        |0.194257  |
|SHITOMASI   |SIFT          |1179    |927        |0.237416  |
|FAST        |SIFT          |4094    |2782       |0.237756  |
|HARRIS      |SIFT          |248     |163        |0.310063  |
|FAST        |FREAK         |4094    |1566       |0.352091  |
|SHITOMASI   |FREAK         |1179    |574        |0.383104  |
|ORB         |FREAK         |1150    |345        |0.400496  |
|HARRIS      |FREAK         |248     |123        |0.475587  |
|ORB         |SIFT          |1150    |756        |0.495856  |
|AKAZE       |BRIEF         |1655    |1096       |0.706907  |
|AKAZE       |BRISK         |1655    |1101       |0.717027  |
|AKAZE       |ORB           |1655    |913        |0.719172  |
|AKAZE       |SIFT          |1655    |1263       |0.801707  |
|SIFT        |BRISK         |1372    |533        |0.825335  |
|AKAZE       |FREAK         |1655    |962        |0.833809  |
|SIFT        |BRIEF         |1372    |593        |0.997947  |
|AKAZE       |AKAZE         |1655    |1161       |1.29697   |
|SIFT        |FREAK         |1372    |504        |1.31677   |
|SIFT        |SIFT          |1372    |792        |1.33121   |
|BRISK       |BRISK         |2713    |1274       |2.61678   |
|BRISK       |ORB           |2713    |910        |2.89708   |
|BRISK       |FREAK         |2713    |1073       |2.92873   |
|BRISK       |BRIEF         |2713    |1301       |2.98821   |
|BRISK       |SIFT          |2713    |1617       |3.26079   |
|BRISK       |AKAZE         |NA      |NA         |NA        |
|FAST        |AKAZE         |NA      |NA         |NA        |
|HARRIS      |AKAZE         |NA      |NA         |NA        |
|ORB         |AKAZE         |NA      |NA         |NA        |
|SHITOMASI   |AKAZE         |NA      |NA         |NA        |
|SIFT        |AKAZE         |NA      |NA         |NA        |
|SIFT        |ORB           |NA      |NA         |NA        |

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.