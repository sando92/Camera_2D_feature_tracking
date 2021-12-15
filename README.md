# HOW to track features in consecutive images

## MP1. Data Buffer Optimization

We use a ring buffer that implements a std::vector of specified max size. After adding an element, if vector's size is higher than max_size then we delete the first element in vector.
Code below:
```
dataBuffer.push_back(frame);
if (dataBuffer.size() > dataBufferSize) {
    dataBuffer.erase(dataBuffer.begin());
}
```

## MP2. Keypoints Detection

Keypoints detection is possible using one of the following algorithm:
- SHITOMASI
- HARRIS
- FAST
- BRISK
- ORB
- AKAZE
- SIFT

The implementation is done using the OpenCV library. You can select a detector with the variable ```detectorType```.

## MP3. Keypoints removal

We define a rectangle representing the area of the preceding vehicle:
```
cv::Rect vehicleRect(535, 180, 180, 150);
```
Using the ```contains``` method of ```cv::Rect```, we can determine if a specific ```cv::Point``` is inside the defined rectangle. Any keypoint outside of the preceding vehicle area is removed.


## MP4. Descriptor extraction

Descriptor extraction based on detected keypoints is done with one the following algorithm:
- BRISK
- BRIEF
- FREAK
- ORB (not usable with SIFT detector)
- AKAZE (Only usable with AKAZE detector)
- SIFT

The implementation is done using the OpenCV library. Variable ``` descriptorType ``` must be set accordingly.

## MP5. Descriptor Matching

Matching descriptors between consecutives images is possible choosing one of the following algorithm:
- Brute Force (crosscheck is possible)
- FLANN

Variable ```matcherType``` must be set accordingly (```MAT_BF``` or ```MAT_FLANN```).
The variable descriptorType is reused to specify the class of the descriptor (```DES_HOG``` or ```DES_BINARY```) at matcher creation. Only SIFT descriptor belongs to Histogram of Oriented Gradients, all others are Binary descriptors.

## MP6. Matches Selection, Descriptor Distance Ratio

On the matched descriptors, selection is computed through one of the following algorithm:
- Nearest Neighbor (best match chosen)
- K - Nearest Neighbor (with K =2) combined with a distance ratio comparaison (thold = 0.8) to remove noisy matches. 

Explanation on KNN selection and descriptor distance ratio calculation:
A keypoint in an image can not have more than one match in the next image. So, the 2 best matches must be significantly different, because at least one of them is wrong. If the best match is more than 20% (for ratio threshold at 0.8) closer than the second best match, then it is supposed enough distinguishable from noise and thus chosen like a good match.

## MP7-8-9. Performance Evaluation
We evaluate three parameters:
- Number of keypoints detected in preceding vehicle area (Task MP7)
- Number of matched keypoints for all combination of detector and descriptor (Task MP8)
- Computation time from detection to matching for all combination. (Task MP9)

We want to know what is the best combination of detector and descriptor to find a maximum number of keypoints quickly as possible.

Based on attached results (see ```performance_data`` folder), the top three combination of detector/descriptor is the following:
- 1 -   FAST detector + BRIEF descriptor. 35,8 matches per image in 2 ms in average.
- 2 -   FAST detector + ORB descriptor. 35,2 matches per image in 4 ms in average.
- 3.1 - BRISK detector + BRIEF descriptor. 28,2 matches per image found in 45 ms in average.
- 3.2 - ORB dectector + BRIEF descriptor. 23,9 matches per image found in 12,5 ms in average.

Those results do not focus on keypoints accuracy. 
In the ```performance_data``` folder, you will find three pdf files, named MP7, MP8 and MP9. Each one gathers data to corresponding rubric points.


# Dependencies

- [OpenCV](https://docs.opencv.org/4.x/df/d65/tutorial_table_of_content_introduction.html)

# Compile and run

Retrieve the source code:
```
git clone https://github.com/sando92/camera_2D_feature_tracking.git
```

Create the build directory at root and enter it:
```
cd camera_2D_feature_tracking
mkdir build
cd build
```

Compile the project:
```
cmake .. && make
```

And finally run it:
```
./2D_feature_tracking
```
