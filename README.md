# HOW to track features in consecutive images

## Data Buffer

We use a ring buffer that implements a std::vector of specified max size. After adding an element, if vector's size is higher than max_size then we delete the first element in vector.

## Keypoints Detection (OpenCV)
Keypoints detection is possible using one of the following algorithm:
- SHITOMASI
- HARRIS
- FAST
- BRISK
- ORB
- AKAZE
- SIFT

## Descriptor extraction and matching (OpenCV)
Descriptor extraction based on detected keypoints is done with one the following algorithm:
- BRISK
- BRIEF
- FREAK
- ORB (not usable with SIFT detector)
- AKAZE (Only usable with AKAZE detector)
- SIFT

Matching descriptors between consecutives images is possible choosing one of the following algorithm:
- Brute Force (crosscheck is possible)
- FLANN

On the matched descriptors, selection is computed through one of the following algorithm:
- Nearest Neighbor (best match chosen)
- K - Nearest Neighbor (with K =2) combined with a distance ratio comparaison (thold = 0.8) to remove noisy matches.

## Performance Evaluation
We evaluated three parameters:
- Number of keypoints detected (Task MP7)
- Number of matched keypoints for all combination of detector and descriptor (Task MP8)
- Computation time from detection to matching for all combination. (Task MP9)

We want to know what is the best combination of detector and descriptor to find a maximum number of keypoints in a minimum time.

Based on attached results, the top three combination of detector/descriptor is the following:
- 1 -   FAST detector + BRIEF descriptor. 35,8 matches per image in 2 ms in average.
- 2 -   FAST detector + ORB descriptor. 35,2 matches per image in 4 ms in average.
- 3.1 - BRISK detector + BRIEF descriptor. 28,2 matches per image found in 45 ms in average.
- 3.2 - ORB dectector + BRIEF descriptor. 23,9 matches per image found in 12,5 ms in average.

Those results do not focus on keypoints accuracy.


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
