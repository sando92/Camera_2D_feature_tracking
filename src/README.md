# HOW to track features in consecutive images

## Data Buffer

Ring buffer using a std::vector of specified max size. After adding an element, if vector's size is higher than max_size then we delete the first element in vector.

## Keypoints Detection
Keypoints detection is possible using one of the following algorithm:
- SHITOMASI
- HARRIS
- FAST
- BRISK
- ORB
- AKAZE
- SIFT

## Descriptor extraction and matching
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
- Number of matched keypoints for all combinaison of detector and descriptor (Task MP8)
- Computation time from detection to matching for all combinaison. (Task MP9)

Based on attached results, the top three combinaison of detector/descriptor is the following:
1 - FAST detector + BRIEF descriptor. 35,8 matches per image in 2 ms in average.
2 - FAST detector + ORB descriptor. 35,2 matches per image in 4 ms in average.
3.1 - BRISK detector + BRIEF descriptor. 28,2 matches per image found in 45 ms in average.
3.2 - ORB dectector + BRIEF descriptor. 23,9 matches per image found in 12,5 ms in average.

Those results do not focus on keypoints accuracy.