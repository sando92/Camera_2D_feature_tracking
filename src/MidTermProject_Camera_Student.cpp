/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    
    // Create log table
    vector<std::vector<double>> v_log; // time, # kp, # kp vehicle, mean, var, std dev of neighbourhood size, time desc, time match, # matched kp, 
    std::vector<double> v;
    for (int i=0; i < 9; i++) {
        v_log.push_back(v);
    }
    /// Create log file
    std::ofstream log_file;
    log_file.open ("log.txt");

    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        log_file << "--------- Image n°" << imgIndex << " ---------" << endl;

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        dataBuffer.push_back(frame);
        if (dataBuffer.size() > dataBufferSize) {
            dataBuffer.erase(dataBuffer.begin());
        }

        cout << "#1 : LOAD IMAGE INTO BUFFER done, buffer size is " << dataBuffer.size() << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        string detectorType = "FAST"; // SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray, log_file, v_log, false);
        }
        else if (detectorType.compare("HARRIS") == 0)
        {
            detKeypointsHarris(keypoints, imgGray, log_file, v_log,  false);
        }
        else 
        {
            detKeypointsModern(keypoints, imgGray, detectorType, log_file, v_log,  false);
        }

        // only keep keypoints on the preceding vehicle, this is an approximation
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150); // cx = 535, cy = 180, w = 180, h = 150
        if (bFocusOnVehicle)
        {
            for (int j = 0; j < keypoints.size(); j++) {
                if (!vehicleRect.contains(keypoints[j].pt)) {
                    keypoints.erase(keypoints.begin() + j);
                    j--;
                }
            }
            log_file << "Found " << keypoints.size() << " points in vehicle area." << endl;
            v_log[2].push_back(keypoints.size());
        }

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = true;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;

        // Variance of keypoints size
        double mean_kp_size = 0;
        for (cv::KeyPoint kp: keypoints) {
            mean_kp_size += kp.size;
        }
        mean_kp_size = mean_kp_size / keypoints.size();

        double sum_squared_dist_to_mean = 0;
        for (cv::KeyPoint kp: keypoints) {
            float dist = kp.size - mean_kp_size;
            sum_squared_dist_to_mean += dist * dist;
        }
        double var_neighbourhood_size = sum_squared_dist_to_mean / keypoints.size();

        log_file << " Mean of keypoint size " << mean_kp_size << ", variance " << var_neighbourhood_size << endl;
        v_log[3].push_back(mean_kp_size);
        v_log[4].push_back(var_neighbourhood_size);
        v_log[5].push_back(std::sqrt(var_neighbourhood_size));
        
        cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */
        cv::Mat descriptors;
        string descriptorType = "BRIEF"; // BRISK, BRIEF, FREAK, ORB, AKAZE // SIFT
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType, log_file, v_log);

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
            if (descriptorType == "SIFT") {
                string descriptorType = "DES_HOG";
            } else {
                string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
            }
            string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType, matcherType, selectorType, log_file, v_log);

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
        }

    } // eof loop over all images

    // TODO change it to csv file creation
    for (std::vector<double> vv: v_log) {
        log_file << endl;
        log_file << endl;
        for (double dd: vv) {
            log_file << dd << " + ";
        }
    }

    log_file.close();
    return 0;
}