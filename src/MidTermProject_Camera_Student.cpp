/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <deque>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"


using cv::xfeatures2d::BriefDescriptorExtractor;
using cv::xfeatures2d::SURF;
using cv::xfeatures2d::SIFT;
using cv::xfeatures2d::DAISY;
using cv::xfeatures2d::FREAK;

using namespace std;

void run(string detectorType, string descriptorType) {

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
    deque<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results
    bool info = false;
    if (info) cout << detectorType << " - " << descriptorType << endl;

    int total_kp = 0;
    int total_match = 0;
    float total_time = 0;

    /* MAIN LOOP OVER ALL IMAGES */
    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++) {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        dataBuffer.push_back(frame);
        if (dataBuffer.size() > dataBufferSize) {
            dataBuffer.pop_front();
        }

        //// EOF STUDENT ASSIGNMENT
        if (info) cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image


        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        if (detectorType == "SHITOMASI") {
            total_time += detKeypointsShiTomasi(keypoints, imgGray, bVis);
        }
        if (detectorType == "HARRIS") {
            total_time += detKeypointsHarris(keypoints, imgGray, bVis);
        } else {
            total_time += detKeypointsModern(keypoints, imgGray, detectorType, bVis);
        }

        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        if (bFocusOnVehicle) {
            auto rm = remove_if(keypoints.begin(), keypoints.end(),
                                [](cv::KeyPoint p) { return !p.pt.inside(cv::Rect(535, 180, 180, 150)); });
            keypoints.erase(rm, keypoints.end());
        }

        total_kp += keypoints.size();

        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts) {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0) {
                // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            if (info) cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        if (info) cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        total_time += descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors,
                                    descriptorType);
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        if (info) cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
            //string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
            string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType, matcherType, selectorType);

            total_match += matches.size();

            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            if (info) cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
            if (bVis) {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
                /*
                vector<float> vec;
                float max = 0.0;
                for(auto p: keypoints){
                    vec.push_back(p.size);
                    if(p.size > max){
                        max = p.size;
                    }
                }
                cv::Mat mat = cv::Mat(1, vec.size(), CV_32F, &vec[0]);
                int nHistSize = 256;
                float fRange[] = { 0.0f, max };
                const float* fHistRange = { fRange };

                cv::Mat matHist;
                cv::calcHist(&mat, 1, 0, cv::Mat(), matHist, 1, &nHistSize, &fHistRange);
                int hist_w = 512, hist_h = 400;
                int bin_w = cvRound( (double) hist_w/nHistSize );
                cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
                normalize(matHist, matHist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
                for( int i = 1; i < nHistSize; i++ )
                {
                    line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(matHist.at<float>(i-1)) ),
                          cv::Point( bin_w*(i), hist_h - cvRound(matHist.at<float>(i)) ),
                          cv::Scalar( 255, 0, 0), 2, 8, 0  );
                }

                string windowName1 = "hist";
                cv::namedWindow(windowName1, 7);
                cv::imshow(windowName1, histImage);
                */
                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                if (info) cout << "Press key to continue to next image" << endl;
                //cv::waitKey(0); // wait for key to be pressed

            }
        }

    } // eof loop over all images
    cout << detectorType << "," << descriptorType << "," << total_kp << "," << total_match << "," << total_time << endl;
}

/* MAIN PROGRAM */
int main(int argc, const char *argv[]) {
    string descriptorTypes[6] = {"BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};
    string detectorTypes[7] = {"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};

    cout << "detectorType" << "," << "descriptorType" << "," << "total_kp" << "," << "total_match" << "," << "total_time" << endl;
    for (const auto &detectorType: detectorTypes) {
        for (const auto &descriptorType: descriptorTypes) {
            try {
                run(detectorType, descriptorType);
            }
            catch (const std::exception &e) {
                //std::cout << e.what();
                cout << detectorType << "," << descriptorType << "," << "NA" << "," << "NA" << "," << "NA" << endl;
            }
        }
    }

    return 0;
}
