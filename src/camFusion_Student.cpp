
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <opencv2/ml/ml.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;

/*
 * Filter Lidar outliers as follows:
 *  - sort all points in X coordinates
 *  - calculate mean of adjacent differences in X axis
 *  - calcualte standard deviation of these difference
 *  - filter out any point that deviates more than standard deviation
 *  - filter out any point that is outside ego lane
 */
static void 
filterLidarOutliers (std::vector<LidarPoint> &lidarPoints) {
    // Vector for adjacent differences of X values
    std::vector<double> xdiffs(lidarPoints.size(), 0);
    // Lidar points without outliers
    std::vector<LidarPoint> filteredLidarPoints;
    double laneWidth = 4.0;

    // Sort all lidar points so that when adding them to their enclosing boxes
    // they are still sorted for later use
    std::sort(std::begin(lidarPoints), std::end(lidarPoints),
            [](const LidarPoint &a, const LidarPoint& b) {
                return a.x < b.x;
            });

/*
    std::adjacent_difference(std::begin(lidarPoints), //first
                             std::end(lidarPoints),   //last
                             std::begin(xdiffs),          //result first
                             [](const LidarPoint& a, const LidarPoint& b) {
                                return cv::norm(a.x - b.x);
                             });
*/


    // Replicate std::adjacent_difference() behavior
    // NOTE: cannot use std::adjacent_difference() since output vector has
    // different type than input
    // See https://en.cppreference.com/w/cpp/algorithm/adjacent_difference
    auto adj_diff = [](std::vector<LidarPoint>::iterator first,
                       std::vector<LidarPoint>::iterator last,
                       std::vector<double>::iterator d_first)
    {
        if (first == last) return d_first;

        double acc = first->x;
        *d_first = acc;
        while (++first != last) {
            double val = first->x;
            *++d_first = val - std::move(acc);
            acc = std::move(val);
        }
        return ++d_first;
    };

    // start a temporary stack frame so that symbols within this frame
    // can be used elsewhere once we are done with current frame
    {
        auto it = std::begin(lidarPoints);
        auto end = std::end(lidarPoints);
        auto diffIt = std::begin(xdiffs);
        adj_diff(it, end, diffIt);
    }

    // first element of xdiffs is unmodified copy of first lidar point X value.
    // Elements starting from xdiffs.being()+1 up to xdiffs.end() has all the
    // diffs we need. So, add them up then divide by the sizeof xdiffs - 1
    double mean = std::accumulate(std::next(std::begin(xdiffs)), //first diff
                                  std::end(xdiffs),     //last
                                  0.0) / (double)(xdiffs.size() - 1);

    double stddev = 0.0;

    for (auto diff = xdiffs.begin()+1; diff != xdiffs.end(); diff++) {
        stddev += pow(*diff - mean, 2);
    }
    stddev = sqrt(stddev / (xdiffs.size() - 1));

    //start a new stack frame for temporary variables
    auto lIt = std::begin(lidarPoints), lEnd = std::prev(std::end(lidarPoints));
    auto diffIt = std::next(std::begin(xdiffs)), diffEnd = std::end(xdiffs);

    for (; lIt != lEnd && diffIt != diffEnd; lIt++, diffIt++) {
        if (abs(lIt->y) > laneWidth / 2.0 ||
            abs(*diffIt - mean) > stddev) {
            continue;
        }

        filteredLidarPoints.push_back(*lIt);
    }

    lidarPoints = filteredLidarPoints;
}

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    filterLidarOutliers(lidarPoints);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }
    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


/*
 * associate a given bounding box with the keypoints it contains
 * If a pair of keypoints (A, B) in current frame matches a pair of keypoints (~A, ~B) in previous frame,
 * A - ~A must be close to B - ~B. One of the ways to eliminate outlier matches is
 * to calculate how much this difference deviates.
 *
 *  - Calculate the mean value of differences
 *  - Calculate the standard deviation
 *  - Choose matches with very low deviation
 */
void clusterKptMatchesWithROI(BoundingBox &boundingBox,
                              std::vector<cv::KeyPoint> &kptsPrev,
                              std::vector<cv::KeyPoint> &kptsCurr,
                              std::vector<cv::DMatch> &kptMatches)
{
    std::vector<std::pair<std::vector<cv::DMatch>::iterator, double>> distPerMatch;

    for (auto matchIt = std::begin(kptMatches); matchIt != std::end(kptMatches); matchIt++) {
        auto &currKP = kptsCurr.at(matchIt->trainIdx);
        auto &prevKP = kptsPrev.at(matchIt->queryIdx);

        if (boundingBox.roi.contains(currKP.pt)) {
            distPerMatch.emplace_back(matchIt, cv::norm(prevKP.pt - currKP.pt));
        }
    }

    double mean = std::accumulate(std::begin(distPerMatch), std::end(distPerMatch), 0.0,
            [](double &a, std::pair<std::vector<cv::DMatch>::iterator, double> &b) {
                return b.second + a; 
            }) / distPerMatch.size();

    double stddev = 0.0;

    for (auto &dm : distPerMatch) {
        stddev += pow(dm.second - mean, 2);
    }
    stddev = sqrt(stddev / distPerMatch.size());

    // ignore any match that deviates more than standard deviation
    for (auto &dm : distPerMatch) {
        if (cv::abs(dm.second - mean) < stddev) {
            boundingBox.kptMatches.push_back(*dm.first);
        }
    }
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
// NOTE: This was done during previous lessons, took the code from Lesson 3
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame

    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            //if (it1 == it2) continue;

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // compute median dist ratio to remove outlier influence
    std::sort(distRatios.begin(), distRatios.end());
    long medIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex];

    double dT = 1 / frameRate;
    TTC = -dT / (1 - medDistRatio);
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr,
                     double frameRate, double &TTC)
{
    // auxiliary variables
    double dT = 1 / frameRate;        // time between two measurements in seconds
    double laneWidth = 4.0; // assumed width of the ego lane

    // Lidar points are already sorted based on X axis; hence, first
    // point in the list is closest
    double minXPrev = 1e9, minXCurr = 1e9;
    if (!lidarPointsPrev.empty()) {
        minXPrev = lidarPointsPrev[0].x;
    }
    if (!lidarPointsCurr.empty()) {
        minXCurr = lidarPointsCurr[0].x;
    }

    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev - minXCurr);
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches,
                        std::map<int, int> &bbBestMatches,
                        DataFrame &prevFrame,
                        DataFrame &currFrame)
{
    /*
     * DMatch::queryIdx -> index of keypoint in previous frame
     * DMatch::trainIdx -> index of keypoint in current frame
     *
     * - For each match, find current and previous keypoints (prevKP, currKP)
     *   - Find bounding boxes containing prevKP in previous frame and save them
     *   - Find bounding boxes containing currKP in current frame and save them
     *   - Consider all possible pairs with saved box ids via a counter
     * - For each box id currID in current frame
     *   - Find most occuring pair with first value currID
     *   - create a pair with these two number and push to bbBestMatches
     */

    // mapping of current and previous frame boxes for counting and finding
    // best match. Use vector instead of multimap for efficiency (speed)
    std::vector<std::pair<int, int>> boxmapping;
    int currBoxCount = currFrame.boundingBoxes.size();
    
    int prevBoxCount = prevFrame.boundingBoxes.size();
    int counter[currBoxCount][prevBoxCount];
    std::memset(counter, 0, sizeof(counter[0][0]) * currBoxCount * prevBoxCount);

    // Store boxID of each BoundBox of current frame
    std::vector<int> currBBIds;
    for (auto& bb : currFrame.boundingBoxes) {
        currBBIds.push_back(bb.boxID);
    }

    for (const auto& match : matches) {

        // Previous and current keypoints respectively
        auto prevKP = prevFrame.keypoints.at(match.queryIdx);
        auto currKP = currFrame.keypoints.at(match.trainIdx);

        // vectors for BoundingBox ids that contain currKP and prevKP respectively
        std::vector<int> currMatchingBoxIds;
        std::vector<int> prevMatchingBoxIds;

        // for each bounding box in current frame, check if currKP is contained
        // if so, add the box id to the corresponding vector
        std::for_each(std::begin(currFrame.boundingBoxes),
                      std::end(currFrame.boundingBoxes),
                      [&](BoundingBox& b) mutable {
                        if (b.roi.contains(currKP.pt)) currMatchingBoxIds.push_back(b.boxID);
                      });

        // same as above, except for previous frame
        std::for_each(std::begin(prevFrame.boundingBoxes),
                      std::end(prevFrame.boundingBoxes),
                      [&](BoundingBox& b) mutable {
                        if (b.roi.contains(prevKP.pt)) prevMatchingBoxIds.push_back(b.boxID);
                      });
        
        // for every box containing the matching keypoints, increment
        // the corresponding counter
        for (auto currID : currMatchingBoxIds) {
            for (auto prevID : prevMatchingBoxIds) {
                counter[currID][prevID] += 1;
            }
        }
    }

    // best match correlates to greatest counter
    for (int i = 0; i < currBoxCount; i++) {
        int maxCount = 0;
        int boxID = -1;
        for (int j = 0; j < prevBoxCount; j++) {
            if (counter[i][j] > maxCount) {
                maxCount = counter[i][j];
                boxID = j;
            }
        }
        // according to main function, key is boxID of previous frame
        // value is boxID of current frame
        // if prev box has already been matched, updated if and only if this one
        // is a better match
        if (bbBestMatches.count(boxID) == 0 ||  maxCount > counter[bbBestMatches[boxID]][boxID]) {
                bbBestMatches[boxID] = i;
        }
    }
}
