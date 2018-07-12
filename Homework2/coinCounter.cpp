/*******************************************************************************************************************//**
 * @file coinCounter.cpp
 * @brief detect coins in an image, draw boundaries, and print the total amount in dollars
 * @author Shaun Sartin
 **********************************************************************************************************************/

// Include necessary dependencies
#include <iostream>
#include <string>
#include <math.h>
#include <iomanip>
#include "opencv2/opencv.hpp"

// Only allow a single command line argument
#define NUM_COMNMAND_LINE_ARGUMENTS 1

#define PENNY_COLOR cv::Scalar(0,0,255)
#define DIME_COLOR cv::Scalar(255,0,0)
#define NICKEL_COLOR cv::Scalar(0,255,255)
#define QUARTER_COLOR cv::Scalar(0,255,0)

/*******************************************************************************************************************//**
 * @brief calculates Euclidean distance between two points
 * @param[in] point1 the first Point2f
 * @param[in] point2 the second Point2f
 * @return Euclidean distance as a float
 * @author Shaun Sartin
 **********************************************************************************************************************/
float euclidDistance(cv::Point2f pt1, cv::Point2f pt2)
{
    float x1 = pt1.x;
    float y1 = pt1.y;
    float x2 = pt2.x;
    float y2 = pt2.y;

    float x = x1 - x2;
    float y = y1 - y2;

    return sqrt(pow(x,2) + pow(y,2));
}

/*******************************************************************************************************************//**
 * @brief used to sort RotatedRects based on their area (from smallest to greatest)
 * @param[in] rect1 first RotatedRect
 * @param[in] rect2 second RotatedRect
 * @return true if rect1's area is smaller than rect2's area, otherwise return false
 * @author Shaun Sartin
 **********************************************************************************************************************/
bool coinAreaSort(cv::RotatedRect rect1, cv::RotatedRect rect2)
{
    return rect1.size.area() < rect2.size.area();
}

/*******************************************************************************************************************//**
 * @brief program entry point
 * @param[in] argc number of command line arguments
 * @param[in] argv string array of command line arguments
 * @return return code (0 for normal termination)
 * @author Shaun Sartin
 **********************************************************************************************************************/
int main(int argc, char **argv)
{
    // Validate and parse the command line arguments
    if(argc != NUM_COMNMAND_LINE_ARGUMENTS + 1)
    {
        std::printf("USAGE: %s <image_path> \n", argv[0]);
        return 0;
    }
    else
    {
        cv::Mat imageIn;
        imageIn = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);

        // Check for file error
        if(!imageIn.data)
        {
            std::cout << "Error while opening file " << argv[1] << std::endl;
            return 0;
        }

        cv::resize(imageIn, imageIn, cv::Size(640,480));

        // Convert to greyscale
        cv::Mat imageGrey;
        cv::cvtColor(imageIn, imageGrey, cv::COLOR_BGR2GRAY);

        // Normalize greyscale image
        cv::Mat imageNormalized;
        cv::normalize(imageGrey, imageNormalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);

        // Find the image edges
        cv::Mat imageEdges;
        const double cannyThreshold1 = 160;
        const double cannyThreshold2 = 200;
        const int cannyAperture = 3;
        cv::Canny(imageNormalized, imageEdges, cannyThreshold1, cannyThreshold2, cannyAperture);

        // Locate the image contours (after applying a threshold or canny)
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(imageEdges, contours, cv::RETR_TREE, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));

        // draw the contours
        cv::Mat imageContours = cv::Mat::zeros(imageEdges.size(), CV_8UC3);
        cv::RNG rand(12345);
        for(int i = 0; i < contours.size(); i++)
        {
            cv::Scalar color = cv::Scalar(rand.uniform(0,256), rand.uniform(0,256), rand.uniform(0,256));
            cv::drawContours(imageContours, contours, i, color);
        }

        // compute minimum area bounding rectangles
        std::vector<cv::RotatedRect> minAreaRectangles(contours.size());
        for(int i = 0; i <contours.size(); i++)
        {
            // compute a minimum area bounding rectangle for the contour
            minAreaRectangles[i] = cv::minAreaRect(contours[i]);
        }

        // draw the rectangles
        cv::Mat imageRectangles = cv::Mat::zeros(imageEdges.size(), CV_8UC3);
        for(int i = 0; i < contours.size(); i++)
        {
            cv::Scalar color = cv::Scalar(rand.uniform(0, 256), rand.uniform(0, 256), rand.uniform(0, 256));
            cv::Point2f rectanglePoints[4];
            minAreaRectangles[i].points(rectanglePoints);
            for(int j = 0; j < 4; j++)
            {
                cv::line(imageRectangles, rectanglePoints[j], rectanglePoints[(j+1) % 4], color);
            }
        }

        // fit ellipses to contours containing sufficient inliers
        std::vector<cv::RotatedRect> fittedEllipses(contours.size());
        for(int i = 0; i < contours.size(); i++)
        {
            // compute an ellipse only if the contour has more than 5 points (the minimum for ellipse fitting)
            if(contours.at(i).size() > 5)
            {
                fittedEllipses[i] = cv::fitEllipse(contours[i]);
            }
        }

        // store ellipses in a vector
        std::vector<cv::RotatedRect> coinCandidates;
        cv::Mat imageEllipse = cv::Mat::zeros(imageEdges.size(), CV_8UC3);
        const int minEllipseInliers = 125;
        for(int i = 0; i < contours.size(); i++)
        {
            // draw any ellipse with sufficient inliers and add to candidate array
            if(contours.at(i).size() > minEllipseInliers)
            {
                cv::Scalar color = cv::Scalar(rand.uniform(0, 256), rand.uniform(0,256), rand.uniform(0,256));
                cv::ellipse(imageEllipse, fittedEllipses[i], color, 2);
                coinCandidates.push_back(fittedEllipses[i]);
            }
        }

        // check to see if any ellispes are essentially the same
        float minCenterDistance = 10;
        for(int i = 0; i < coinCandidates.size(); i++)
        {
            for(int j = 0; j < coinCandidates.size(); j++)
            {
                // ensure that an ellipse isn't compared with itself
                if(i != j)
                {
                    // compare centers of the RotatedRects
                    // if euclidean distance is too small, assume they wrap the same coin and remove one
                    if(euclidDistance(coinCandidates[i].center, coinCandidates[j].center) < minCenterDistance)
                    {
                        auto iter = coinCandidates.begin() + i;
                        coinCandidates.erase(iter);
                    }
                }
            }
        }

        // sort RotatedRects by their area (from smallest to greatest)
        std::sort(coinCandidates.begin(), coinCandidates.end(), coinAreaSort);

        // draw candidates for coins
        cv::Mat imageCoins = imageIn.clone();
        float avgDimeArea = 0;
        float avgPennyArea = 0;
        float avgNickelArea = 0;
        float avgQuarterArea = 0;
        float sameCoinMinSizeDistance = 250;
        std::vector<float> dimeSizes;
        std::vector<float> pennySizes;
        std::vector<float> nickelSizes;
        std::vector<float> quarterSizes;
        for(int i = 0; i < coinCandidates.size(); i++)
        {
            float currCoinSize = coinCandidates[i].size.area();

            // We know there must be at least 1 dime, so the coin with the smallest area, a.k.a the first coin, should be a dime
            if(avgDimeArea == 0)
            {
                avgDimeArea = currCoinSize;
                dimeSizes.push_back(currCoinSize);
                cv::ellipse(imageCoins, coinCandidates[0], DIME_COLOR, 2);
                continue;
            }
            else if(currCoinSize < (avgDimeArea + sameCoinMinSizeDistance))
            {
                dimeSizes.push_back(currCoinSize);
                avgDimeArea = accumulate(dimeSizes.begin(), dimeSizes.end(), 0.0) / dimeSizes.size();
                cv::ellipse(imageCoins, coinCandidates[i], DIME_COLOR, 2);
                continue;
            }

            // check for pennies
            if(avgPennyArea == 0)
            {
                avgPennyArea = currCoinSize;
                pennySizes.push_back(currCoinSize);
                cv::ellipse(imageCoins, coinCandidates[i], PENNY_COLOR, 2);
                continue;
            }
            else if((currCoinSize < (avgPennyArea + sameCoinMinSizeDistance)) && (currCoinSize > (avgPennyArea - sameCoinMinSizeDistance)))
            {
                pennySizes.push_back(currCoinSize);
                avgPennyArea = accumulate(pennySizes.begin(), pennySizes.end(), 0.0) / pennySizes.size();
                cv::ellipse(imageCoins, coinCandidates[i], PENNY_COLOR, 2);
                continue;
            }

            // check for nickels
            if(avgNickelArea == 0)
            {
                avgNickelArea = currCoinSize;
                nickelSizes.push_back(currCoinSize);
                cv::ellipse(imageCoins, coinCandidates[i], NICKEL_COLOR, 2);
                continue;
            }
            else if((currCoinSize > (avgNickelArea - sameCoinMinSizeDistance)) && (currCoinSize < (avgNickelArea + sameCoinMinSizeDistance)))
            {
                nickelSizes.push_back(currCoinSize);
                avgNickelArea = accumulate(nickelSizes.begin(), nickelSizes.end(), 0.0) / nickelSizes.size();
                cv::ellipse(imageCoins, coinCandidates[i], NICKEL_COLOR, 2);
                continue;
            }

            // check for quarters
            if(avgQuarterArea == 0)
            {
                avgQuarterArea = coinCandidates[i].size.area();
                quarterSizes.push_back(currCoinSize);
                cv::ellipse(imageCoins, coinCandidates[i], QUARTER_COLOR, 2);
                continue;
            }
            else if(currCoinSize > (avgQuarterArea - sameCoinMinSizeDistance))
            {
                quarterSizes.push_back(currCoinSize);
                avgQuarterArea = accumulate(quarterSizes.begin(), quarterSizes.end(), 0.0) / quarterSizes.size();
                cv::ellipse(imageCoins, coinCandidates[i], QUARTER_COLOR, 2);
                continue;
            }

        }

        //compute the amount of money in the image
        std::cout << "Penny - " << pennySizes.size() << std::endl;
        std::cout << "Nickel - " << nickelSizes.size() << std::endl;
        std::cout << "Dime - " << dimeSizes.size() << std::endl;
        std::cout << "Quarter - " << quarterSizes.size() << std::endl;

        std::cout << std::fixed;
        std::cout << std::setprecision(2);
        std::cout << "Total: $" << (0.01 * pennySizes.size()) + (0.05 * nickelSizes.size()) + (0.10 * dimeSizes.size()) + (0.25 * quarterSizes.size()) << std::endl;

        cv::imshow("imageIn", imageIn);
        cv::imshow("imageCoins", imageCoins);
        cv::waitKey();

    }
    return 0;
}
