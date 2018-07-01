/*******************************************************************************************************************//**
 * @file
 * @brief
 * @author
 **********************************************************************************************************************/

// Include necessary dependencies
#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"

// Only allow a single command line argument
#define NUM_COMNMAND_LINE_ARGUMENTS 1


/*******************************************************************************************************************//**
 * @brief program entry point
 * @param[in] argc number of command line arguments
 * @param[in] argv string array of command line arguments
 * @return return code (0 for normal termination)
 * @author Shaun Sartin
 **********************************************************************************************************************/
int main(int argc, char **argv)
{
    cv::Mat imageIn;

    // Validate and parse the command line arguments
    if(argc != NUM_COMNMAND_LINE_ARGUMENTS + 1)
    {
        std::printf("USAGE: %s <image_path> \n", argv[0]);
        return 0;
    }
    else
    {
        imageIn = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);

        // Check for file error
        if(!imageIn.data)
        {
            std::cout << "Error while opening file " << argv[1] << std::endl;
            return 0;
        }

        // Store image metadata
        int imageWidth = imageIn.size().width;
        int imageHeight = imageIn.size().height;

        cv::Mat imageGrey;
        cv::cvtColor(imageIn, imageGrey, cv::COLOR_BGR2GRAY);

        // Find the image edges
        cv::Mat imageEdges;
        const double cannyThreshold1 = 100;
        const double cannyThreshold2 = 200;
        const int cannyAperture = 3;
        cv::Canny(imageGrey, imageEdges, cannyThreshold1, cannyThreshold2, cannyAperture);

        // Locate the image contours (after applying a threshold or canny)
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(imageEdges, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

        // compute minimum area bounding rectangles
        std::vector<cv::RotatedRect> minAreaRectangles(contours.size());
        for(int i = 0; i < contours.size(); i++)
        {
            // compute a minimum area bounding rectangle for the contour
            minAreaRectangles[i] = cv::minAreaRect(contours[i]);
        }

        // draw the rectangles
        /*
        cv::Mat imageRectangles = cv::Mat::zeros(imageEdges.size(), CV_8UC3);
        for(int i = 0; i < contours.size(); i++)
        {
            cv::Scalar color = cv::Scalar(rand.uniform(0, 256), rand.uniform(0,256), rand.uniform(0,256));
            cv::Point2f rectanglePoints[4];
            minAreaRectangles[i].points(rectanglePoints);
            for(int j = 0; j < 4; j++)
            {
                cv::line(imageRectangles, rectanglePoints[j], rectanglePoints[(j+1) % 4], color);
            }
        }
        */

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

        // draw the ellipses
        cv::Mat imageEllipse = cv::Mat::zeros(imageEdges.size(), CV_8UC3);
        const int minEllipseInliers = 500;
        for(int i = 0; i < contours.size(); i++)
        {
            // draw any ellipse with sufficient inliers
            if(contours.at(i).size() > minEllipseInliers)
            {
                cv::ellipse(imageEllipse, fittedEllipses[i], cv::Scalar(255,255,255), 2);
            }
        }

    }
    return 0;
}
