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

/*
 * Taken from Alireza at:
 * https://stackoverflow.com/questions/28562401/resize-an-image-to-a-square-but-keep-aspect-ratio-c-opencv
 */
cv::Mat resizeKeepAspectRatio(const cv::Mat &input, const cv::Size &dstSize, const cv::Scalar &bgcolor)
{
    cv::Mat output;

    double h1 = dstSize.width * (input.rows/(double)input.cols);
    double w2 = dstSize.height * (input.cols/(double)input.rows);
    if( h1 <= dstSize.height) {
        cv::resize( input, output, cv::Size(dstSize.width, h1));
    } else {
        cv::resize( input, output, cv::Size(w2, dstSize.height));
    }

    int top = (dstSize.height-output.rows) / 2;
    int down = (dstSize.height-output.rows+1) / 2;
    int left = (dstSize.width - output.cols) / 2;
    int right = (dstSize.width - output.cols+1) / 2;

    cv::copyMakeBorder(output, output, top, down, left, right, cv::BORDER_CONSTANT, bgcolor );

    return output;
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
        cv::Mat imageNormalizedResized = resizeKeepAspectRatio(imageNormalized, cv::Size(640,480), cv::Scalar(0,0,0));

        // Find the image edges
        cv::Mat imageEdges;
        const double cannyThreshold1 = 160;
        const double cannyThreshold2 = 200;
        const int cannyAperture = 3;
        cv::Canny(imageNormalized, imageEdges, cannyThreshold1, cannyThreshold2, cannyAperture);
        cv::Mat imageEdgesResized = resizeKeepAspectRatio(imageEdges, cv::Size(640,480), cv::Scalar(0,0,0));

        // Locate the image contours (after applying a threshold or canny)
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(imageEdges, contours, cv::RETR_TREE, cv::CHAIN_APPROX_NONE, cv::Point(0, 0)); //NOTE: changed from CHAIN_APPROX_SIMPLE

        // draw the contours
        cv::Mat imageContours = cv::Mat::zeros(imageEdges.size(), CV_8UC3);
        cv::RNG rand(12345);
        for(int i = 0; i < contours.size(); i++)
        {
            cv::Scalar color = cv::Scalar(rand.uniform(0,256), rand.uniform(0,256), rand.uniform(0,256));
            cv::drawContours(imageContours, contours, i, color);
        }
        cv::Mat imageContoursResized = resizeKeepAspectRatio(imageContours, cv::Size(640,480), cv::Scalar(0,0,0));

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
        cv::Mat imageRectanglesResized = resizeKeepAspectRatio(imageRectangles, cv::Size(640, 480), cv::Scalar(0,0,0));


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

        // store ellipse in a vector
        cv::Mat imageEllipse = cv::Mat::zeros(imageEdges.size(), CV_8UC3);
        const int minEllipseInliers = 280;
        for(int i = 0; i < contours.size(); i++)
        {
            // draw any ellipse with sufficient inliers
            if(contours.at(i).size() > minEllipseInliers)
            {
                cv::Scalar color = cv::Scalar(rand.uniform(0, 256), rand.uniform(0,256), rand.uniform(0,256));
                cv::ellipse(imageEllipse, fittedEllipses[i], color, 2);
            }
        }


        //cv::imshow("imageIn", imageIn);
        cv::imshow("imageNormalized", imageNormalizedResized);
        //cv::imshow("imageEqualized", imageEqualizedResized);
        cv::imshow("imageEdges", imageEdgesResized);
        cv::imshow("imageContours", imageContoursResized);
        cv::imshow("imageRectangles", imageRectanglesResized);
        cv::imshow("imageEllipse", imageEllipse);
        cv::waitKey();

    }
    return 0;
}
