/*******************************************************************************************************************//**
 * @file ComputerVision_HW1.cpp
 * @brief A rudementary implementation of MS Paint
 * @author Shaun Sartin
 **********************************************************************************************************************/

// include necessary dependencies
#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"

//store necessary information for image manipulation
struct imgCommands_
{

    std::string commands[5] = {"Eyedropper", "Crop", "Pencil", "Paint Bucket", "Reset"};
    int commandIndex;
    cv::Vec3b dropperColor;
    cv::Mat image;
    cv::Mat backupImage;
    cv::Point lastPoint;
    int lastPointFlag;
    int leftMouseButtonDown;
};

/*******************************************************************************************************************//**
 * @brief recursive function to use paint-bucket tool over contiguous same-color areas
 * @param[in] currX: the x position of the pixel being currently examined
 * @param[in] currY: the y position of the pixel being currently examined
 * @param[in] oldColor: the color of the starting pixel before it was clicked
 * @param[in] newColor: the replacement color
 * @param[in] image: the memory address of the image
 * @param[in] finishedFlag: a marker to finish recursion (true when all 4-neighbor pixels are finished being analyzed), 1 -> true, 0 -> false
 * @author Shaun Sartin
 **********************************************************************************************************************/
void paintBucketRecursion(int currX, int currY, cv::Vec3b oldColor, cv::Vec3b newColor, cv::Mat* image, int finishedFlag)
{

    if(image->at<cv::Vec3b>(cv::Point(currX, currY)) == newColor)
    {
        return;
    }

    image->at<cv::Vec3b>(cv::Point(currX, currY)) = newColor;

    if(finishedFlag == 1)
    {
        return;
    }

    //check northern neighbor
    if((image->at<cv::Vec3b>(cv::Point(currX, currY-1)) == oldColor) && (currY-1 >= 0))
    {
        paintBucketRecursion(currX, currY-1, oldColor, newColor, image, finishedFlag);
    }

    //check eastern neighbor
    if((image->at<cv::Vec3b>(cv::Point(currX+1, currY)) == oldColor) && (currX+1 < image->size().width))
    {
        paintBucketRecursion(currX+1, currY, oldColor, newColor, image, finishedFlag);
    }

    //check southern neighbor
    if((image->at<cv::Vec3b>(cv::Point(currX, currY+1)) == oldColor) && (currY+1 < image->size().height))
    {
        paintBucketRecursion(currX, currY+1, oldColor, newColor, image, finishedFlag);
    }

    //check western neighbor
    if((image->at<cv::Vec3b>(cv::Point(currX-1, currY)) == oldColor) && (currX-1 >= 0))
    {
        paintBucketRecursion(currX-1, currY, oldColor, newColor, image, finishedFlag);
    }

    finishedFlag = 1;
    paintBucketRecursion(currX, currY, oldColor, newColor, image, finishedFlag);

}

/*******************************************************************************************************************//**
 * @brief handler for image click callbacks
 * @param[in] event: mouse event
 * @param[in] x: x position of the mouse cursor
 * @param[in] y: y position of the mouse cursor
 * @param[in] flags: unused parameter
 * @param[in] imgCommands: a void* cast structure containing all the necessary elements for the image manipulation
 * @author Shaun Sartin
 **********************************************************************************************************************/
static void clickCallback(int event, int x, int y, int flags, void* imgManipulation)
{
    cv::Point point(x, y);
    struct imgCommands_* imgCommands = (struct imgCommands_*) imgManipulation;

    int b;
    int g;
    int r;

    switch(event)
    {
        case cv::EVENT_LBUTTONDOWN:
            imgCommands->leftMouseButtonDown = 1;
            if(imgCommands->commands[imgCommands->commandIndex] == "Eyedropper")
            {
                b = imgCommands->image.at<cv::Vec3b>(point)[0];
                g = imgCommands->image.at<cv::Vec3b>(point)[1];
                r = imgCommands->image.at<cv::Vec3b>(point)[2];
                imgCommands->dropperColor = cv::Vec3b(b,g,r);
            }
            else if(imgCommands->commands[imgCommands->commandIndex] == "Pencil")
            {
                imgCommands->image.at<cv::Vec3b>(point) = imgCommands->dropperColor;
                cv::imshow("imageIn", imgCommands->image);
                cv::waitKey();
            }
            else if(imgCommands->commands[imgCommands->commandIndex] == "Crop")
            {
                imgCommands->lastPoint = point;
                imgCommands->lastPointFlag = 1;
            }
            else if(imgCommands->commands[imgCommands->commandIndex] == "Paint Bucket")
            {
                b = imgCommands->image.at<cv::Vec3b>(point)[0];
                g = imgCommands->image.at<cv::Vec3b>(point)[1];
                r = imgCommands->image.at<cv::Vec3b>(point)[2];
                cv::Vec3b oldColor(b, g, r);
                paintBucketRecursion(x, y, oldColor, imgCommands->dropperColor, &imgCommands->image, 0);
                cv::imshow("imageIn", imgCommands->image);
                cv::waitKey();
            }
            break;

        case cv::EVENT_RBUTTONDOWN:
            imgCommands->commandIndex = imgCommands->commandIndex + 1;
            if(imgCommands->commandIndex >= 5)
            {
                imgCommands->commandIndex = 0;
            }
            std::cout << "Current Tool: " << imgCommands->commands[imgCommands->commandIndex] << std::endl;
            break;

        case cv::EVENT_MOUSEMOVE:
            if((imgCommands->leftMouseButtonDown == 1) && (imgCommands->commands[imgCommands->commandIndex] == "Pencil"))
            {
                imgCommands->image.at<cv::Vec3b>(point) = imgCommands->dropperColor;
                cv::imshow("imageIn", imgCommands->image);
                cv::waitKey();
            }
            break;

        case cv::EVENT_LBUTTONUP:
            imgCommands->leftMouseButtonDown = 0;

            if((imgCommands->lastPointFlag == 1) && (imgCommands->commands[imgCommands->commandIndex] == "Crop"))
            {
                cv::Rect cropArea = cv::Rect(imgCommands->lastPoint, point);
                imgCommands->image = imgCommands->image(cropArea);
                cv::imshow("imageIn", imgCommands->image);
                cv::waitKey();
            }
            break;

        case cv::EVENT_LBUTTONDBLCLK:
            if(imgCommands->commands[imgCommands->commandIndex] == "Reset")
            {
                imgCommands->image = imgCommands->backupImage.clone();
                cv::imshow("imageIn", imgCommands->image);
                cv::waitKey();
            }
            break;
    }
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
    std::string inputFileName;

    if(argc > 1)
    {
        inputFileName = argv[1];
    }
    else
    {
        std::cout << "Error: You must pass argument for an image filename" << std::endl;
        return 0;
    }

    imgCommands_ imgCommands;

    imgCommands.commandIndex = 0;
    imgCommands.dropperColor = cv::Vec3b(255,255,255);
    imgCommands.image = cv::imread(inputFileName, CV_LOAD_IMAGE_COLOR);
    imgCommands.backupImage = imgCommands.image.clone();
    imgCommands.lastPointFlag = 0;
    imgCommands.leftMouseButtonDown = 0;

	// check for file error
	if(!imgCommands.image.data)
	{
		std::cout << "Error while opening file " << inputFileName << std::endl;
		return 0;
	}

    // display the input image
	cv::imshow("imageIn", imgCommands.image);
	cv::setMouseCallback("imageIn", clickCallback, (void*) &imgCommands);
	std::cout << "Current Tool: " << imgCommands.commands[0] << std::endl;
	cv::waitKey();
}
