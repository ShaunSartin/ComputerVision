/*******************************************************************************************************************//**
 * @file main.cpp
 * @brief A rudementary implementation of MS Paint
 * @author Shaun Sartin
 **********************************************************************************************************************/

// include necessary dependencies
#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"

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

void paintBucketRecursion(int currX, int currY, cv::Vec3b oldColor, cv::Vec3b newColor, cv::Mat *image, int finishedFlag, int callCount)
{
    callCount = callCount + 1;
    std::cout << "# OF RECURSIVE CALLS: " << callCount << std::endl;

    if(image->at<cv::Vec3b>(cv::Point(currX, currY)) == newColor)
    {
        return;
    }

    image->at<cv::Vec3b>(cv::Point(currX, currY)) = newColor;
    //cv::imshow("imageIn", *image);
    //cv::waitKey(1);

    if(finishedFlag == 1)
    {
        return;
    }

    //check northern neighbor
    if((image->at<cv::Vec3b>(cv::Point(currX, currY-1)) == oldColor) && (currY-1 >= 0))
    {
        paintBucketRecursion(currX, currY-1, oldColor, newColor, image, finishedFlag, callCount);
    }

    //check eastern neighbor
    if((image->at<cv::Vec3b>(cv::Point(currX+1, currY)) == oldColor) && (currX+1 < image->size().width))
    {
        paintBucketRecursion(currX+1, currY, oldColor, newColor, image, finishedFlag, callCount);
    }

    //check southern neighbor
    if((image->at<cv::Vec3b>(cv::Point(currX, currY+1)) == oldColor) && (currY+1 < image->size().height))
    {
        paintBucketRecursion(currX, currY+1, oldColor, newColor, image, finishedFlag, callCount);
    }

    //check western neighbor
    if((image->at<cv::Vec3b>(cv::Point(currX-1, currY)) == oldColor) && (currX-1 >= 0))
    {
        paintBucketRecursion(currX-1, currY, oldColor, newColor, image, finishedFlag, callCount);
    }

    finishedFlag = 1;
    paintBucketRecursion(currX, currY, oldColor, newColor, image, finishedFlag, callCount);

}

/*******************************************************************************************************************//**
 * @brief handler for image click callbacks
 * @param[in] mouse event
 * @param[in] x position of the mouse cursor
 * @param[in] y position of the mouse cursor
 * @param[in] flags string array of command line arguments
 * @param[in] userdata string array of command line arguments
 * @author Shaun Sartin
 **********************************************************************************************************************/
static void clickCallback(int event, int x, int y, int flags, void* imgCommands)
{
    cv::Point point(x, y);
    struct imgCommands_* imgCommands1 = (struct imgCommands_*) imgCommands;

    int b;
    int g;
    int r;

    switch(event)
    {
        case cv::EVENT_LBUTTONDOWN:
            imgCommands1->leftMouseButtonDown = 1;
            if(imgCommands1->commands[imgCommands1->commandIndex] == "Eyedropper")
            {
                b = imgCommands1->image.at<cv::Vec3b>(point)[0];
                g = imgCommands1->image.at<cv::Vec3b>(point)[1];
                r = imgCommands1->image.at<cv::Vec3b>(point)[2];
                imgCommands1->dropperColor = cv::Vec3b(b,g,r);
            }
            else if(imgCommands1->commands[imgCommands1->commandIndex] == "Pencil")
            {
                imgCommands1->image.at<cv::Vec3b>(point) = imgCommands1->dropperColor;
                cv::imshow("imageIn", imgCommands1->image);
                cv::waitKey();
            }
            else if(imgCommands1->commands[imgCommands1->commandIndex] == "Crop")
            {
                imgCommands1->lastPoint = point;
                imgCommands1->lastPointFlag = 1;
            }
            else if(imgCommands1->commands[imgCommands1->commandIndex] == "Paint Bucket")
            {
                b = imgCommands1->image.at<cv::Vec3b>(point)[0];
                g = imgCommands1->image.at<cv::Vec3b>(point)[1];
                r = imgCommands1->image.at<cv::Vec3b>(point)[2];
                cv::Vec3b oldColor(b, g, r);
                paintBucketRecursion(x, y, oldColor, imgCommands1->dropperColor, &imgCommands1->image, 0, 0);
                cv::imshow("imageIn", imgCommands1->image);
                cv::waitKey();
            }
            break;

        case cv::EVENT_RBUTTONDOWN:
            imgCommands1->commandIndex = imgCommands1->commandIndex + 1;
            if(imgCommands1->commandIndex >= 5)
            {
                imgCommands1->commandIndex = 0;
            }
            std::cout << "Current Tool: " << imgCommands1->commands[imgCommands1->commandIndex] << std::endl;
            break;

        case cv::EVENT_MOUSEMOVE:
            if((imgCommands1->leftMouseButtonDown == 1) && (imgCommands1->commands[imgCommands1->commandIndex] == "Pencil"))
            {
                imgCommands1->image.at<cv::Vec3b>(point) = imgCommands1->dropperColor;
                cv::imshow("imageIn", imgCommands1->image);
                cv::waitKey();
            }
            break;

        case cv::EVENT_LBUTTONUP:
            imgCommands1->leftMouseButtonDown = 0;

            if((imgCommands1->lastPointFlag == 1) && (imgCommands1->commands[imgCommands1->commandIndex] == "Crop"))
            {
                cv::Rect cropArea = cv::Rect(imgCommands1->lastPoint, point);
                imgCommands1->image = imgCommands1->image(cropArea);
                cv::imshow("imageIn", imgCommands1->image);
                cv::waitKey();
            }
            break;

        case cv::EVENT_LBUTTONDBLCLK:
            if(imgCommands1->commands[imgCommands1->commandIndex] == "Reset")
            {
                imgCommands1->image = imgCommands1->backupImage.clone();
                cv::imshow("imageIn", imgCommands1->image);
                cv::waitKey();
            }
            break;

        case cv::EVENT_RBUTTONDBLCLK:
            cv::imwrite("OUTPUT.png", imgCommands1->image);
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
    // open the input image and save a backup
    std::string inputFileName = "test.png"; //todo: be sure to change this!

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
