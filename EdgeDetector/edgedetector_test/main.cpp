#include <stdio.h>
#include "opencv/cv.hpp"

using namespace cv;

int main(int argc, char** argv )
{
    // Check if correct usage
    if ( argc != 2 )
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }

    // Initialize image matrix
    Mat image;
    Mat gray_image;
    Mat canny_contours;

    image = imread( argv[1], 1 );


    // Check if image data exists
    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }

    // Convert to grayscale and blur
    cvtColor(image, gray_image, CV_RGB2GRAY);
    blur(gray_image, gray_image, Size(3,3));

    // Get canny contours
    Canny(gray_image, canny_contours, 10,350);

    // Display Original Image
    namedWindow("Original Image", WINDOW_AUTOSIZE );
    imshow("Original Image", image);

    // Display Contours
    namedWindow("Canny Contours", WINDOW_AUTOSIZE);
    imshow("Canny Contours", canny_contours);

    waitKey(0);

    return 0;
}