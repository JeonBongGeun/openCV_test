// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <vector>

int main(int argc, char* argv[]) try
{
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();

    using namespace cv;
    const auto window_name = "Display Image";

    int lowH = 0;
    int highH = 179;

    int lowS = 200;
    int highS = 255;

    int lowV = 102;
    int highV = 255;


    Mat hsvImg;
    Mat threshImg;

    namedWindow(window_name, WINDOW_AUTOSIZE);

    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame depth = data.get_color_frame().apply_filter(color_map);

        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat image(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);



        cvtColor(image, hsvImg, COLOR_BGR2HSV);      // Convert Original Image to HSV Thresh Image

        inRange(hsvImg, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), threshImg);

        GaussianBlur(threshImg, threshImg, cv::Size(3, 3), 0);   //Blur Effect
        dilate(threshImg, threshImg, 0);        // Dilate Filter Effect
        erode(threshImg, threshImg, 0);         // Erode Filter Effect

        createTrackbar("LowH", window_name, &lowH, 179); //Hue (0 - 179)
        createTrackbar("HighH", window_name, &highH, 179);

        createTrackbar("LowS", window_name, &lowS, 255); //Saturation (0 - 255)
        createTrackbar("HighS", window_name, &highS, 255);

        createTrackbar("LowV", window_name, &lowV, 255); //Value (0 - 255)
        createTrackbar("HighV", window_name, &highV, 255);


        // Update the window with new data1
        imshow(window_name, threshImg);
        imshow("original_image", image);
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
