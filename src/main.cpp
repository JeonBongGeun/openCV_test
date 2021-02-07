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

    CascadeClassifier faceCascade;
    faceCascade.load("C:\\opencv\\sources\\data\\haarcascades\\haarcascade_frontalface_alt.xml");

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
        Mat small_image;
        Mat grayimage;

        resize(image, small_image, Size(200, 200), 0, 0, 1);
        cvtColor(small_image, grayimage, COLOR_BGR2GRAY);

        std::vector<Rect> faces;
        faceCascade.detectMultiScale(grayimage, faces, 1.1, 3, 0, Size(30, 30));

        for (Rect area : faces)
        {
            Scalar drawColor = Scalar(255, 255, 255);
            rectangle(small_image, Point(cvRound(area.x * 1.0), cvRound(area.y * 1.0)), 
                Point(cvRound((area.x + area.width - 1) * 1.0), cvRound((area.y + area.height - 1) * 1.0)), drawColor, 2, 8, 0);
        }

        // Update the window with new data1
        imshow(window_name, small_image);
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
