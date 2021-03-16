// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <vector>
#include <string>

using namespace std;
using namespace cv;

bool mouse_is_pressing = false;
int start_x, start_y, end_x, end_y;
int step = 0;


void swap(int* v1, int* v2) {
    int temp = *v1;
    *v1 = *v2;
    *v2 = temp;
}


void mouse_callback(int event, int x, int y, int flags, void* userdata)
{
  
    if (event == EVENT_LBUTTONDOWN) {
        step = 1;
        mouse_is_pressing = true;
        start_x = x;
        start_y = y;
    }
    else if (event == EVENT_MOUSEMOVE) {

        if (mouse_is_pressing) {
            end_x = x;
            end_y = y;
            step = 2;
        }
    }
    else if (event == EVENT_LBUTTONUP) {
        mouse_is_pressing = false;
        end_x = x;
        end_y = y;
        step = 3;
    }
}


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

    namedWindow(window_name, WINDOW_AUTOSIZE);
    setMouseCallback(window_name, mouse_callback);

    Rect2d bbox;

    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame color = data.get_color_frame().apply_filter(color_map);
        rs2::depth_frame depth = data.get_depth_frame();

        float width = depth.get_width();
        float height = depth.get_height();
        
        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat image(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

        switch (step)
        {
        case 1:
            circle(image, Point(start_x, start_y), 10, Scalar(0, 255, 0), -1);
            break;

        case 2:
            rectangle(image, Point(start_x, start_y), Point(end_x, end_y), Scalar(0, 255, 0), 3);
            break;

        case 3:
            if (start_x > end_x) {
                swap(&start_x, &end_x);
                swap(&start_y, &end_y);
            }

            Rect2d roi(start_x, start_y, end_x - start_x, end_y - start_y);
            bbox = roi;
            break;
        }
        rectangle(image, bbox, Scalar(255, 0, 0), 2, 3);

        float dist_to_width = bbox.x + (bbox.width / 2);
        float dist_to_height = bbox.y + (bbox.height / 2);

        float dist_to_center = depth.get_distance(dist_to_width, dist_to_height) * 100;

        //printf("%.2f\n", dist_to_center);

        circle(image, Point(dist_to_width, dist_to_height), 3, Scalar(255, 0, 0), 2);

        std::string dist_text(std::to_string(dist_to_center));

        putText(image, dist_text, Point(100, 100), 2, 1.4, Scalar(255, 0, 0), 2);

        imshow(window_name, image);
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
