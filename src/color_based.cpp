// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <vector>
#include <string>

using namespace cv;

typedef struct NCC
{
    float r_m;
    float g_m;
    float r_s;
    float g_s;
}NCC;

NCC extract_color(const Mat &roi);

int main(int argc, char* argv[]) try
{
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();

    const auto window_name = "Display Image";

    int key = 0; 
    int event = 0;

    namedWindow(window_name, WINDOW_AUTOSIZE);

    while (getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
       
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame color = data.get_color_frame().apply_filter(color_map);
        rs2::depth_frame depth = data.get_depth_frame();

        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat image(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
        rectangle(image, Rect(Point((w  / 2) - 50, (h  / 2) - 50), Point((w / 2) + 50, (h / 2) + 50)), Scalar(0, 0, 255), 1, 8, 0);
        Mat extract_zone = image(Rect(Point((w / 2) - 50, (h / 2) - 50), Point((w / 2) + 50, (h / 2) + 50)));
        Mat convert;
        Mat result_img;

        MatND histo;

        
        if (event == 'h')
        {
            cvtColor(extract_zone, convert, COLOR_BGR2RGB);
            histo = extract_color(convert);

            for (int i = 0; i < 256; i++)
            {
                std::cout << "Value : " << i << " = " << histo.at<float>(i) << std::endl;

            }
            event = 0;
        }
       
        imshow(window_name, image);
       
        key = waitKey(1);

        if (key != -1)
        {
            event = key;
        }
        if (key == 27)
        {
            
            break;
        }
      
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

NCC extract_color(const Mat& roi)
{
    /*
    MatND histogramR, histogramG, histogramB;

    int channel_R[] = { 0 };    
    int channel_G[] = { 1 };
    //int channel_B[] = { 2 };

    float channel_range[2] = { 0.0, 255.0 };
    const float* channel_ranges[1] = { channel_range };
    int histSize[1] = { 256 };

    calcHist(&roi, 1, channel_R, Mat(), histogramR, 1, histSize, channel_ranges);
    calcHist(&roi, 1, channel_G, Mat(), histogramG, 1, histSize, channel_ranges);
    calcHist(&roi, 1, channel_B, Mat(), histogramB, 1, histSize, channel_ranges);
    */
    for (int i = 0; i < roi.rows; i++)
    {
        for (int j = 0; j < roi.cols; j++)
        {

        }
    }

    

}