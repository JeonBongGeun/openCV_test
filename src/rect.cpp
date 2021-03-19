// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv4/opencv2/tracking/tracker.hpp>
#include <vector>
#include <string>
#include <unistd.h>
#include "cv-helpers.hpp"

using namespace std;
using namespace cv;
using namespace rs2;

bool mouse_is_pressing = false;
int start_x, start_y, end_x, end_y;
int step = 0;

const size_t inWidth = 300;
const size_t inHeight = 300;
const float WHRatio = inWidth / (float)inHeight;
const float inScaleFactor = 0.007843f;
const float meanVal = 127.5;


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
	// Declare RealSense pipeline, encapsulating the actual device and sensors
	pipeline pipe;
	// Start streaming with default recommended configuration
	auto config = pipe.start();
	// What is mean as?? 
	auto profile = config.get_stream(RS2_STREAM_COLOR).as<vidio_stream_profile>();

	// What is mean align??
	rs2::align align_to(RS2_STREAM_COLOR);

	Size cropSize;

	if (profile.width() / (float)profile.height > WHRatio)
	{
		cropSize = Size(static_cast<int>(profile.height() * WHRatio), profile.height());
	}
	else
	{
		cropSize = Size(profile.width(), static_cast<int>(profile.width() / WHRatio));
	}

	Rect crop(Point((profile.width() - cropSize.width) / 2, (profile.height() - cropSize.height) / 2), cropSize);


	const auto window_name = "Display Image";
	namedWindow(window_name, WIDOW_AUTOSIZE);

	// What is the Return value of getWindowProperty function?
	while (getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
	{
		// Wait for the next set of frames;
		auto data = pipe.wait_for_frames();
		// Make sure the frames are spatially alinged
		data = align_to.process(data);

		auto color_frame = data.get_color_frame();
		auto depth_frame = data.get_depth_frame();

		// If we only received new depth framem, but the color did not update, continue
		static int last_frame_number = 0;
		if (color_frame.get_frame_number() == last_frame_number) continue;

		last_frame_number = color_frame.get_frame_number();

		// ----------------------------------------------------------------------------------- //
		
		// Convert Realsense frame to OpenCV matrix.
		// static cv::Mat frame_to_mat( const rs2::frame &f );
		// static cv::Mat depth_frame_to_meters( const rs2::depth_frame &f );
		auto color_mat = frame_to_mat(color_frame);	
		auto depth_mat = depth_frame_to_meters(depth_frame);

		// ----------------------------------------------------------------------------------- //
		
		// Convert Mat to batch of images
		Mat inputBlob = blobFromImage(color_mat, inScaleFactor, Size(inWidth, inHeight), meanVal, false);

		// set the network input
		net.setInput(inputBlob, "data");

		// Compute Output ( N-Dimention )
		Mat detection = net.forward("detection_out");

		// detection.size[2] = detection_cols, detection.size[3] = detection_rows
		Mat detectionMat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());

		// Crop both color and depth frames
		//
		// cv::Mat(Rect &r);
		// Share Memory??
		color_mat = color_mat(crop);
		depth_mat = depth_mat(crop);

		// What is the confidenceThreshold ?
		float confidenceThreshold = 0.8f;

		for (int i = 0; i < detectionMat.rows; i++)
		{
			float confidence = detectionMat.at<float>(i, 2);

			if (confidence > confidenceThreshold)
			{
				size_t objectClass = (size_t)(detectionMat.at<float>(i, 1));

				// Detection Point * Mat Cols, Detection Point * Mat Rows = Detection Rectangle
				int xLeftBottom = static_cast<int>(detectionMat.at<float>(i, 3) * color_mat.cols);
				int yLeftBottom = static_cast<int>(detectionMat.at<float>(i, 4) * color_mat.rows);
				int xRightTop = static_cast<int>(detectionMat.at<float>(i, 5) * color_mat.cols);
				int yRightTop = static_cast<int>(detectionMat.at<float>(i, 6) * color_mat.rows);

				// Detected Object Rectangle.
				Rect object((int)xLeftBottom, (int)yLeftBottom, 
						(int)(xRightTop - xLeftBottom),
						(int)(yRightTop - yLeftBottom));
				
				// Intersection of Union ?
				object = object & Rect(0, 0, depth_mat.cols, depth_mat.rows);

				// Calculate mean depth inside the detection region.
				// This is a very naive way to estimate objects depth
				// but it is intended to demonstrate how one might use depth data in general.

				// Scalar m (B, G, R)
				// B = m[2];
				// G = m[1];
				// R = m[0];
				Scalar m = mean(depth_mat(object));
				/*
				 * static cv::Mat depth_frame_to_meters( const rs2::depth_frame &f )
				 * {
				 * 	cv::Mat dm = frame_to_mat(f);
				 * 	dm.converTo( dm, CV_64F );
				 * 	dm = dm * f.get_units();
				 * 	return dm;
				 * }
				 *
				 */
				// Deteced Label and Covert String
				std::ostringstream ss;
				ss << classNames[objectClass] << " ";
				ss << std::setprecision(2) << m[0] << " meters away";
				String conf(ss.str());

				rectangle(color_mat, object, Scalar(0, 255, 0));
				int baseLine = 0;
				Size labelSize =  getTextSize(ss.str(), FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

				auto center = (object.br() + object.tl()) * 0.5;
				
				center.x = center.x - labelSize.width / 2;

			}
		}

		imshow(window_name, color_mat);


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

