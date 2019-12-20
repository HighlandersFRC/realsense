
// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rs_advanced_mode.hpp>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/imgproc/types_c.h>
#include "opencv2/highgui/highgui.hpp"  
#include <iostream>
#include <vector>
#include <windows.h>
#include <dos.h> //for delay

//#include "opencv2/core.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include <iostream>
//#include <stdio.h>
//#include <stdlib.h>

using namespace cv;
using namespace std;


RNG rng(12345);
enum class direction
{
	to_depth,
	to_color
};



int main(int argc, char* argv[]) try
{
	rs2::context                          ctx;        // Create librealsense context for managing devices
	rs2::sensor		color_sensor;
	rs2::sensor		depth_sensor;



	for (auto&& dev : ctx.query_devices())
	{
		std::cout << "Found device " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
		// Given a device, we can query its sensors using:
		std::vector<rs2::sensor> sensors = dev.query_sensors();

		std::cout << "Device consists of " << sensors.size() << " sensors:\n" << std::endl;
		int index = 0;
		// We can now iterate the sensors and print their names
		for (rs2::sensor sensor : sensors)
		{
			if (index == 1)
			{
				color_sensor = sensor;
				std::cout << "Set color_sensor" << std::endl;
			}
			if (index == 0)
			{
				depth_sensor = sensor;
				std::cout << "Set color_sensor" << std::endl;
			}

			std::cout << "  " << index++ << " : " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
			//	get_sensor_name(sensor) << std::endl;
		}

		
	}

	// Declare depth colorizer for pretty visualization of depth data
	rs2::colorizer color_map;
	//rs2::sensor color_sensor;
	color_sensor.set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
	color_sensor.set_option(rs2_option::RS2_OPTION_EXPOSURE, 5); // 1/10 ms (10)
	auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
	depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
	
	depth_sensor.set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;
	// Start streaming with default recommended configuration
	rs2::config cfg;

	rs2::pipeline_profile selection = pipe.start();



	// ...
	//frameset = align_to_depth.process(frameset);

	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);

	rs2::align align_to_depth(RS2_STREAM_DEPTH);
	rs2::align align_to_color(RS2_STREAM_COLOR);
	//direction   dir = direction::to_depth;  // Alignment direction
	using namespace cv;
	const auto window_name = "Display Image";
	namedWindow(window_name, WINDOW_AUTOSIZE);
	std::vector < std::vector < cv::Point > >contours;


	while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
	{
		
		
		rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
		data = align_to_color.process(data);

		rs2::frame color = data.get_color_frame();
		rs2::depth_frame depth = data.get_depth_frame();
	
		

		// Query frame size (width and height)
		const int w = color.as<rs2::video_frame>().get_width();
		const int h = color.as<rs2::video_frame>().get_height();

		float width = depth.get_width();
		float height = depth.get_height();
		// Create OpenCV matrix of size (w,h) from the colorized depth data
		Mat image(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP), image2, image3, image4, image5, image6;

		Mat element = getStructuringElement(MORPH_CROSS,
			Size(2 * .5 + 1, 2 * .5 + 1),
			Point(.5, .5));


		//while (true)
		//{

			cvtColor(image, image2, COLOR_BGR2HSV);

			inRange(image2, Scalar(15, 215, 55), Scalar(65, 255, 125), image3);
	
			erode(image3, image4, element);

			dilate(image4, image5, element);




			
	
			// find moments of the image
			
			Moments m = moments(image5, true);
			Point p(m.m10 / m.m00, m.m01 / m.m00);

			// coordinates of centroid
			//cout << Mat(p) << endl;

			// show the image with a point mark at the centroid
			circle(image, p, 5, Scalar(200, 200, 200), -1);



			cv::findContours(image5, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
			cv::drawContours(image, contours, -1, cv::Scalar(0, 0, 255), 2);

			float dist_to_center = depth.get_distance((p.x), p.y);
			std::cout << "The target is " << (dist_to_center*39.3701) << " inches away \r";
			



			/*
			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;

			// detect edges using canny
			//Canny(image5, image6, 50, 150, 3);

			// find contours
			cv::findContours(image5, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

			// get the moments
			vector<Moments> mu(contours.size());
			for (int i = 0; i < contours.size(); i++)
			{
				mu[i] = moments(contours[i], false);
			}
			
			// get the centroid of figures.
			vector<Point2f> mc(contours.size());
			for (int i = 0; i < contours.size(); i++)
			{
				mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
			}
			

			// draw contours
			
			for (int i = 0; i < contours.size(); i++)
			{
				Scalar color = Scalar(167, 151, 0); // B G R values
				cv::drawContours(image, contours, i, color, 2, 8, hierarchy, 0, Point());
				circle(image, mc[i], 4, color, -1, 8, 0);
			}
			*/
			imshow(window_name, image);




	}
	//std::cout << "huh" << std::endl;
	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	//std::cout << "bad" << std::endl;
	return EXIT_FAILURE;
}
