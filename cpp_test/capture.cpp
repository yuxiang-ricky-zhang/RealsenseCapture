#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp> // Include OpenCV API
#include <iostream>
#include <iomanip>

using namespace cv;

int main(){

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline pipeline;
    rs2::config config;
 // 	config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);
	// config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 15);
    config.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 15);
	config.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 15);


    // Configure and start the pipeline
    // pipeline.start();
    rs2::pipeline_profile profile = pipeline.start(config);

    int frame_count = 0;

    while (frame_count < 100)
    {
        // Block program until frames arrive
        rs2::frameset frames = pipeline.wait_for_frames();

        // Try to get a frame of a depth image
        rs2::depth_frame depth_frame = frames.get_depth_frame();
        rs2::video_frame color_frame = frames.get_color_frame();

  		// const int w = depth_frame.as<rs2::video_frame>().get_width();
		// const int h = depth_frame.as<rs2::video_frame>().get_height();

        // Get the depth frame's dimensions
        int w = depth_frame.get_width();
        int h = depth_frame.get_height();

        // Create OpenCV matrix of size (w,h) from the depth and color data
		Mat depth_image(Size(w, h), CV_16U, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
		Mat color_image(Size(w, h), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

		frame_count++;

		if (frame_count > 50){

			std::ostringstream filename;
			filename << std::setw(5) << std::setfill('0') << frame_count;

			// cv::imwrite("./rgb/"+std::to_string(filename)+".png", color_image);
			// cv::imwrite("./depth/"+std::to_string(filename)+".png", depth_image);
			cv::imwrite("./rgb/"+ filename.str()+".png", color_image);
			cv::imwrite("./depth/"+filename.str()+".png", depth_image);
		}


        // Print the frame count
        std::cout << "frame count: "+ std::to_string(frame_count) << std::endl;
        //std::cout << "timestamp: " + std::to_string(depth_frame.get_timestamp()) << std::endl;
    }

	auto sensor = profile.get_device().first<rs2::depth_sensor>();
	auto scale =  sensor.get_depth_scale();
	std::cout << "depth scale: "+ std::to_string(scale) << std::endl;

    pipeline.stop();

    return 0;
}