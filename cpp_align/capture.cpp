#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp> // Include OpenCV API
#include <fstream>
#include <iostream>
#include <iomanip>
#include <time.h>

using namespace cv;

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);

int main(){

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline pipe;
    rs2::config config;
    config.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 15);
    config.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 15);


    // Configure and start the pipeline
    rs2::pipeline_profile profile = pipe.start(config);
    
    auto color_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto intrinsic_para = color_stream.get_intrinsics();
    std::cout << "fx: "+ std::to_string(intrinsic_para.fx) << std::endl;
    std::cout << "fy: "+ std::to_string(intrinsic_para.fy) << std::endl;
    std::cout << "cx: "+ std::to_string(intrinsic_para.ppx) << std::endl;
    std::cout << "cy: "+ std::to_string(intrinsic_para.ppy) << std::endl;
    std::cout << "k1"+ std::to_string(intrinsic_para.coeffs[1]) << std::endl;
    std::cout << "k2"+ std::to_string(intrinsic_para.coeffs[2]) << std::endl;
    std::cout << "p1"+ std::to_string(intrinsic_para.coeffs[3]) << std::endl;
    std::cout << "p2"+ std::to_string(intrinsic_para.coeffs[4]) << std::endl;
    std::cout << "k3"+ std::to_string(intrinsic_para.coeffs[5]) << std::endl;


    rs2_stream align_to = find_stream_to_align(profile.get_streams());
	
    rs2::align align(align_to);

    std::ofstream outfile;
    outfile.open("./timestamp.txt", std::ios_base::app);

    int frame_count = 0;

    while (frame_count < 100)
    {
        // Block program until frames arrive
        rs2::frameset frameset = pipe.wait_for_frames();

		if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
		{
			profile = pipe.get_active_profile();
			align_to = find_stream_to_align(profile.get_streams());
			align = rs2::align(align_to);
		}

		auto processed = align.process(frameset);
	
        // Try to get a frame of a depth image
        rs2::depth_frame depth_frame = processed.get_depth_frame();
        rs2::video_frame color_frame = processed.get_color_frame();


        // Get the depth frame's dimensions
        int w = depth_frame.get_width();
        int h = depth_frame.get_height();

        // Create OpenCV matrix of size (w,h) from the depth and color data
		Mat depth_image(Size(w, h), CV_16U, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
		Mat color_image(Size(w, h), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

		frame_count++;

		if (frame_count > 50){

			// Save rgb and depth images
			std::ostringstream filename;
			filename << std::setw(5) << std::setfill('0') << frame_count;
			cv::imwrite("./rgb/"+ filename.str()+".png", color_image);
			cv::imwrite("./depth/"+filename.str()+".png", depth_image);


			// Save timestamp

			// Absolute time
			time_t rawtime;
			struct tm * timeinfo;
			char current_time_str [80];
			time (&rawtime);
			timeinfo = localtime (&rawtime);
			strftime (current_time_str,80,"%Y:%m:%d %H:%M:%S",timeinfo);


			// Relative stime from realsense
			std::string depth_time = std::to_string(depth_frame.get_timestamp()); 
			std::string color_time = std::to_string(color_frame.get_timestamp()); 

			std::string data_entry = filename.str()+","+current_time_str+","+depth_time+","+color_time+"\n";
			outfile << data_entry;

		}


        // Print the frame count
        std::cout << "frame count: "+ std::to_string(frame_count) << std::endl;
    }

	auto sensor = profile.get_device().first<rs2::depth_sensor>();
	auto scale =  sensor.get_depth_scale();
	std::cout << "depth scale: "+ std::to_string(scale) << std::endl;

    pipe.stop();

    return 0;
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
    //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    //We prioritize color streams to make the view look better.
    //If color is not available, we take another stream that (other than depth)
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found)         //Prefer color
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR)
            {
                color_stream_found = true;
            }
        }
        else
        {
            depth_stream_found = true;
        }
    }

    if(!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}


bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
    for (auto&& sp : prev)
    {
        //If previous profile is in current (maybe just added another)
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
        if (itr == std::end(current)) //If it previous stream wasn't found in current
        {
            return true;
        }
    }
    return false;
}
