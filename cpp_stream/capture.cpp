#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp> // Include OpenCV API
#include <iostream>
#include <iomanip>

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

    rs2_stream align_to = find_stream_to_align(profile.get_streams());
	
    rs2::align align(align_to);
    
    std::cout << "Now Streaming.." << std::endl;

    int frame_count = 0;

    while (frame_count < 1000000)
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

			std::ostringstream filename;
			filename << std::setw(5) << std::setfill('0') << frame_count;

			// cv::imwrite("./rgb/"+std::to_string(filename)+".png", color_image);
			// cv::imwrite("./depth/"+std::to_string(filename)+".png", depth_image);
			cv::imwrite("./rgb/"+ filename.str()+".png", color_image);
			cv::imwrite("./depth/"+filename.str()+".png", depth_image);
		}


        // Print the frame count at checkpoints
	if (frame_count % 500 == 0)
	{
        	std::cout << "frame count: "+ std::to_string(frame_count) << std::endl;
	}
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
