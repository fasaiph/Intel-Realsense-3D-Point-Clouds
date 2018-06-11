#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cmath>
#include <iomanip>
#include <boost/format.hpp>
#include <chrono>
#include "opencv/cv.hpp"

// compile time flags to adjust behavior
#define WARMUP 0
#define TOTAL_SAVE 10

using namespace std::chrono;

using pixel = std::pair<int, int>;

float dist_3d(const rs2_intrinsics& intr, const rs2::depth_frame& frame, pixel u, pixel v);

class VideoFrame{
public:
    int set_data(const void * realsense_data){
        data = realsense_data;
    }

    const void * get_data(void){
        return data;
    }

    int set_stride_in_bytes(int realsense_stride){
        stride = realsense_stride;
    }

    int get_stride_in_bytes(void){
        return stride;
    }
private:
    const void * data;
    int stride;
};

class DepthFrame
{
public:
    int set_width(int realsense_width){
        width = realsense_width;
    }

    int get_width(void){
        return width;
    }

    int set_height(int realsense_height){
        height = realsense_height;
    }

    int get_height(void){
        return height;

    }
    float get_distance(int x, int y){
        return distance[x][y];
    }
private:
    int width;
    int height;
    float distance[height][width];
};

int main(int argc, char * argv[])
{
    if( argc < 2 )
        throw std::runtime_error("need to specify output file");

    std::string output_path = argv[1];

    milliseconds time0;
    milliseconds time1;
    milliseconds time2;
    milliseconds time3;

    std::cout << "Enter main function" << std::endl;

    // initialize buffer
    std::vector<VideoFrame*> color_buffer(TOTAL_SAVE);
    std::vector<DepthFrame*> depth_buffer(TOTAL_SAVE);

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH);
    cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGB8);

    // Start streaming with default recommended configuration
    auto profile = pipe.start(cfg);

    std::cout << "Capture and ignore " << WARMUP << " frames" << std::endl;
    // Capture 30 frames to give autoexposure, etc. a chance to settle
    for (auto i = 0; i < WARMUP; ++i) pipe.wait_for_frames();

    for (auto save_index = 0; save_index < TOTAL_SAVE; save_index++ ) {

        VideoFrame save_video_frame;
        DepthFrame save_depth_frame;

        // Block program until frames arrive
        time0 = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        rs2::frameset frames = pipe.wait_for_frames();
        time1 = duration_cast<milliseconds>(system_clock::now().time_since_epoch());

        rs2::align align_to(RS2_STREAM_DEPTH);

        auto align = rs2::align(align_to);
        auto processed = align.process(frames);
        time2 = duration_cast<milliseconds>(system_clock::now().time_since_epoch());

        // Try to get a frame of a depth image
        rs2::depth_frame depth = processed.get_depth_frame();
        rs2::video_frame rgb = processed.get_color_frame();

        save_video_frame.set_data(rgb.get_data());
        save_video_frame.set_stride_in_bytes(rgb.get_stride_in_bytes());

        save_depth_frame.set_height(depth.get_height());
        save_depth_frame.set_width(depth.get_width());

        // Store frames in buffer
        depth_buffer[save_index] = &save_depth_frame;
        color_buffer[save_index] = &save_video_frame;

        std::cout << "  depth=" << depth.get_width() << "x" << depth.get_height() << "  rgb=" << rgb.get_width() << "x"
        << rgb.get_height() << std::endl;
    }

    auto stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto intrinsics = stream.get_intrinsics(); // Calibration data

    for (auto save_index = 0; save_index < TOTAL_SAVE; save_index++ ){

        boost::basic_format<char> output_file = boost::format("%s%04d.txt") % output_path % save_index;
        std::cout << "Saving Frame to " << output_file.str() << std::endl;

        std::ofstream myfile;
        myfile.open(output_file.str());
        myfile << "# Realsense RGB + XYZ output\n";

       // std::cout << depth_buffer[save_index] << "\n";

        // Get the depth frame's dimensions
     //   auto width = depth_buffer[save_index] -> get_width();
     //   auto height = depth_buffer[save_index] -> get_height();
//        auto width = depth.get_width();
//        auto height = depth.get_height();


        std::cout << "Got Stride: " << color_buffer[save_index] -> get_stride_in_bytes() <<"\n";
        myfile << "resolution " << depth_buffer[save_index] -> get_width() << " " << depth_buffer[save_index] -> get_height() << std::endl;

        float point3d[3];
        float pixel2d[2];

        // Query the distance from the camera to the object in the center of the image
        const auto *rgb_row = (const uint8_t *) color_buffer[save_index] -> get_data();
        auto stride = color_buffer[save_index] -> get_stride_in_bytes();
        for (int row = 0; row < depth_buffer[save_index] -> get_height(); row++, rgb_row += stride) {

            auto rgb_ptr = rgb_row;

            for (int col = 0; col < depth_buffer[save_index] -> get_width(); col++) {

                pixel2d[0] = col;
                pixel2d[1] = row;

                auto dist = depth_buffer[save_index] -> get_distance(col, row);
                rs2_deproject_pixel_to_point(point3d, &intrinsics, pixel2d, dist);

                int r = *rgb_ptr++;
                int g = *rgb_ptr++;
                int b = *rgb_ptr++;

                myfile << r << " " << g << " " << b << " " << point3d[0] << " " << point3d[1] << " " << point3d[2]
                << std::endl;
            }
        }

        myfile.close();
        time3 = duration_cast<milliseconds >( system_clock::now().time_since_epoch() );
        std::cout << "Saved!  wait=" << (time1-time0).count() << " align="<< (time2-time1).count() << " save=" << (time3-time2).count() << std::endl;
    }
    std::cout << "Closing" << std::endl;

    return EXIT_SUCCESS;
}

float dist_3d(const rs2_intrinsics& intr, const rs2::depth_frame& frame, pixel u, pixel v)
{
    float upixel[2]; // From pixel
    float upoint[3]; // From point (in 3D)

    float vpixel[2]; // To pixel
    float vpoint[3]; // To point (in 3D)

    // Copy pixels into the arrays (to match rsutil signatures)
    upixel[0] = u.first;
    upixel[1] = u.second;
    vpixel[0] = v.first;
    vpixel[1] = v.second;

    // Query the frame for distance
    // Note: this can be optimized
    // It is not recommended to issue an API call for each pixel
    // (since the compiler can't inline these)
    // However, in this example it is not one of the bottlenecks
    auto udist = frame.get_distance(upixel[0], upixel[1]);
    auto vdist = frame.get_distance(vpixel[0], vpixel[1]);

    // Deproject from pixel to point in 3D
    rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);
    rs2_deproject_pixel_to_point(vpoint, &intr, vpixel, vdist);

    // Calculate euclidean distance between the two points
    return sqrt(pow(upoint[0] - vpoint[0], 2) +
                pow(upoint[1] - vpoint[1], 2) +
                pow(upoint[2] - vpoint[2], 2));
}