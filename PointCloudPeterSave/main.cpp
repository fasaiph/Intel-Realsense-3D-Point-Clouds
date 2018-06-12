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
#define WARMUP 10
#define TOTAL_SAVE 50

using namespace std::chrono;

using pixel = std::pair<int, int>;

float dist_3d(const rs2_intrinsics& intr, const rs2::depth_frame& frame, pixel u, pixel v);

class VideoFrame{
public:
    VideoFrame( int width , int height , int stride, int num_bands  ) {
        this->width = width;
        this->height = height;
        this->stride = stride;
        this->num_bands = num_bands;
        this->data = new uint8_t[  stride*height*num_bands ];
    }

    // TODO: look up c++ destructor
    ~VideoFrame() {
        delete this->data;
        this->data = NULL;
    }

    void set_image(uint8_t* realsense_data) {
        // do sanity check to see if DATA's width, height ... etc match's this objects

        // copy into data
       memcpy(data, realsense_data, (size_t)stride*height*sizeof(uint8_t));
    }

public:
    int width, height, stride, num_bands;
    uint8_t* data;
};


class DepthFrame
{
public:
    DepthFrame( int width , int height) {
        this->width = width;
        this->height = height;
        this->xyz = new float[ width*height*3 ];
    }

    ~DepthFrame(){
        delete this->xyz;
        this->xyz = NULL;
    }

    void set_cloud(int row, int col, float x, float y, float z) {
        if(col > width || row > height){
            std::cout << "Row or Col exceeds specified dimensions" << std::endl;
        }

        // if a pixel is invalid set xyz to -1 -1 -1
        xyz[(col+row*width)*3] = x;
        xyz[(col+row*width)*3+1] = y;
        xyz[(col+row*width)*3+2] = z;
    }

public:
    int width;
    int height;
    float *xyz;
};


int main(int argc, char * argv[]) {
    if (argc < 2)
        throw std::runtime_error("need to specify output file");

    std::string output_path = argv[1];

    milliseconds time0;
    milliseconds time1;
    milliseconds time2;
    milliseconds time3;

    std::cout << "Enter main function" << std::endl;

    // initialize buffer
    std::vector<VideoFrame *> color_buffer(TOTAL_SAVE);
    std::vector<DepthFrame *> depth_buffer(TOTAL_SAVE);
    std::cout << "vector size: " << color_buffer.size() << std::endl;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    rs2::config cfg; //valid fps: 6, 15, 30 at full resolution
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_ANY, 6);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 6);

    // Start streaming with default recommended configuration
    auto profile = pipe.start(cfg);

    std::cout << "Capture and ignore " << WARMUP << " frames" << std::endl;
    // Capture 30 frames to give autoexposure, etc. a chance to settle
    for (auto i = 0; i < WARMUP; ++i) pipe.wait_for_frames();

    int fps_counter = 0;
    std::chrono::system_clock::time_point fps_time = std::chrono::system_clock::now();
    std::chrono::system_clock::time_point start_capture = std::chrono::system_clock::now();

// Capturing and processing ----------------------------------------------------------------------------------------------------
    for (auto save_index = 0; save_index < TOTAL_SAVE; save_index++) {
        // Print out FPS
        if(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - fps_time).count() == 1){
            std::cout << "FPS: " << fps_counter << " -------------------------------------------------------\n";
            fps_counter = 0;
            fps_time = std::chrono::system_clock::now();
        }
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

        // Initialize save objects
        VideoFrame *save_video_frame = new VideoFrame
                (rgb.get_width(), rgb.get_height(), rgb.get_stride_in_bytes(), rgb.get_bytes_per_pixel());
        DepthFrame *save_depth_frame = new DepthFrame(depth.get_width(), depth.get_height());

        // Copy RGB data to object
        save_video_frame->set_image((uint8_t *) rgb.get_data());

        auto width = depth.get_width();
        auto height = depth.get_height();

        std::cout << "Frame " << save_index << std::endl;
        std::cout << "  depth=" << width << "x" << height << "  rgb=" << rgb.get_width() << "x"
        << rgb.get_height() << std::endl;

        // Get x, y, z
        float point3d[3];
        float pixel2d[2];

        auto stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        auto intrinsics = stream.get_intrinsics(); // Calibration data

        for (int row = 0; row < height; row++) {

            for (int col = 0; col < width; col++) {
                pixel2d[0] = col;
                pixel2d[1] = row;

                auto dist = depth.get_distance(col, row);
                rs2_deproject_pixel_to_point(point3d, &intrinsics, pixel2d, dist);

                save_depth_frame->set_cloud(row, col, point3d[0], point3d[1], point3d[2]);
            }
        }

        // Save objects to buffer
        color_buffer.at(save_index) = save_video_frame;
        depth_buffer.at(save_index) = save_depth_frame;

        fps_counter++;
    }

    std::chrono::system_clock::time_point end_capture = std::chrono::system_clock::now();
    std::cout << "Total Capture Time:" << std::chrono::duration_cast<std::chrono::seconds>(end_capture - start_capture).count() << "s \n";

// Saving -------------------------------------------------------------------------------------------------------------------
    for (auto save_index = 0; save_index < TOTAL_SAVE; save_index++) {

        boost::basic_format<char> output_file = boost::format("%s%04d.txt") % output_path % save_index;
        std::cout << "Saving Frame to " << output_file.str() << std::endl;

        std::ofstream myfile;
        myfile.open(output_file.str());
        myfile << "# Realsense RGB + XYZ output\n";

        std::cout << "Got Stride: " << color_buffer[save_index]->stride << "\n";
        myfile << "resolution " << depth_buffer[save_index]->width << " " << depth_buffer[save_index]->height <<
        std::endl;

        auto height = depth_buffer[save_index] -> height;
        auto width = depth_buffer[save_index] -> width;

        // Query the distance from the camera to the object in the center of the image
        const auto *rgb_row = (const uint8_t *) color_buffer[save_index]->data;
        auto stride = color_buffer[save_index]->stride;

        for (int row = 0; row < height; row++, rgb_row += stride) {
            auto rgb_ptr = rgb_row;

            for (int col = 0; col < width; col++) {
                float x = depth_buffer[save_index] -> xyz[(col+row*width)*3];
                float y = depth_buffer[save_index] -> xyz[(col+row*width)*3+1];
                float z = depth_buffer[save_index] -> xyz[(col+row*width)*3+2];

                int r = *rgb_ptr++;
                int g = *rgb_ptr++;
                int b = *rgb_ptr++;
                // myfile << x << " " << y << " " << z << std::endl;
                myfile << r << " " << g << " " << b << " " << x << " " << y << " " << z << std::endl;
            }
        }

        myfile.close();
        time3 = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        std::cout << "Saved!  wait=" << (time1 - time0).count() << " align=" << (time2 - time1).count() <<
        " save=" << (time3 - time2).count() << std::endl;
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