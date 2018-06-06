#include <iostream>
#include <chrono>
#include <librealsense2/rs.hpp>
#include <opencv/cv.hpp>
#include <fstream>
#include <string>
#include <vector>

//TODO: command line arg for num frames and resolution, save benchmark results to file
inline bool prompt_yes_no(const std::string& prompt_msg);
inline uint32_t get_user_selection(const std::string& prompt_msg);



int main() {

    //bool save_img_to_disk = prompt_yes_no("Save Images to Disk? ");
    uint32_t n_buffer = get_user_selection("What Buffer Size? (Recommended: 10): ");
    //bool save_benchmark_to_disk = prompt_yes_no("Save Benchmark to Disk?");

    long save_ms;
    long frame_receipts_ms;
    long frameset_wait_for_receipts_ms;
    long time_taken_to_save[n_buffer-1] = {};
    long time_between_frame_receipts[n_buffer-1] = {};
    long time_taken_to_receive[n_buffer-1] = {};

    int fps_counter = 0; // for counting FPS
    int frame_counter = 0; // for counting how many frames until # of frames user has requested

    // initialize buffer
    std::vector<rs2::points> points_buffer(n_buffer);
    std::vector<rs2::frame> color_buffer(n_buffer);
    //rs2::points points_buffer[10] = {};
    //rs2::frame color_buffer[10] = {};

    // create pipeline and pointcloud
    rs2::pipeline p;
    rs2::pointcloud pc;
    rs2::points points;

    // configure (10 frames / sec)
    rs2::config buff_config;
    buff_config.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_RGB8, 30);
    buff_config.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);

    // start pipeline and keep track of what position is in buffer
    p.start(buff_config);
    int idx = 0;

    std::chrono::system_clock::time_point prev_time = std::chrono::system_clock::now(); // Initialize time of previous frame
    std::chrono::system_clock::time_point fps_time = std::chrono::system_clock::now(); // Initialize time of previous frame

    while(idx < n_buffer) {

        // Wait for frames
        std::chrono::system_clock::time_point start_waiting_for_frames = std::chrono::system_clock::now();
        rs2::frameset frames = p.wait_for_frames();

        std::chrono::system_clock::time_point receive_time = std::chrono::system_clock::now();
        frame_receipts_ms = std::chrono::duration_cast<std::chrono::milliseconds>(receive_time - prev_time).count();
        frameset_wait_for_receipts_ms = std::chrono::duration_cast<std::chrono::milliseconds>(receive_time - start_waiting_for_frames).count();

        std::cout << "Time Taken to Receive:" << frameset_wait_for_receipts_ms << "ms \n";
        std::cout << "Time Between Two Frame Receipts: " << frame_receipts_ms << "ms \n";

        if(frame_counter != 0){
            time_between_frame_receipts[frame_counter-1] = frame_receipts_ms;
            time_taken_to_receive[frame_counter-1] = frameset_wait_for_receipts_ms;
        }

        prev_time = receive_time;

        // extract frames
        auto depth = frames.get_depth_frame();
        auto color = frames.get_color_frame();

        // Extract point cloud
        points = pc.calculate(depth);
        pc.map_to(color);

        // keep filling buffer while it is not full
        points_buffer[idx] = points;
        color_buffer[idx] = color;

        std::chrono::system_clock::time_point extracted_time = std::chrono::system_clock::now();
        std::cout << "Time taken to extract:" << std::chrono::duration_cast<std::chrono::milliseconds>(extracted_time - receive_time).count() << "ms \n";

        // Print out FPS
        if(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - fps_time).count() == 1){
            std::cout << "FPS: " << fps_counter << " -------------------------------------------------------\n";
            fps_counter = 0;
            fps_time = std::chrono::system_clock::now();
        }

        idx++;
        fps_counter++;
        frame_counter ++;
    }

    // stop pipeline and free camera
    p.stop();

    // start saving and emptying buffer
    for(int i=0; i<n_buffer; i++){

        std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();
        //points_buffer[i].export_to_ply("pointcloud.ply", color_buffer[i]);
        points_buffer[i].export_to_ply("../pointcloud" + std::to_string(i) + ".ply", color_buffer[i]);

        std::chrono::system_clock::time_point save_time = std::chrono::system_clock::now();
        save_ms = std::chrono::duration_cast<std::chrono::milliseconds>(save_time - start_time).count();
        std::cout << "Time taken to save:" << save_ms << "ms \n";

        if(frame_counter != 0){
            time_taken_to_save[frame_counter-1] = save_ms;
        }
    }

    return 0;
}

inline bool prompt_yes_no(const std::string& prompt_msg)
{
    char ans;
    do
    {
        std::cout << prompt_msg << "[y/n]: ";
        std::cin >> ans;
        std::cout << std::endl;
    } while (!std::cin.fail() && ans != 'y' && ans != 'n');
    return ans == 'y';
}

inline uint32_t get_user_selection(const std::string& prompt_message)
{
    std::cout << "\n" << prompt_message;
    uint32_t input;
    std::cin >> input;
    std::cout << std::endl;
    return input;
}
