#include <iostream>
#include <chrono>
#include <librealsense2/rs.hpp>
#include <opencv/cv.hpp>
#include <fstream>
//#include <algorithm>
//#include "../../librealsense/examples/example.hpp"          // Include short list of convenience functions for rendering

// Helper functions
//void register_glfw_callbacks(window& app, glfw_state& app_state);
inline bool prompt_yes_no(const std::string& prompt_msg);
inline uint32_t get_user_selection(const std::string& prompt_msg);


// TODO: command line arguments, namespace, save extract time to csv too

int main() {

    bool save_img_to_disk = prompt_yes_no("Save Images to Disk? ");
    uint32_t num_frames = get_user_selection("How Many Frames to Grab? (Recommended: 120): ");
    bool save_benchmark_to_disk = prompt_yes_no("Save Benchmark to Disk?");
    //TODO: Choose resolution / Output for average benchmark ms per resolution

    // Store benchmark numbers to csv (We don't save the very first one)
    long save_ms;
    long frame_receipts_ms;
    long frameset_wait_for_receipts_ms;
    long time_taken_to_save[num_frames-1] = {};
    long time_between_frame_receipts[num_frames-1] = {};
    long time_taken_to_receive[num_frames-1] = {};

    // Create Pipeline
    rs2::pipeline p;

    // Create Pointcloud
    rs2::pointcloud pc;
    rs2::points points;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cv_config;

    //Add desired streams to configuration
    //cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    //cv_config.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
    cv_config.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_RGB8, 30);
    cv_config.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    p.start(cv_config); // pipeline started (ignore editor error)

    int fps_counter = 0; // for counting FPS
    int frame_counter = 0; // for counting how many frames until # of frames user has requested

    std::chrono::system_clock::time_point prev_time = std::chrono::system_clock::now(); // Initialize time of previous frame
    std::chrono::system_clock::time_point fps_time = std::chrono::system_clock::now(); // Initialize time of previous frame


    while(frame_counter < num_frames){
        std::chrono::system_clock::time_point start_waiting_for_frames = std::chrono::system_clock::now();
        rs2::frameset frames = p.wait_for_frames();

        std::chrono::system_clock::time_point receive_time = std::chrono::system_clock::now();
        frame_receipts_ms = std::chrono::duration_cast<std::chrono::milliseconds>(receive_time - prev_time).count();
        frameset_wait_for_receipts_ms = std::chrono::duration_cast<std::chrono::milliseconds>(receive_time - start_waiting_for_frames).count();
        std::cout << "Time Taken to Receive:" << frameset_wait_for_receipts_ms << "ms \n";
        std::cout << "Time Between Two Frame Receipts: " << frame_receipts_ms << "ms \n";
        //std::cout << "Number of Frames: " << frames.size() << " frames \n"

        if(frame_counter != 0){
            time_between_frame_receipts[frame_counter-1] = frame_receipts_ms;
            time_taken_to_receive[frame_counter-1] = frameset_wait_for_receipts_ms;
        }

        prev_time = receive_time;


        auto depth = frames.get_depth_frame();
        auto color = frames.get_color_frame();

        // Extract point cloud
        points = pc.calculate(depth);
        pc.map_to(color);

        std::chrono::system_clock::time_point extracted_time = std::chrono::system_clock::now();
        std::cout << "Time taken to extract:" << std::chrono::duration_cast<std::chrono::milliseconds>(extracted_time - receive_time).count() << "ms \n";

        // Print out FPS
        if(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - fps_time).count() == 1){
            std::cout << "FPS: " << fps_counter << " -------------------------------------------------------\n";
            fps_counter = 0;
            fps_time = std::chrono::system_clock::now();
        }

        if(save_img_to_disk){
            std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();
            points.export_to_ply("pointcloud.ply", color);

            std::chrono::system_clock::time_point save_time = std::chrono::system_clock::now();
            save_ms = std::chrono::duration_cast<std::chrono::milliseconds>(save_time - start_time).count();
            std::cout << "Time taken to save:" << save_ms << "ms \n";

            if(frame_counter != 0){
                time_taken_to_save[frame_counter-1] = save_ms;
            }
        }

        fps_counter++;
        frame_counter ++;

        if(save_benchmark_to_disk && frame_counter == num_frames){

            //TODO: Add extract time
            std::ofstream benchmark_results;
            benchmark_results.open("../benchmark_results. csv");
            benchmark_results << "Frame, Time Taken to Save (ms),Time Between Frame Receipts (ms),Time Taken to Receive (ms)\n";
            for(int i=0; i<num_frames-1; i++){
                benchmark_results << i+1 << "," << time_taken_to_save[i] << "," << time_between_frame_receipts[i] << "," << time_taken_to_receive[i] << "\n";
            }
            benchmark_results.close();
            std::cout << "Saved Benchmark Results to CSV! \n";
        }


    }
    return EXIT_SUCCESS;
}
//catch (const rs2::error & e)
//{
//    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
//    return EXIT_FAILURE;
//}
//catch (const std::exception & e)
//{
//    std::cerr << e.what() << std::endl;
//    return EXIT_FAILURE;
//}


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

