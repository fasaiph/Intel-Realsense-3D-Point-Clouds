#include <iostream>
#include <librealsense2/rs.hpp>

void get_field_of_view(const rs2::stream_profile& stream)
{
    // A sensor's stream (rs2::stream_profile) is in general a stream of data with no specific type.
    // For video streams (streams of images), the sensor that produces the data has a lens and thus has properties such
    //  as a focal point, distortion, and principal point.
    // To get these intrinsics parameters, we need to take a stream and first check if it is a video stream
    if (auto video_stream = stream.as<rs2::video_stream_profile>())
    {
        try
        {
            //If the stream is indeed a video stream, we can now simply call get_intrinsics()
            rs2_intrinsics intrinsics = video_stream.get_intrinsics();

            auto principal_point = std::make_pair(intrinsics.ppx, intrinsics.ppy);
            auto focal_length = std::make_pair(intrinsics.fx, intrinsics.fy);
            rs2_distortion model = intrinsics.model;

            std::cout << "Principal Point         : " << principal_point.first << ", " << principal_point.second << std::endl;
            std::cout << "Focal Length            : " << focal_length.first << ", " << focal_length.second << std::endl;
            std::cout << "Distortion Model        : " << model << std::endl;
            std::cout << "Distortion Coefficients : [" << intrinsics.coeffs[0] << "," << intrinsics.coeffs[1] << "," <<
                intrinsics.coeffs[2] << "," << intrinsics.coeffs[3] << "," << intrinsics.coeffs[4] << "]" << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cerr << "Failed to get intrinsics for the given stream. " << e.what() << std::endl;
        }
    }
    else if (auto motion_stream = stream.as<rs2::motion_stream_profile>())
    {
        try
        {
            //If the stream is indeed a motion stream, we can now simply call get_motion_intrinsics()
            rs2_motion_device_intrinsic intrinsics = motion_stream.get_motion_intrinsics();

            std::cout << " Scale X      cross axis      cross axis  Bias X \n";
            std::cout << " cross axis    Scale Y        cross axis  Bias Y  \n";
            std::cout << " cross axis    cross axis     Scale Z     Bias Z  \n";
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    std::cout << intrinsics.data[i][j] << "    ";
                }
                std::cout << "\n";
            }

            std::cout << "Variance of noise for X, Y, Z axis \n";
            for (int i = 0; i < 3; i++)
                std::cout << intrinsics.noise_variances[i] << " ";
            std::cout << "\n";

            std::cout << "Variance of bias for X, Y, Z axis \n";
            for (int i = 0; i < 3; i++)
                std::cout << intrinsics.bias_variances[i] << " ";
            std::cout << "\n";
        }
        catch (const std::exception& e)
        {
            std::cerr << "Failed to get intrinsics for the given stream. " << e.what() << std::endl;
        }
    }
    else
    {
        std::cerr << "Given stream profile has no intrinsics data" << std::endl;
    }
}

void get_extrinsics(const rs2::stream_profile& from_stream, const rs2::stream_profile& to_stream)
{
    // If the device/sensor that you are using contains more than a single stream, and it was calibrated
    // then the SDK provides a way of getting the transformation between any two streams (if such exists)
    try
    {
        // Given two streams, use the get_extrinsics_to() function to get the transformation from the stream to the other stream
        rs2_extrinsics extrinsics = from_stream.get_extrinsics_to(to_stream);
        std::cout << "Translation Vector : [" << extrinsics.translation[0] << "," << extrinsics.translation[1] << "," << extrinsics.translation[2] << "]\n";
        std::cout << "Rotation Matrix    : [" << extrinsics.rotation[0] << "," << extrinsics.rotation[3] << "," << extrinsics.rotation[6] << "]\n";
        std::cout << "                   : [" << extrinsics.rotation[1] << "," << extrinsics.rotation[4] << "," << extrinsics.rotation[7] << "]\n";
        std::cout << "                   : [" << extrinsics.rotation[2] << "," << extrinsics.rotation[5] << "," << extrinsics.rotation[8] << "]" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Failed to get extrinsics for the given streams. " << e.what() << std::endl;
    }
}

int main() {
    std::cout << "Hello, World!" << std::endl;
    return 0;
}