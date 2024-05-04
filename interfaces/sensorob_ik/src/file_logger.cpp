
#include "sensorob_ik/file_logger.h"

namespace file_logger {

std::string get_current_time() {
    // Get the current system time point
    auto now = std::chrono::system_clock::now();

    // Convert the system time point to a time_t object
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);

    // Convert the time_t object to a tm struct for local time
    std::tm local_tm = *std::localtime(&now_time_t);

    // return the current date and time
    std::stringstream sst;
    sst << std::put_time(&local_tm, "%Y-%m-%d_%H-%M-%S");
    return sst.str();
}


std::string create_new_dir(const std::string name, const std::string path, rclcpp::Logger& LOGGER) {
    std::string dir_name = path + "/" + name;    
    // Check if the directory already exists
    if (access(dir_name.c_str(), F_OK) == -1) {
        // Create the directory
        if (mkdir(dir_name.c_str(), 0777) == 0) {
            // clog("Directory " + dir_name + " created successfully", LOGGER);
            return dir_name;
        } else {
            clog("Failed to create directory " + dir_name, LOGGER, ERROR);
        }
    } 
    // else {
    //     std::cerr << "Directory already exists: " << dir_name << std::endl;
    // }

    return dir_name;

}



}