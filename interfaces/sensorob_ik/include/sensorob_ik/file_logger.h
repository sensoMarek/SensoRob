#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sys/stat.h>
#include <algorithm>
#include <cstdlib>
#include <iomanip>  // for std::put_time (optional)
#include <unistd.h>



#include "sensorob_ik/logger.h"

namespace file_logger {

std::string get_current_time();
std::string create_new_dir(const std::string name, const std::string path, rclcpp::Logger& LOGGER);

}