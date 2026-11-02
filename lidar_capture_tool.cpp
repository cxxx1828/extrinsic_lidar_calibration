
#include <memory>  

#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/msg/pcl_point_cloud_msg.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <string>
#include <chrono>
#include <thread>       
#include <fstream>
#include <iostream>
#include <cstdio>       
#include <stdexcept>
#include <unistd.h>     
#include <linux/limits.h> 

using namespace robosense::lidar;

typedef pcl::PointXYZI PointT;
typedef PointCloudT<PointT> PointCloudMsg;

std::string getExecutableDir()
{
    char buf[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (len == -1)
    {
        throw std::runtime_error("Cannot read /proc/self/exe");
    }
    buf[len] = '\0';
    std::string exe_path(buf);
    return exe_path.substr(0, exe_path.find_last_of('/'));
}

static const std::string EXE_DIR      = getExecutableDir();       
static const std::string PROJECT_ROOT = EXE_DIR + "/..";           
static const std::string OUTPUT_DIR   = PROJECT_ROOT + "/calib_images/lidar/";
static const std::string TRIGGER_FILE = OUTPUT_DIR + ".trigger";

std::shared_ptr<PointCloudMsg> pointCloudGetCallback()
{
    return std::make_shared<PointCloudMsg>();
}

void pointCloudPutCallback(std::shared_ptr<PointCloudMsg> msg)
{
    std::ifstream trigger(TRIGGER_FILE);
    if (!trigger.good())
    {
        return;  
    }

    std::string index;
    trigger >> index;
    trigger.close();

    if (index.empty())
    {
        return;  
    }

    std::string filename = OUTPUT_DIR + "lidar_" + index + ".pcd";
    try
    {
        pcl::io::savePCDFileASCII(filename, *msg);
        std::cout << "Saved " << filename << "\n";
    }
    catch (const pcl::IOException& e)
    {
        std::cerr << "Failed to save " << filename << ": " << e.what() << "\n";
    }

    std::remove(TRIGGER_FILE.c_str());  
}

void exceptionCallback(const Error& code)
{
    // Log 
}

int main()
{
    LidarDriver<PointCloudMsg> driver;
    RSDriverParam param;
    param.lidar_type = LidarType::RSAIRYLITE_ETH;
    param.input_type = InputType::ONLINE_LIDAR;
    param.input_param.msop_port = 6699;
    param.input_param.difop_port = 7788;

    driver.regPointCloudCallback(pointCloudGetCallback, pointCloudPutCallback);
    driver.regExceptionCallback(exceptionCallback);

    if (!driver.init(param))
    {
        std::cerr << "Driver init failed\n";
        return -1;
    }
    driver.start();

    std::cout << "Output dir: "   << OUTPUT_DIR   << "\n";
    std::cout << "Trigger file: " << TRIGGER_FILE  << "\n";
    std::cout << "Waiting for triggers, Ctrl+C to stop...\n";
    while (true) { std::this_thread::sleep_for(std::chrono::seconds(1)); }
}
