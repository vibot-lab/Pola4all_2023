// OpenCV includes

// STD includes
#include <iostream>
#include <signal.h>

// Qt Includes

// Custom includes
#include "PolarimetricCameraServer.hpp"

// ROS includes
#include <ros/ros.h>

// \brief: List of implemented / valid camera drivers.
static const std::vector<std::string> availableCameraDrivers(
    {
        "BaslerUSB",
        "TemplateDriver",
        "EthernetDriver"
    }
);

template<class T>
T readSingleParam(std::string name)
{
    T param;
    // We use the ~ to access to the private params of the node
    if (!ros::param::get("~" + name, param))
    {
        ROS_ERROR("Cannot retrieve the parameter %s", name.c_str());
    }
    else
    {
        std::cout << name << " : " << param << std::endl;
    }
    return param;
}

/**
 * @brief convertListToVector: Take a ROS param input array, and we convert it
 *   into a vector of strings.
 *
 * @arg listOfVals: Array of type XmlRpc::XmlRpcValue, that contains a list of strings
 *   coming from the launch file.
 *
 * @returns Vector of the equivalent std::string values.
*/
std::vector<std::string> convertListToVector(const XmlRpc::XmlRpcValue& listOfVals)
{
    std::vector<std::string> vals;
    if(listOfVals.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        for( int i=0; i<listOfVals.size(); ++i )
        {
            std::string retVal = "";
            if(listOfVals[i].getType() == XmlRpc::XmlRpcValue::TypeString)
            {
                retVal = static_cast<std::string>(listOfVals[i]);
            }
            else
            {
                std::cout << "ERROR: Cannot parse argument into string" << std::endl;
            }
            std::cout << "Retrieved value: " << retVal << std::endl;
            vals.push_back(retVal);
        }
    }
    return vals;
}

/**
 * @brief verifyDriverNames: Verify if the strings in the input vector are
 *   included in the list of implemented drivers. If a given driver is not
 *   implemented, an assertion exception will be thrown.
 *
 * @arg vec: Vector with strings that represent the list of available drivers
 *   for this package.
*/
void verifyDriverNames(std::vector<std::string> vec)
{
    for (std::string driver : vec)
    {
        if (!std::count(availableCameraDrivers.begin(), availableCameraDrivers.end(), driver))
        {
            ROS_WARN("Cannot find the driver %s in the list of options. The available options are:", driver.c_str());
            for (std::string v : availableCameraDrivers)
            {
                ROS_WARN("%s", v.c_str());
            }
            ROS_WARN("Closing...");
            assert(0);
        }
    }
}

/**
 * @brief fillCamerasStruct: Add camera configurations to the output configurations vector.
 *   These configurations include the information about the camera user-defined name,
 *   the driver to use for it, and if the camera is a master or a slave.
 *
 * @arg camNames: Vector with strings that are supposed to be valid the camera user-defined names.
 * @arg camDrivers: Vector with strings that are supposed to be valid implemented camera drivers.
 * @arg isMaster: If true, the corresponding flag is set to master mode. If not, it is set to slave mode.
 * @arg configs [out]: Output vector to which the new camera configuration structs will be added.
*/
void fillCamerasStruct(std::vector<std::string> camNames, std::vector<std::string> camDrivers, bool isMaster, std::vector<CameraConfig>& configs)
{
    assert(camNames.size() == camDrivers.size());
    for (size_t i = 0; i < camDrivers.size(); i++)
    {
        CameraConfig conf = {
            .camName = camNames[i],
            .camDriver = camDrivers[i],
            .isMaster = isMaster
        };
        configs.push_back(conf);
    }
}

void onShutdown(int sig)
{
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_server_node", ros::init_options::AnonymousName);

    ros::NodeHandle nh;
    signal(SIGINT, onShutdown);

    // We read the parameters from the launch file
    bool free_run = readSingleParam<bool>("mode");
    int freq = readSingleParam<int>("freq");
    int bitDepth = readSingleParam<int>("bit_depth");

    // We read the camera user-defined IDS and the corresponding drivers to use
    std::vector<std::string> masterCamNames = convertListToVector(readSingleParam<XmlRpc::XmlRpcValue>("master_cameras_names"));
    std::vector<std::string> slaveCamNames = convertListToVector(readSingleParam<XmlRpc::XmlRpcValue>("slave_cameras_names"));

    std::vector<std::string> masterCamDrv = convertListToVector(readSingleParam<XmlRpc::XmlRpcValue>("master_cameras_drivers"));
    std::vector<std::string> slaveCamDrv = convertListToVector(readSingleParam<XmlRpc::XmlRpcValue>("slave_cameras_drivers"));

    // We verify that the entered drivers are valid options
    verifyDriverNames(masterCamDrv);
    verifyDriverNames(slaveCamDrv);

    // We fill the structs with master / slave camera names and drivers
    std::vector<CameraConfig> cameraParams;
    fillCamerasStruct(masterCamNames, masterCamDrv, true, cameraParams);
    fillCamerasStruct(slaveCamNames, slaveCamDrv, false, cameraParams);

    PolarimetricCameraServer myCam(&nh, cameraParams);
    myCam.initializeServer(free_run, freq, bitDepth);

    ros::spin();
    myCam.stopThreads();
    std::cout << "Server closed!" << std::endl;
    return 0;
}
