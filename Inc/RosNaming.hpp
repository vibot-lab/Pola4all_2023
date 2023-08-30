#ifndef __ROS_NAMING_HPP__
#define __ROS_NAMING_HPP__

/**
 * @brief ROS Naming header file
 *
 *  This file contains all the defines required for the ROS client. It defines
 * macros with the different topic and services names, so we do not have to synchronize
 * the names in the server and the client side. The same file is used.
 *
 * NOTE: THIS FILE IS COPIED FROM THE MAIN PROJECT TO THE ROS PROJECT EACH
 * TIME WE COMPILE THE SERVER. ONLY THE VERSION THAT IS IN THE PATH Inc/RosNaming.hpp
 * MUST BE MODIFIED. THE ONE AT
 * RosCameraServer/src/polarimetric_camera_server/include/polarimetric_camera_server/RosNaming.hpp
 * WILL BE ALWAYS OVERWRITTEN!!!
*/

#define CAMERA_IMAGE_TOPIC_NAME "camera_image_topic"
#define CAMERA_STATE_TOPIC_NAME "camera_state_topic"
#define CAMERA_TEMPERATURE_TOPIC_NAME "camera_temp_topic"
#define CAMERA_CHANGE_PARAM_SERVICE "camera_set_params_service"
#define CAMERA_REQUEST_PARAMS_TOPIC_NAME "camera_request_params_topic"
#define CAMERA_IS_ALIVE_SERVICE "camera_is_alive_srv"
#define CAMERA_REQUEST_SPECIFIC_PARAM_SERVICE "camera_request_single_param_srv"
#define CAMERA_REQUEST_CAMERA_IMAGE_SERVICE "camera_request_single_image_srv"

#endif // __ROS_NAMING_HPP__