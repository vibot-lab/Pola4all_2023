#ifndef __CAMERA_STATE_CALLBACK_HPP__
#define __CAMERA_STATE_CALLBACK_HPP__

#include <CameraTypes.hpp>

/**
 * @brief CameraStateCallback class
 *
 *  This class is just an interface to generically pass a callback to another class.
 * It has been created in order to remove the Qt dependency from some modules.
 * Using this class is similar to define signals and slots in Qt, but in a
 * synchronous way, and without the need of having Qt in the system.
 *
 *  A class that wants to be notified from certain event must inherit from this
 * class and implement its only method: updateStateCallback. As a consequence,
 * the child class becomes a CameraStateCallback type also, and it can be passed
 * to another classes as a CameraStateCallback pointer. Through this pointer,
 * the other class that wants to use the updateStateCallback function
 * does not need to know the type of origin of the class.
*/
class CameraStateCallback
{
public:
    /// \brief Constructor. Since this is a pure-virtual class, it cannot be instanciated.
    CameraStateCallback() {}

    /**
     * @brief updateStateCallback: Pure-virtual method. This method must be called
     *  whenever the class that hold a pointer to this class wants to notify the
     *  camera state.
    */
    virtual void updateStateCallback(const CameraState &newState) = 0;
};
#endif // __CAMERA_STATE_CALLBACK_HPP__