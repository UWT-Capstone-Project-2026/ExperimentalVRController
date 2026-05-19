// ====================== Copyright (c) Valve Corporation, All rights reserved. ============================ //
#pragma once

#include <array>
#include <string>

#include "openvr_driver.h"
#include <atomic>
#include <thread>

enum MyComponent
{
    MyComponent_a_touch,
    MyComponent_a_click,
    MyComponent_trigger_value,
    MyComponent_trigger_click,
    MyComponent_haptic,
    MyComponent_MAX
};

/* Original version of controller driver - ignore for now but review for implementation of our actual controller design.
// Winsock for UDP communication with the ESP32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>

enum InputHandles {
    kInputHandle_A_click,
    kInputHandle_A_touch,
    kInputHandle_trigger_value,
    kInputHandle_trigger_click,
    kInputHandle_joystick_x,
    kInputHandle_joystick_y,
    kInputHandle_joystick_click,
    kInputHandle_haptic,
    kInputHandle_COUNT
};

// This struct defines the binary packet layout that BOTH the ESP32 firmware
// and this driver must agree on exactly. Any change here must be mirrored on
// the ESP32 side. Total size: 2 + 28 + 12 + 12 + 2 = 56 bytes.
// Use #pragma pack to prevent the compiler from adding padding bytes between
// fields, which would break the binary layout on one side or the other.
#pragma pack(push, 1)
struct ControllerPacket {
    uint8_t     header;         // 0xAA - sync byte
    uint8_t     controller_id;  // 0 = left, 1 = right
    float       qw, qx, qy, qz; // Quaternion from sensor fusion
    float       ax, ay, az;     // Raw acceleration for velocity intergration
    float       x, y, z;        // Position from base states (filled in by PI)
    uint16_t    checksum;
};
#pragma pack(pop)

*/

// ----------------------------------------------------------------------
// Purpose: Represent a single tracked device in the system.
// What this device actually is (controller, hmd) depends on the
// properties you set within the device (see implementation of Activate)
// ----------------------------------------------------------------------
class MyControllerDevice : public vr::ITrackedDeviceServerDriver 
{
public:
    MyControllerDevice(vr::ETrackedControllerRole role);

    vr::EVRInitError Activate(uint32_t unObjectID) override;

    void EnterStandby() override;

    void* GetComponent(const char* pchComponentNameAndVersion) override;

    void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;

    vr::DriverPose_t GetPose() override;

    void Deactivate() override;

    // ------------- Functions we declare ourselves below ----------------- //
    const std::string& MyGetSerialNumber();

    void MyRunFrame();
    void MyProcessEvent(const vr::VREvent_t& vrevent);

    void MyPoseUpdateThread();

private:
    std::atomic<vr::TrackedDeviceIndex_t> my_controller_index;

    vr::ETrackedControllerRole my_controller_role;
    std::string my_controller_model_number;
    std::string my_controller_serial_number;

    std::array<vr::VRInputComponentHandle_t, MyComponent_MAX> input_handles;

    std::atomic<bool> is_active;
    std::thread my_pose_update_thread;
};
