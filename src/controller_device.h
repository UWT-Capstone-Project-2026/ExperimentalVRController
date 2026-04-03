#pragma once
#include <array>
#include <thread>
#include <mutex>
#include <atomic>

#include "openvr_driver.h"

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

struct ControllerPacket {
    uint8_t     header;         // 0xAA - sync byte
    uint8_t     controller_id;  // 0 = left, 1 = right
    float       qw, qx, qy, qz; // Quaternion from sensor fusion
    float       ax, ay, az;     // Raw acceleration for velocity intergration
    float       x, y, z;        // Position from base states (filled in by PI)
    uint16_t    checksum;
};

class ControllerDevice : public vr::ITrackedDeviceServerDriver {
public:
    ControllerDevice(vr::ETrackedControllerRole role);

    // Inherited via ITrackedDeviceServerDriver
    virtual vr::EVRInitError Activate(uint32_t unObjectId) override;
    virtual void Deactivate() override;
    virtual void EnterStandby() override;
    virtual void* GetComponent(const char* pchComponentNameAndVersion) override;
    virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;
    virtual vr::DriverPose_t GetPose() override;
    virtual void RunFrame();

private:
    struct LivePoseData {
        float qw = 1, qx = 0, qy = 0, qz = 0;
        float x = 0, y = 0, z = 0;
    };

    std::mutex pose_mutex_;
    LivePoseData live_pose_;
    std::thread recv_thread_;
    std::atomic<bool> running_ {false};

    void ReceiverThread();
    vr::ETrackedControllerRole my_controller_role_;
    vr::TrackedDeviceIndex_t device_id_;
    std::array<vr::VRInputComponentHandle_t, kInputHandle_COUNT> my_input_handles_;
};
