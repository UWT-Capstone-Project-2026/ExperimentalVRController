#pragma once
#include <array>
#include <thread>
#include <mutex>
#include <atomic>

#include "openvr_driver.h"

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

class ControllerDevice : public vr::ITrackedDeviceServerDriver {
public:
    // udp_port: the port this controller listens on (e.g. 5555 for left, 5556 for right)
    ControllerDevice(vr::ETrackedControllerRole role, uint16_t udp_port);

    // Inherited via ITrackedDeviceServerDriver
    virtual vr::EVRInitError Activate(uint32_t unObjectId) override;
    virtual void Deactivate() override;
    virtual void EnterStandby() override;
    virtual void* GetComponent(const char* pchComponentNameAndVersion) override;
    virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;
    virtual vr::DriverPose_t GetPose() override;
    virtual void RunFrame();

private:
    // The latest pose data received from the ESP32, protected by pose_mutex_.
    // GetPose() reads this; ReceiverThread() writes it.
    struct LivePoseData {
        float qw = 1, qx = 0, qy = 0, qz = 0;   // Identity quaternion by default
        float x = 0, y = 0, z = 0;
    };

    void ReceiverThread();
    bool ValidateChecksum(const ControllerPacket& pkt);

    vr::ETrackedControllerRole  my_controller_role_;
    vr::TrackedDeviceIndex_t    device_id_;
    uint16_t                    udp_port_;

    std::array<vr::VRInputComponentHandle_t, kInputHandle_COUNT> my_input_handles_;

    std::mutex          pose_mutex_;
    LivePoseData        live_pose_;
    std::thread         recv_thread_;
    std::atomic<bool>   running_ {false};

    // Winsock socket handle for UDP receive
    SOCKET udp_socket_ = INVALID_SOCKET;
};
