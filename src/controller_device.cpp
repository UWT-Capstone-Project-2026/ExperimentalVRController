#include "controller_device.h"

// -------------------------------------------------------------------------------------
// Construction
// -------------------------------------------------------------------------------------
ControllerDevice::ControllerDevice(vr::ETrackedControllerRole role, uint16_t udp_port) 
    : my_controller_role_(role), 
      device_id_(vr::k_unTrackedDeviceIndexInvalid), 
      udp_port_(udp_port)
{
    // Initialize Winsock once per controller. WSAStartup is safe to call multiple times
    // as long as WSACleanup is called the same number of times.
    WSADATA wsa_data;
    WSAStartup(MAKEWORD(2, 2), &wsa_data);
};

// -------------------------------------------------------------------------------------
// ITrackedDeviceServerDriver interface
// -------------------------------------------------------------------------------------
vr::EVRInitError ControllerDevice::Activate(uint32_t unObjectId) {
    vr::VRDriverLog()->Log("ControllerDevice::Activate");

    device_id_ = unObjectId;

    const vr::PropertyContainerHandle_t container = 
        vr::VRProperties()->TrackedDeviceToPropertyContainer(unObjectId);

    vr::VRProperties()->SetInt32Property(container, vr::Prop_ControllerRoleHint_Int32, my_controller_role_);
    vr::VRProperties()->SetStringProperty(container, vr::Prop_ModelNumber_String, "VR_Controller_v1.0.0");

    // Tell the runtime where our input profile is located
    vr::VRProperties()->SetStringProperty(container, vr::Prop_InputProfilePath_String,
        "{vr_controller_driver}/resources/input/vr_controller_driver_profile.json");

    // --Register input components-------------------------------------------------------
    // Boolean components: report true/false (button pressed or not)
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/a/click",        &my_input_handles_[kInputHandle_A_click]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/a/touch",        &my_input_handles_[kInputHandle_A_touch]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/trigger/click",  &my_input_handles_[kInputHandle_trigger_click]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/joystick/click", &my_input_handles_[kInputHandle_joystick_click]);

    // Scalar components: report a float value in a range
    // NormalizedOneSided = range [0.0, 1.0] (trigger pull depth)
    vr::VRDriverInput()->CreateScalarComponent(container, "/input/trigger/value", 
        &my_input_handles_[kInputHandle_trigger_value],
        vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
    
    // NormalizedTwoSided = range [-1.0, 1.0] (joystick axis)
    vr::VRDriverInput()->CreateScalarComponent(container, "/input/joystick/x", 
        &my_input_handles_[kInputHandle_joystick_x],
        vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
    vr::VRDriverInput()->CreateScalarComponent(container, "/input/joystick/y", 
        &my_input_handles_[kInputHandle_joystick_y],
        vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
    
    // Haptic output component
    vr::VRDriverInput()->CreateHapticComponent(container, "/output/haptic", 
        &my_input_handles_[kInputHandle_haptic]);

    // --Start the UDP receiver thread------------------------------------------------------
    // Set running_ to true BEFORE launching the thread so the thread's
    // while(running_) loop doesn't exit immediately on start.
    running_ = true;
    recv_thread_ = std::thread(&ControllerDevice::ReceiverThread, this);

    return vr::VRInitError_None;
}

void ControllerDevice::Deactivate() {
    // Signal the receiver thread to stop, then close the socket.
    // Closing the socket causes any blocking recvfrom() to return with an
    // error, which lets the thread exit its loop and join cleanly.
    running_ = false;

    if (udp_socket_ != INVALID_SOCKET) {
        closesocket(udp_socket_);
        udp_socket_ = INVALID_SOCKET;
    }

    // Wait for the receiver thread to fully exit before this object is destroyed.
    // Without this join, the thread could still be accessing member variables
    // after the destructor runs, causing a crash.
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }

    WSACleanup();
    device_id_ = vr::k_unTrackedDeviceIndexInvalid;
}

void ControllerDevice::EnterStandby() {}

void* ControllerDevice::GetComponent(const char* pchComponentNameAndVersion) {
    return nullptr;
}

void ControllerDevice::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) {
    if (unResponseBufferSize >= 1)
        pchResponseBuffer[0] = 0;
}

// -------------------------------------------------------------------------------------
// RunFrame - called by SteamVR on the main driver thread, ~90 times per second
// -------------------------------------------------------------------------------------
void ControllerDevice::RunFrame() {
    // Submit the latest pose to SteamVR. GetPose() locks pose_mutex_ internally
    // so it reads a consistent snapshot of live_pose_.
	vr::VRServerDriverHost()->TrackedDevicePoseUpdated(device_id_, GetPose(), sizeof(vr::DriverPose_t));

    // --Input Updates----------------------------------------------------------------
    // There are placeholder values. Replace with real data from the ESP32 packet
    // once button state is included in ControllerPacket
    vr::VRDriverInput()->UpdateBooleanComponent(my_input_handles_[kInputHandle_A_click],        false,  0.0);
    vr::VRDriverInput()->UpdateBooleanComponent(my_input_handles_[kInputHandle_A_touch],        false,  0.0);
    vr::VRDriverInput()->UpdateScalarComponent(my_input_handles_[kInputHandle_trigger_value],   0.f,    0.0);
    vr::VRDriverInput()->UpdateScalarComponent(my_input_handles_[kInputHandle_trigger_click],   false,  0.0);
    vr::VRDriverInput()->UpdateScalarComponent(my_input_handles_[kInputHandle_joystick_x],      0.0f,   0.0);
    vr::VRDriverInput()->UpdateScalarComponent(my_input_handles_[kInputHandle_joystick_y],      0.0f,   0.0);
    vr::VRDriverInput()->UpdateBooleanComponent(my_input_handles_[kInputHandle_joystick_click], false,  0.0);
}

// -------------------------------------------------------------------------------------
// GetPose - reads the latest pose from live_pose_ under the mutex
// -------------------------------------------------------------------------------------
vr::DriverPose_t ControllerDevice::GetPose() {
    vr::DriverPose_t pose   = { 0 };
    pose.poseIsValid        = true;
    pose.result             = vr::TrackingResult_Running_OK;
    pose.deviceIsConnected  = true;

    // These two quaternions describe coordinate space transforms.
    // Setting them to identity (w=1) means we're working in the same
    // coordinate space as SteamVR - which is correct for out setup/
    pose.qWorldFromDriverRotation.w = 1.f;
    pose.qDriverFromHeadRotation.w  = 1.f;

    // Lock and copy the latest data written by ReceiverThread.
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        pose.qRotation.w = live_pose_.qw;
        pose.qRotation.x = live_pose_.qx;
        pose.qRotation.y = live_pose_.qy;
        pose.qRotation.z = live_pose_.qz;
        pose.vecPosition[0] = live_pose_.x;
        pose.vecPosition[1] = live_pose_.y;
        pose.vecPosition[2] = live_pose_.z;
    }

    return pose;
}

// -------------------------------------------------------------------------------------
// ReceiverThread - runs on a background thread, received UDP packets from ESP32
// -------------------------------------------------------------------------------------
bool ControllerDevice::ValidateChecksum(const ControllerPacket& pkt) {
    // Sum every byte except the last 2 (the checksum field itself)
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&pkt);
    uint16_t sum = 0;
    for (size_t i = 0; i < sizeof(ControllerPacket) - sizeof(uint16_t); ++i) {
        sum += bytes[i];
    }
    return sum == pkt.checksum;
}

void ControllerDevice::ReceiverThread() {
    // --1. Create the UDP socket ------------------------------------------------------
    // AF_INET      = IPv4, 
    // SOCK_DGRAM   = UDP (connectionless datagrams)
    // IPPROTO_UDP  = explicitly UDP
    udp_socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_socket_ == INVALID_SOCKET) {
        vr::VRDriverLog()->Log("ReceiverThread: Failed to create UDP socket");
        return;
    }

    // Set a receive timeout so the thread can periodically check running_
    // even if no packets arrive. Without this, recvfrom() would block forever
    // and the thread could not exit cleanly when Deactivate() is called.
    DWORD timeout_ms = 500; // check running_ flag every 500ms at minimum
    setsockopt(udp_socket_, SOL_SOCKET, SO_RCVTIMEO,
               reinterpret_cast<const char*>(&timeout_ms), sizeof(timeout_ms));

    // --2. Bind the socket to a local port --------------------------------------------
    // INADDR_ANY means accept packets on any local network interface,
    // so we don't need to hard-code which network adapter to use.
    sockaddr_in local_addr{};
    local_addr.sin_family       = AF_INET;
    local_addr.sin_addr.s_addr  = INADDR_ANY;
    local_addr.sin_port         = htons(udp_port_); // htons converts host byte order to network byte order 

    if (bind(udp_socket_, reinterpret_cast<sockaddr*>(&local_addr), sizeof(local_addr)) == SOCKET_ERROR) {
        vr::VRDriverLog()->Log("ReceiverThread: Failed to bind UDP socket");
        closesocket(udp_socket_);
        udp_socket_ = INVALID_SOCKET;
        return;
    }

    char log_buf[128];
    snprintf(log_buf, sizeof(log_buf), "ReceiverThread: Listening on UDP port %d", udp_port_);
    vr::VRDriverLog()->Log(log_buf);

    // --3. Receive Loop----------------------------------------------------------------
    while (running_) {
        ControllerPacket    pkt{};
        sockaddr_in         sender_addr{};
        int                 sender_len = sizeof(sender_addr);
        
        // recvfrom blocks until a packet arrives or the timeout expires. 
        // It returns the number of bytes received, or SOCKET_ERROR on failure.
        int bytes_received = recvfrom(
            udp_socket_,
            reinterpret_cast<char*>(&pkt),  // receive directly into packet struct
            sizeof(ControllerPacket),
            0,                              // no flags
            reinterpret_cast<sockaddr*>(&sender_addr),
            &sender_len
        );

        if (bytes_received == SOCKET_ERROR) {
            int err = WSAGetLastError();
            // WSAETIMEDOUT is expected - it just means no packet arrived in
            // the timeout window. Loop back and check running_.
            if (err == WSAETIMEDOUT) continue;
            // Any other error (WSAENOTSOCK happens when Deactivate() closes
            // the socket) means we should stop the thread.
            break;
        }

        // Discard packets that are the wrong size
        if (bytes_received != sizeof(ControllerPacket)) continue;

        // Discard packets with wrong header byte
        if (pkt.header != 0xAA) continue;

        // Discard packets that fail checksum validation
        if (!ValidateChecksum(pkt)) continue;

        // Discard packets not meant for this controller
        uint8_t expected_id = (my_controller_role_ == vr::TrackedControllerRole_LeftHand) ? 0 : 1;
        if (pkt.controller_id != expected_id) continue;

        // --4. Update the shared pose data---------------------------------------------
        // The lock_guard locks pose_mutex_ on contruction and unlocks it
        // when it goes out of scope at the end of this block.
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            live_pose_.qw = pkt.qw;
            live_pose_.qx = pkt.qx;
            live_pose_.qy = pkt.qy;
            live_pose_.qz = pkt.qz;
            live_pose_.x = pkt.x;
            live_pose_.y = pkt.y;
            live_pose_.z = pkt.z;
        } 
    }

    vr::VRDriverLog()->Log("ReceiverThread: Exiting");
}
