#include "controller_device.h"

ControllerDevice::ControllerDevice(vr::ETrackedControllerRole role) : my_controller_role_(role), device_id_(vr::k_unTrackedDeviceIndexInvalid) {};

vr::EVRInitError ControllerDevice::Activate(uint32_t unObjectId) {
    vr::VRDriverLog()->Log("ControllerDevice::Activate");

    const vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer(unObjectId);
    vr::VRProperties()->SetInt32Property(container, vr::Prop_ControllerRoleHint_Int32, my_controller_role_);
    vr::VRProperties()->SetStringProperty(container, vr::Prop_ModelNumber_String, "<my_controller_model_number>");

    device_id_ = unObjectId;

    // Tell the runtime where our input profile is located
    vr::VRProperties()->SetStringProperty(container, vr::Prop_InputProfilePath_String,
        "{vr_controller_driver}/resources/input/vr_controller_driver_profile.json");

    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/a/click", &my_input_handles_[kInputHandle_A_click]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/a/touch", &my_input_handles_[kInputHandle_A_touch]);

    vr::VRDriverInput()->CreateScalarComponent(container, "/input/trigger/value", &my_input_handles_[kInputHandle_trigger_value],
        vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/trigger/click", &my_input_handles_[kInputHandle_trigger_click]);

    vr::VRDriverInput()->CreateScalarComponent(container, "/input/joystick/x", &my_input_handles_[kInputHandle_joystick_x],
        vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
    vr::VRDriverInput()->CreateScalarComponent(container, "/input/joystick/y", &my_input_handles_[kInputHandle_joystick_y],
        vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/joystick/click", &my_input_handles_[kInputHandle_joystick_click]);

    vr::VRDriverInput()->CreateHapticComponent(container, "/output/haptic", &my_input_handles_[kInputHandle_haptic]);

    return vr::VRInitError_None;
}

void ControllerDevice::Deactivate() {
}

void ControllerDevice::EnterStandby() {
}

void* ControllerDevice::GetComponent(const char* pchComponentNameAndVersion) {
    return nullptr;
}

void ControllerDevice::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) {
    if (unResponseBufferSize >= 1)
        pchResponseBuffer[0] = 0;
}

void ControllerDevice::RunFrame() {
	vr::VRServerDriverHost()->TrackedDevicePoseUpdated(device_id_, GetPose(), sizeof(vr::DriverPose_t));

    vr::VRDriverInput()->UpdateBooleanComponent(my_input_handles_[kInputHandle_A_click], 1, 0.0);
    vr::VRDriverInput()->UpdateBooleanComponent(my_input_handles_[kInputHandle_A_touch], 1, 0.0);
    vr::VRDriverInput()->UpdateScalarComponent(my_input_handles_[kInputHandle_trigger_value], 0.5, 0.0);
    vr::VRDriverInput()->UpdateScalarComponent(my_input_handles_[kInputHandle_trigger_click], true, 0.0);
    vr::VRDriverInput()->UpdateScalarComponent(my_input_handles_[kInputHandle_joystick_x], true, 0.0);
    vr::VRDriverInput()->UpdateScalarComponent(my_input_handles_[kInputHandle_joystick_y], true, 0.0);
    vr::VRDriverInput()->UpdateBooleanComponent(my_input_handles_[kInputHandle_joystick_click], 1, 0.0);
}

template < class T >
vr::HmdQuaternion_t HmdQuaternion_FromMatrix(const T& matrix)
{
    vr::HmdQuaternion_t q{};

    q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;

    q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
    q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
    q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);

    return q;
}

vr::DriverPose_t ControllerDevice::GetPose() {
    vr::DriverPose_t pose = { 0 };
    pose.poseIsValid = true;
    pose.result = vr::TrackingResult_Running_OK;
    pose.deviceIsConnected = true;
    pose.qWorldFromDriverRotation.w = 1.f;
    pose.qDriverFromHeadRotation.w = 1.f;

    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose.qRotation.w = live_pose_.qw;
    pose.qRotation.x = live_pose_.qx;
    pose.qRotation.y = live_pose_.qy;
    pose.qRotation.z = live_pose_.qz;
    pose.vecPosition[0] = live_pose_.x;
    pose.vecPosition[1] = live_pose_.y;
    pose.vecPosition[2] = live_pose_.z;

    return pose;
}

void ControllerDevice::ReceiverThread() {
    // Windows: Winsock, Linux: POSIX sockets
    // Bind to UDP port 5555 (left) or 5556 (right)

    while (running_) {
        ControllerPacket pkt;
        // recvfrom(...) into pkt

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
