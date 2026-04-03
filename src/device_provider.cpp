#include "device_provider.h"

vr::EVRInitError DeviceProvider::Init(vr::IVRDriverContext* pDriverContext) {
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

    vr::VRDriverLog()->Log("VR Controller Driver: Initializing");

    // Left hand controller listens for ESP32 packets on UDP port 5555
	my_left_device = std::make_unique<ControllerDevice>(vr::TrackedControllerRole_LeftHand, 5555);
    vr::VRServerDriverHost()->TrackedDeviceAdded(
        "VRController_Left_001",
		vr::TrackedDeviceClass_Controller, 
        my_left_device.get());

    // Right hand controller listens on UDP port 5556
    my_right_device = std::make_unique<ControllerDevice>(vr::TrackedControllerRole_RightHand, 5556);
    vr::VRServerDriverHost()->TrackedDeviceAdded(
        "VRController_Right_001",
        vr::TrackedDeviceClass_Controller,
        my_right_device.get());

    return vr::VRInitError_None;
}

void DeviceProvider::Cleanup() {
    VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

const char* const* DeviceProvider::GetInterfaceVersions() {
    return vr::k_InterfaceVersions;
}

void DeviceProvider::RunFrame() {
	if (my_left_device != nullptr) {
        my_left_device->RunFrame();
    }

	if (my_right_device != nullptr) {
        my_right_device->RunFrame();
    }
}

bool DeviceProvider::ShouldBlockStandbyMode() {
    return false;
}

void DeviceProvider::EnterStandby() {}
void DeviceProvider::LeaveStandby() {}