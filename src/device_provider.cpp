#include "device_provider.h"

vr::EVRInitError DeviceProvider::Init(vr::IVRDriverContext* pDriverContext) {
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

    vr::VRDriverLog()->Log("Hello world!");

	my_left_device = std::make_unique<ControllerDevice>(vr::TrackedControllerRole_LeftHand);
    vr::VRServerDriverHost()->TrackedDeviceAdded("<my_left_serial_number>",
		                                        vr::TrackedDeviceClass_Controller, 
                                                my_left_device.get());

    my_right_device = std::make_unique<ControllerDevice>(vr::TrackedControllerRole_RightHand);
    vr::VRServerDriverHost()->TrackedDeviceAdded("<my_right_serial_number>",
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

void DeviceProvider::EnterStandby() {

}

void DeviceProvider::LeaveStandby() {

}