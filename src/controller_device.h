#pragma once

#include "openvr_driver.h"

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
    vr::ETrackedControllerRole my_controller_role_;
    vr::TrackedDeviceIndex_t device_id_;
};
