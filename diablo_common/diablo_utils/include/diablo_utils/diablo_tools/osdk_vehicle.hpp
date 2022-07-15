/**
 * @file OSDK_Vehicle.hpp
 * @brief main interface to interact with robot
 */
#pragma once

#include "osdk_hal.hpp"
#include "osdk_vehicle.hpp"
#include "osdk_movement.hpp"
#include "osdk_telemetry.hpp"
#include "osdk_virtual_rc.hpp"

namespace DIABLO{
namespace OSDK{

class Vehicle
{
public: 
    Vehicle(HAL* hal): hal(hal),
    movement_ctrl(NULL), 
    virtual_rc(NULL),
    telemetry(NULL)
    {}

    ~Vehicle()
    {
        if(movement_ctrl) delete movement_ctrl;
        if(virtual_rc)    delete virtual_rc;
        if(telemetry)     delete telemetry;
    }

    /**
     * @brief   Initialize SDK port to vehicle
     */
    uint8_t init(void);

public:
    HAL*               hal;
    
public:
    Movement_Ctrl*  movement_ctrl;
    Virtual_RC*     virtual_rc;
    Telemetry*      telemetry;

};

}
}
