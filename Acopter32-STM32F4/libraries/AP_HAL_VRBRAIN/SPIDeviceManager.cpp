#include <AP_HAL.h>

#include <AP_HAL.h>
#include "SPIDriver.h"
#include "SPIDevices.h"

using namespace VRBRAIN;

extern const AP_HAL::HAL& hal;

void VRBRAINSPIDeviceManager::init(void* machtnichts) {

    _mpu6k = new VRBRAINSPI2DeviceDriver(77);
    _mpu6k->init();

    _mpu6k_ext = new VRBRAINSPI3DeviceDriver(22);
    _mpu6k_ext->init();


    _ms5611 = new VRBRAINSPI1DeviceDriver(94);
    _ms5611->init();

    _dataflash  = new VRBRAINSPI1DeviceDriver(83);
    _dataflash ->init();

}

AP_HAL::SPIDeviceDriver* VRBRAINSPIDeviceManager::device(enum AP_HAL::SPIDevice d)
{
    switch (d) {
        case AP_HAL::SPIDevice_Dataflash:
            return _dataflash;
        case AP_HAL::SPIDevice_MS5611:
            return _ms5611;
        case AP_HAL::SPIDevice_MPU6000:
            return _mpu6k;
        case AP_HAL::SPIDevice_MPU6000_Ext:
            return _mpu6k_ext;
        default:
            return NULL;
    };
}
