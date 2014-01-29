#include <AP_HAL.h>

#include <AP_HAL.h>
#include "SPIDriver.h"
#include "SPIDevices.h"

using namespace VRBRAIN;

extern const AP_HAL::HAL& hal;

void VRBRAINSPIDeviceManager::init(void* machtnichts) {

    /* Note that the order of the init() of the MS5611 and MPU6k is
     * critical for the APM2. If you initialise in the wrong order
     * then the MS5611 doesn't initialise itself correctly. This
     * indicates an electrical fault in the APM2 which needs to be
     * investigated. Meanwhile, initialising the MPU6k CS pin before
     * the MS5611 CS pin works around the problem
     */

    _mpu6k = new VRBRAINSPI2DeviceDriver(77);
    _mpu6k->init();

    _mpu6k_ext = new VRBRAINSPI2DeviceDriver(8);
    _mpu6k_ext->init();


    _ms5611 = new VRBRAINSPI1DeviceDriver(94);
    _ms5611->init();

    _dataflash  = new VRBRAINSPI3DeviceDriver(83);
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
