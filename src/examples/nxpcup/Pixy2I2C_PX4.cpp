#include "Pixy2I2C_PX4.h"


PIXY2_I2C::PIXY2_I2C() :
	I2C("PIXY2_I2C", IRLOCK0_DEVICE_PATH, PX4_I2C_BUS_EXPANSION, IRLOCK_I2C_ADDRESS, 400000)
{
	_external = true;
}

bool PIXY2_I2C::is_external()
{
	return _external;
};

int PIXY2_I2C::init()
{
	return I2C::init();
};
