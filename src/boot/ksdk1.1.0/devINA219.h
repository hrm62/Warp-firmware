/*
    2023.  Holly Martin.  Partly copied from devMMA8451Q.h, Phillip Stanley-Marbell et al. 2016-2018.
*/

void		initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);
WarpStatus	readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes);
WarpStatus	writeSensorRegisterINA219(uint8_t deviceRegister,
					uint8_t payloadBtye);
//WarpStatus	configureSensorINA219(uint8_t payloadF_SETUP, uint8_t payloadCTRL_REG1);
void		printSensorDataINA219(bool hexModeFlag);