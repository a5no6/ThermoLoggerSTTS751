enum i2c_eeprom_write_state {
    I2C_EEPROM_WRITE_INIT,
    I2C_EEPROM_WRITE_OPEN,
    I2C_EEPROM_WRITE_WRITE,
    I2C_EEPROM_WRITE_CLOSE,
    I2C_EEPROM_WRITE_FINISH,
};

enum i2c_eeprom_write_state I2C_EEPROM_WriteNBytes(unsigned long Address,uint8_t *data,size_t len,enum i2c_eeprom_write_state state);

