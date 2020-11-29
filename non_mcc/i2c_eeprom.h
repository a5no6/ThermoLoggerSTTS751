
enum i2c_eeprom_write_state {
    I2C_EEPROM_WRITE_INIT,
    I2C_EEPROM_WRITE_OPEN,
    I2C_EEPROM_WRITE_ADDRESS_WRITE,
    I2C_EEPROM_WRITE_DATA_WRITE,
    I2C_EEPROM_WRITE_CLOSE,
    I2C_EEPROM_WRITE_FINISH,
};

enum i2c_eeprom_read_state {
    I2C_EEPROM_READ_INIT,
    I2C_EEPROM_READ_OPEN,
    I2C_EEPROM_READ_ADDRESS_WRITE,
    I2C_EEPROM_READ_DATA_READ,
    I2C_EEPROM_READ_CLOSE,
    I2C_EEPROM_READ_FINISH,
};

enum i2c_eeprom_write_state I2C_EEPROM_WriteNBytes(unsigned long Address,uint8_t *data,size_t len,enum i2c_eeprom_write_state state);

