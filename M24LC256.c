#define EEPROM_ADR	0b10100000

void
i2c_start(void)
{
	StartI2C2();
	IFS3bits.MI2C2IF = 0;
	while(!IFS3bits.MI2C2IF);
}

void
i2c_stop(void)
{
	StopI2C2();
	IFS3bits.MI2C2IF = 0;
	while(!IFS3bits.MI2C2IF);
}

unsigned char
i2c_read(unsigned char ACK)
{
	unsigned char ret;
    I2C2CONbits.RCEN = 1;
    while(I2C2CONbits.RCEN);
    I2C2STATbits.I2COV = 0;
	ret = I2C2RCV;
    I2C2CONbits.ACKDT = ACK;
    I2C2CONbits.ACKEN = 1;
    while(I2C2CONbits.ACKEN);
    return ret;
}

unsigned char
i2c_write(unsigned char i2cWriteData)
{
	MasterWriteI2C2(i2cWriteData);		// адрес
	IFS3bits.MI2C2IF = 0;
	while(!IFS3bits.MI2C2IF);
	return I2C2STATbits.ACKSTAT;
}

unsigned int
i2c_waitACK(void)
{
	unsigned char ret;

	do{
		i2c_start();
		ret = i2c_write(EEPROM_ADR);
		i2c_stop();
	}while(ret);
	return 1;
}

void
i2c_WritePage(unsigned char *pos, unsigned char size, unsigned int adres)
{
	unsigned char cnt;

	i2c_waitACK();
	i2c_start();
	i2c_write(EEPROM_ADR);
	i2c_write((unsigned char)(adres >> 8));
	i2c_write((unsigned char)adres);
	for(cnt = 0; cnt < size; cnt++)
		i2c_write(*pos++);
	i2c_stop();
}

void
i2c_WriteByte(unsigned char byte, unsigned int adres)
{
	i2c_waitACK();
	i2c_start();
	i2c_write(EEPROM_ADR);
	i2c_write((unsigned char)(adres >> 8));
	i2c_write((unsigned char)adres);
	i2c_write(byte);
	i2c_stop();
}

void
i2c_ReadPage(unsigned char *pos, unsigned int size, unsigned int adres)
{
	unsigned int cnt;

	i2c_waitACK();
	i2c_start();
	i2c_write(EEPROM_ADR);
	i2c_write((unsigned char)(adres >> 8));
	i2c_write((unsigned char)adres);
	i2c_stop();
	i2c_start();
	i2c_write(EEPROM_ADR|1);
	for(cnt = 0; cnt < (size - 1); cnt++)
		*pos++ = i2c_read(0);
	*pos++ = i2c_read(1);
	i2c_stop();
}
