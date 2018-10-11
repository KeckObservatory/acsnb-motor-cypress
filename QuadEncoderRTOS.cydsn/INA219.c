/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#include "INA219.h"

                
/* Buffer that holds data to be send to INA219 slave */
static uint8 I2C_INA_buff[10];


/* Variable used for buffer indexing */
static uint32 I2C_INA_buffIndex = 0u;
/* Variable stores the I2C address */
static uint32 I2C_INA_address = 0x40;      

uint32_t ina219_calValue = 8192;

uint32_t ReadData;

void Init_INA(uint32 INA_Addr)
{
    I2C_INA_address = INA_Addr;
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                INA219_CONFIG_GAIN_8_320MV |
                INA219_CONFIG_BADCRES_12BIT |
                INA219_CONFIG_SADCRES_12BIT_1S_532US |
                INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
    
    
    
    I2C_INA_SendData(INA219_REG_CALIBRATION, 0x20, 0x00);
    
    // setup the Config Register    
    I2C_INA_SendData(INA219_REG_CONFIG, config&0xff00>>8, config&0x00ff);
}


void I2C_INA_SendData(uint8 reg, uint8 data1, uint8 data2) CYREENTRANT
{
    I2C_INA_buff[0u] = reg;
    I2C_INA_buff[1u] = data1;
    I2C_INA_buff[2u] = data2;
    I2C_INA_buffIndex = 3;
    
    I2C_INA_SendSequence();
}

void I2C_INA_ReadData(uint8 readreg) CYREENTRANT
{
    I2C_INA_buff[0u] = readreg;
    I2C_INA_buffIndex = 1;
    I2C_INA_SendSequence();
    I2C_INA_ReadSequence();
}

void I2C_INA_SendSequence(void) CYREENTRANT
{
    (void) I2C_I2CMasterWriteBuf(I2C_INA_address,
                                           I2C_INA_buff,
                                           I2C_INA_buffIndex,
                                           0x00);

    while(0u == (I2C_I2CMasterStatus() & 0x02))
    {
        /* Wait until I2C Master finishes transaction */
    }

    /* Reset buffer index */
    I2C_INA_buffIndex = 0u;
}
void I2C_INA_ReadSequence(void) CYREENTRANT
{
    I2C_INA_buff[0u] = 0x00;
    I2C_INA_buff[1u] = 0x00;
    I2C_INA_buffIndex = 2;
    I2C_I2CMasterReadBuf(I2C_INA_address,I2C_INA_buff,I2C_INA_buffIndex,0x00);

    while(0u == (I2C_I2CMasterStatus() & 0x02))
    {
        /* Wait until I2C Master finishes transaction */
    }

    /* Reset buffer index */
    I2C_INA_buffIndex = 0u;
}

int16_t getCurrent_raw(uint32 INA_addr) 
{
    I2C_INA_address = INA_addr;
    
    uint16_t value=0;

    // Sometimes a sharp load will reset the INA219, which will
    // reset the cal register, meaning CURRENT and POWER will
    // not be available ... avoid this by always setting a cal
    // value even if it's an unfortunate extra step
    I2C_INA_SendData(INA219_REG_CALIBRATION, 0x20, 0x00);

    // Now we can safely read the CURRENT register!
    I2C_INA_ReadData(INA219_REG_CURRENT);

    CyDelay(1u);
    value = (uint16_t)(I2C_INA_buff[0]<<8) + I2C_INA_buff[1];
    CyDelay(1u);
    return (int16_t)value;
}

float getCurrent_mA(uint32 INA_Addr) 
{
  float valueDec = getCurrent_raw(INA_Addr) * 0.05;
  return valueDec;
}

int16_t getBusVoltage_raw(uint32 INA_addr) 
{
    I2C_INA_address = INA_addr;      
    uint16_t value=0;
    I2C_INA_ReadData(INA219_REG_BUSVOLTAGE);
    
    CyDelay(1u);
    value = (uint16_t)(I2C_INA_buff[0]<<8) + I2C_INA_buff[1];
    CyDelay(1u);    

    // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
    return (int16_t)((value >> 3) * 4);
}

float getBusVoltage_V(uint32 INA_addr) 
{    
    int16_t value = getBusVoltage_raw(INA_addr);
    return value * 0.001;
}

/* [] END OF FILE */
