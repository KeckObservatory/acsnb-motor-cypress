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

/* Calibration and scale for the expected voltage across the shunt */
uint32_t ina219_calValue = 40960;
float ina219_calScale = 0.01; 

uint32_t ReadData;

void Init_INA(uint32 INA_Addr)
{
    I2C_INA_address = INA_Addr;
    
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
                      INA219_CONFIG_GAIN_8_320MV | 
                      INA219_CONFIG_BADCRES_12BIT |
                      INA219_CONFIG_SADCRES_12BIT_128S_69MS |
                      INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
   
    // Setup the Config Register    
    I2C_INA_SendData(INA219_REG_CONFIG, (config >> 8) & 0xFF, config & 0xFF);

    // Calibrate
    I2C_INA_SendData(INA219_REG_CALIBRATION, (ina219_calValue >> 8) & 0xFF, ina219_calValue & 0xFF);

}


uint32 I2C_INA_SendData(uint8 reg, uint8 data1, uint8 data2) CYREENTRANT {
    uint32 errStatus;
    
    I2C_INA_buff[0u] = reg;
    I2C_INA_buff[1u] = data1;
    I2C_INA_buff[2u] = data2;
    I2C_INA_buffIndex = 3;
    
    errStatus = I2C_INA_SendSequence();
    return errStatus;
}

uint32 I2C_INA_ReadData(uint8 readreg) CYREENTRANT {
    uint32 errStatus;
    
    I2C_INA_buff[0u] = readreg;
    I2C_INA_buffIndex = 1;
    
    errStatus = I2C_INA_SendSequence();
    if (errStatus != I2C_I2C_MSTR_NO_ERROR)
        return errStatus;
    
    errStatus = I2C_INA_ReadSequence();    
    return errStatus;    
}

uint32 I2C_INA_SendSequence(void) CYREENTRANT
{
    uint32 errStatus;
    errStatus = I2C_I2CMasterWriteBuf(I2C_INA_address, I2C_INA_buff, I2C_INA_buffIndex, 0x00);

    /* Wait until I2C Master finishes transaction */
    while(0u == (I2C_I2CMasterStatus() & 0x02)) {}
    
    switch (errStatus) {
    
        case I2C_I2C_MSTR_NO_ERROR:
            break;
    
        case I2C_I2C_MSTR_BUS_BUSY:
        case I2C_I2C_MSTR_NOT_READY:
            I2C_INA_buff[0] = 0;
            I2C_INA_buff[1] = 0;
            break;
    }

    /* Reset buffer index */
    I2C_INA_buffIndex = 0u;
    
    return errStatus;
}

uint32 I2C_INA_ReadSequence(void) CYREENTRANT {
    
    uint32 errStatus;
    
    I2C_INA_buff[0u] = 0x00;
    I2C_INA_buff[1u] = 0x00;
    I2C_INA_buffIndex = 2;
    
    errStatus = I2C_I2CMasterReadBuf(I2C_INA_address, I2C_INA_buff, I2C_INA_buffIndex, 0x00);

    while(0u == (I2C_I2CMasterStatus() & 0x02))
    {
        /* Wait until I2C Master finishes transaction */
    }

    /* Reset buffer index */
    I2C_INA_buffIndex = 0u;
    
    return errStatus;
}

int16_t getCurrent_raw(uint32 INA_addr)  {
    
    uint32 errStatus;
    I2C_INA_address = INA_addr;
    
    int16_t value;

    // Sometimes a sharp load will reset the INA219, which will
    // reset the cal register, meaning CURRENT and POWER will
    // not be available ... avoid this by always setting a cal
    // value even if it's an unfortunate extra step
    
    errStatus = I2C_INA_SendData(INA219_REG_CALIBRATION, (ina219_calValue >> 8) & 0xFF, ina219_calValue & 0xFF);
    if (errStatus != I2C_I2C_MSTR_NO_ERROR)
        return 0;
        
    // Now we can safely read the CURRENT register!
    errStatus = I2C_INA_ReadData(INA219_REG_CURRENT);
    if (errStatus != I2C_I2C_MSTR_NO_ERROR)
        return 0;

    //TODO 20230908 pmr - why was there a delay here?!?!?!
    //CyDelay(1u);
    
    // Reassemble raw/unscaled current value from the buffer
    value = (int16_t) ((I2C_INA_buff[0] << 8) | I2C_INA_buff[1]);
    
    return value;
}

float getCurrent_mA(uint32 INA_Addr)  {
  float valueScaled = getCurrent_raw(INA_Addr) * ina219_calScale;
  return valueScaled;
}

int16_t getBusVoltage_raw(uint32 INA_addr)  {
    I2C_INA_address = INA_addr;      
    uint16_t value=0;
    I2C_INA_ReadData(INA219_REG_BUSVOLTAGE);
    
    CyDelay(1u);
    value = (uint16_t)(I2C_INA_buff[0]<<8) + I2C_INA_buff[1];
    CyDelay(1u);    

    // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
    return (int16_t)((value >> 3) * 4);
}

float getBusVoltage_V(uint32 INA_addr)  {    
    int16_t value = getBusVoltage_raw(INA_addr);
    return value * 0.001;
}

/* [] END OF FILE */
