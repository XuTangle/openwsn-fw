/**
 * Author: Tengfei Chang (tengfei.chang@inria.fr)
 * Date:   September 2016
 * Description:OpenMoteSTM-specific definition of the "i2c" bsp module.
 */

#include "stdio.h"
#include "stdint.h"

// for A8 include #include "stm32f10x_lib.h" instead of #include "stm32f10x_conf.h"
#include "stm32f10x_conf.h"
#include "stm32f10x_i2c.h"
#include "rcc.h"
#include "nvic.h"
#include "i2c.h"
#include "leds.h"

//=========================== define ==========================================

#define I2Cx                    I2C1
#define I2C_GPIO                GPIOB
#define I2C_PIN_SCL             GPIO_Pin_6
#define I2C_PIN_SDA             GPIO_Pin_7

//=========================== variables =======================================


//=========================== prototypes ======================================

void i2c_start(void);
void i2c_stop(void);
void i2c_transmit(uint8_t byte);
void i2c_address_direction(uint8_t address, uint8_t direction);

uint8_t i2c_receive_ack(void);
uint8_t i2c_receive_nack(void);

//=========================== public ==========================================

void i2c_init(void){
    // Initialization struct
    I2C_InitTypeDef  I2C_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    // 1. Clear PE bit.
    I2Cx->CR1 &= 0xFFFE;
    
    //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    
    GPIO_InitStruct.GPIO_Pin   = I2C_PIN_SCL | I2C_PIN_SDA;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C_GPIO, &GPIO_InitStruct);
    
    GPIO_Write(GPIOB, I2C_PIN_SCL | I2C_PIN_SDA | GPIO_ReadOutputData(GPIOB));

    // 3. Check SCL and SDA High level in GPIOx_IDR.
    while (1 != GPIO_ReadOutputDataBit(GPIOB, I2C_PIN_SCL));
    while (1 != GPIO_ReadOutputDataBit(GPIOB, I2C_PIN_SDA));
    
    // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    GPIO_Write(GPIOB, (~(I2C_PIN_SDA)) & GPIO_ReadOutputData(GPIOB));

    //  5. Check SDA Low level in GPIOx_IDR.
    while (0 != GPIO_ReadOutputDataBit(GPIOB, I2C_PIN_SDA));

    // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    GPIO_Write(GPIOB, (~(I2C_PIN_SCL)) & GPIO_ReadOutputData(GPIOB));

    //  7. Check SCL Low level in GPIOx_IDR.
    while (0 != GPIO_ReadOutputDataBit(GPIOB, I2C_PIN_SDA));

    // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    GPIO_Write(GPIOB, I2C_PIN_SCL | GPIO_ReadOutputData(GPIOB));

    // 9. Check SCL High level in GPIOx_IDR.
    while (1 != GPIO_ReadOutputDataBit(GPIOB, I2C_PIN_SCL));

    // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
    GPIO_Write(GPIOB, I2C_PIN_SCL | I2C_PIN_SDA | GPIO_ReadOutputData(GPIOB));

    // 11. Check SDA High level in GPIOx_IDR.
    while (1 != GPIO_ReadOutputDataBit(GPIOB, I2C_PIN_SDA));

    // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
    GPIO_InitStruct.GPIO_Pin = I2C_PIN_SCL | I2C_PIN_SDA;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C_GPIO, &GPIO_InitStruct);

    // 13. Set SWRST bit in I2Cx_CR1 register.
    I2Cx->CR1 |= 0x8000;

    // 14. Clear SWRST bit in I2Cx_CR1 register.
    I2Cx->CR1 &= 0x7FFF;

    // 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
    I2Cx->CR1 |= 0x0001;
    
    // Step 2: Initialize I2C
    I2C_InitStruct.I2C_ClockSpeed = 400000;
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0x00;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2Cx, &I2C_InitStruct);
    I2C_Cmd(I2Cx, ENABLE);
}

void i2c_read_registers(uint8_t bus_num,uint8_t slave_addr,
                             uint8_t reg_addr,
                             uint8_t numBytes,
                             uint8_t* spaceToWrite){

}

void i2c_write_register(uint8_t bus_num, uint8_t slave_addr, uint8_t length, uint8_t* data){
}

uint8_t i2c_slave_present(uint8_t bus_num,uint8_t slave_address){
    uint8_t returnVal;
    returnVal = 0;
    return returnVal;
}

void i2c_read_byte(uint8_t slave_address, uint8_t register_address, uint8_t* byte){
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
    i2c_address_direction(slave_address << 1, I2C_Direction_Transmitter);
    i2c_transmit(register_address);
    i2c_start();
    i2c_address_direction(slave_address << 1, I2C_Direction_Receiver);
    *byte = i2c_receive_nack();
    i2c_stop();
}

void i2c_read_bytes(uint8_t slave_address, uint8_t register_address, uint8_t* buffer, uint8_t length){
    uint8_t i;
    I2C_Cmd(I2Cx, DISABLE);
    I2C_Cmd(I2Cx, ENABLE);
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
    i2c_start();
    i2c_address_direction(slave_address << 1, I2C_Direction_Transmitter);
    i2c_transmit(register_address);
    i2c_start();
    i2c_address_direction(slave_address << 1, I2C_Direction_Receiver);
    for(i=0;i<length-1;i++){
        *(buffer+i) = i2c_receive_ack();
    }
    *(buffer+i) = i2c_receive_nack();
    i2c_stop();
}

void i2c_write_byte(uint8_t slave_address, uint8_t register_address,  uint8_t byte){
    // Wait until I2Cx is not busy anymore
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
    i2c_start();
    i2c_address_direction(slave_address << 1, I2C_Direction_Transmitter);
    i2c_transmit(register_address);
    i2c_transmit(byte);
    i2c_stop();
}

void i2c_write_bytes(uint8_t slave_address, uint8_t register_address, uint8_t* buffer, uint8_t length){
    uint8_t i;
    // Wait until I2Cx is not busy anymore
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
    i2c_start();
    i2c_address_direction(slave_address << 1, I2C_Direction_Transmitter);
    i2c_transmit(register_address);
    for (i=0;i<length;i++){
        i2c_transmit(*(buffer+i));
    }
    i2c_stop();
}

// interrupt handlers
void isr_i2c_tx(uint8_t bus_num){
}

void isr_i2c_rx(uint8_t bus_num){
}

//=========================== private =========================================

void i2c_start(){
    // Generate start condition
    I2C_GenerateSTART(I2Cx, ENABLE);
    // Wait for I2C EV5.
    // It means that the start condition has been correctly released
    // on the I2C bus (the bus is free, no other devices is communicating))
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
}

void i2c_stop(){
    // Generate I2C stop condition
    I2C_GenerateSTOP(I2Cx, ENABLE);
    // Wait until I2C stop condition is finished
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF));
}

void i2c_address_direction(uint8_t address, uint8_t direction){
    // Send slave address
    I2C_Send7bitAddress(I2Cx, address, direction);
    // Wait for I2C EV6
    // It means that a slave acknowledges his address
    if (direction == I2C_Direction_Transmitter){
        while (!I2C_CheckEvent(I2Cx,
            I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    }
    else if (direction == I2C_Direction_Receiver){
        while (!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    }
}

void i2c_transmit(uint8_t byte){
    // Send data byte
    I2C_SendData(I2Cx, byte);
    // Wait for I2C EV8_2.
    // It means that the data has been physically shifted out and
    // output on the bus)
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

uint8_t i2c_receive_ack(){
    // Enable ACK of received data
    I2C_AcknowledgeConfig(I2Cx, ENABLE);
    // Wait for I2C EV7
    // It means that the data has been received in I2C data register
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
    // Read and return data byte from I2C data register
    return I2C_ReceiveData(I2Cx);
}

uint8_t i2c_receive_nack(){
    // Disable ACK of received data
    I2C_AcknowledgeConfig(I2Cx, DISABLE);
    // Wait for I2C EV7
    // It means that the data has been received in I2C data register
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
    // Read and return data byte from I2C data register
    return I2C_ReceiveData(I2Cx);
}

