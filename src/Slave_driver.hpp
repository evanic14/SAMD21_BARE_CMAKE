//
// Created by emiel on 20-11-2023.
//

#ifndef ATMELSAMD21_SLAVE_DRIVER_HPP
#define ATMELSAMD21_SLAVE_DRIVER_HPP
#include <hal_i2c_slave.h>
#include <FreeRTOS.h>
#include "message_buffer.h"


//typedef enum {
//    I2C_SLAVE_DRIVER_REGISTER_STATUS,
//    I2C_SLAVE_DRIVER_REGISTER_I2CCON,
//    I2C_SLAVE_DRIVER_REGISTER_PORTACONF,
//    I2C_SLAVE_DRIVER_REGISTER_PORTADATA,
//    I2C_SLAVE_DRIVER_REGISTER_PORTASAMPLERDY,
//    I2C_SLAVE_DRIVER_REGISTER_PORTBCONF,
//    I2C_SLAVE_DRIVER_REGISTER_PORTBDATA,
//    I2C_SLAVE_DRIVER_REGISTER_PORTBSAMPLERDY
//}i2c_slave_reg_t;


typedef struct {
    uint8_t STATUS[2];
    uint8_t I2CCON[3];
    uint8_t PORTACONF[2];
    uint8_t  PORTADATA[255];
    uint8_t PORTASAMPLERDY[2];
    uint8_t PORTBCONF[2];
    uint8_t  PORTBDATA[255];
    uint8_t PORTBSAMPLERDY[2];
}i2c_slave_reg_t;

extern uint8_t i2c_registers_priv[523];


class I2CSlaveDriver {
public:
    I2CSlaveDriver(i2c_periph_inst_t peripheral) {
        peripheral_ = peripheral;
    }
    void init(uint8_t slave_addr);

    void set_external_register_buffer(i2c_slave_reg_t *reg);

    void recalculate_crc();

    void force_update_external_buffer(uint8_t *reg, uint8_t reg_size);

    void force_update_internal_buffer(uint8_t *reg, uint8_t reg_size);

private:
    i2c_periph_inst_t peripheral_;
    size_t registers_size;
};
#endif //ATMELSAMD21_SLAVE_DRIVER_HPP
