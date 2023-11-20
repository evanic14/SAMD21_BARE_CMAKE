////
//// Created by emiel on 20-11-2023.
////
////#include "Slave_driver.hpp"
////#include "FastCRC.h"
//#include "sam.h"
//#include <hal_i2c_slave.h>
//#include <cstring>
//
//#define NUM_OF_I2C_REGISTERS 8
//
//#define I2C_SLAVE_REG_STATUS_SIZE 2
//#define I2C_SLAVE_REG_I2CCON_SIZE 3
//#define I2C_SLAVE_REG_PORTACONF_SIZE 2
//#define I2C_SLAVE_REG_PORTADATA_SIZE 255
//#define I2C_SLAVE_REG_PORTASAMPLERDY_SIZE 2
//#define I2C_SLAVE_REG_PORTBCONF_SIZE 2
//#define I2C_SLAVE_REG_PORTBDATA_SIZE 255
//#define I2C_SLAVE_REG_PORTBSAMPLERDY_SIZE 2
//
//
//#define I2C_PERMISSION_RW 0
//#define I2C_PERMISSION_RO 1
//
//#define I2C_REGS_MAX_ADDR 7
//
//i2c_slave_reg_t reg_priv;
//i2c_slave_reg_t *reg = NULL;
//
//
//FastCRC8 CRC_Gen;
//
//typedef struct {
//    uint8_t *buf;
//    size_t size;
//    uint8_t permission;
//    uint8_t crc;
//} i2c_registers_base_t;
//
//i2c_registers_base_t registers[NUM_OF_I2C_REGISTERS]{
//        {reg_priv.STATUS,         I2C_SLAVE_REG_STATUS_SIZE,         I2C_PERMISSION_RO, 0},
//        {reg_priv.I2CCON,         I2C_SLAVE_REG_I2CCON_SIZE,         I2C_PERMISSION_RW, 0},
//        {reg_priv.PORTACONF,      I2C_SLAVE_REG_PORTACONF_SIZE,      I2C_PERMISSION_RW, 0},
//        {reg_priv.PORTADATA,      I2C_SLAVE_REG_PORTADATA_SIZE,      I2C_PERMISSION_RW, 0},
//        {reg_priv.PORTASAMPLERDY, I2C_SLAVE_REG_PORTASAMPLERDY_SIZE, I2C_PERMISSION_RW, 0},
//        {reg_priv.PORTBCONF,      I2C_SLAVE_REG_PORTBCONF_SIZE,      I2C_PERMISSION_RW, 0},
//        {reg_priv.PORTBDATA,      I2C_SLAVE_REG_PORTBDATA_SIZE,      I2C_PERMISSION_RW, 0},
//        {reg_priv.PORTBSAMPLERDY, I2C_SLAVE_REG_PORTBSAMPLERDY_SIZE, I2C_PERMISSION_RW, 0}
//};
//
//typedef enum {
//    TRANSACTION_TYPE_NONE,
//    TRANSACTION_TYPE_REG_READ,
//    TRANSACTION_TYPE_REG_WRITE
//} transaction_type_t;
//
//struct I2CTransaction {
//    bool error_occured;
//    transaction_type_t transaction_type;
//    i2c_registers_base_t *reg;
//    uint8_t byte_cnt;
//    uint8_t crc;
//};
//
//
//typedef enum {
//    EVSYS_TRIGGER_NONE,
//    EVSYS_TRIGGER_I2C_SLAVE_STOP_ISR,
//    EVSYS_TRIGGER_MANUAL
//} evsys_trigger_t;
//
//
//struct I2CTransaction CurrTransaction = {false, TRANSACTION_TYPE_NONE, NULL, 0, 0};
//
//typedef struct {
//    evsys_trigger_t evsys_trigger;
//    transaction_type_t transaction_type;
//    uint8_t *reg;
//    uint8_t reg_size;
//} evsys_transaction_t;
//
//evsys_transaction_t evsys_transaction;
//
//void EVSYS_Handler_EVD1(void) {
//    evsys_trigger_t evsys_trigger = evsys_transaction.evsys_trigger;
//    switch (evsys_trigger) {
//        case EVSYS_TRIGGER_MANUAL: {
//            if (evsys_transaction.transaction_type == TRANSACTION_TYPE_REG_WRITE) {
//                size_t offset = (size_t) (evsys_transaction.reg) - (size_t) (reg);
//                uint8_t *reg_loc = (reg->STATUS);
//                memcpy(reg_loc + offset, evsys_transaction.reg, evsys_transaction.reg_size);
//            } else if (evsys_transaction.transaction_type == TRANSACTION_TYPE_REG_READ) {
//                size_t offset = (size_t) (evsys_transaction.reg) - (size_t) (reg);
//                uint8_t *reg_loc = (reg->STATUS);
//                memcpy(evsys_transaction.reg, reg_loc + offset, evsys_transaction.reg_size);
//            }
//            break;
//        }
//        case EVSYS_TRIGGER_I2C_SLAVE_STOP_ISR: {
//            if (evsys_transaction.transaction_type == TRANSACTION_TYPE_REG_WRITE) {
//                size_t offset = (size_t) (evsys_transaction.reg) - (size_t) (&reg_priv);
//                uint8_t *reg_loc = (reg->STATUS);
//                memcpy(reg_loc + offset, evsys_transaction.reg, evsys_transaction.reg_size);
//            } else if (evsys_transaction.transaction_type == TRANSACTION_TYPE_REG_READ) {
//                size_t offset = (size_t) (evsys_transaction.reg) - (size_t) (&reg_priv);
//                uint8_t *reg_loc = (reg->STATUS);
//                memcpy(evsys_transaction.reg, reg_loc + offset, evsys_transaction.reg_size);
//            }
//            break;
//        }
//        default: {
//            break;
//        }
//    }
//}
//
//
//void i2c_slave_data_send_irq(const void *const hw, volatile bustransaction_t *bustransaction) {
//    ((Sercom *) hw)->I2CS.DATA.reg = 0;
//
//}
//
//void i2c_slave_data_recv_irq(const void *const hw, volatile bustransaction_t *bustransaction) {
//    const uint8_t data = ((Sercom *) hw)->I2CS.DATA.reg;
//    if (!CurrTransaction.error_occured) {
//        if (CurrTransaction.byte_cnt == 0) {
//            if (data > I2C_REGS_MAX_ADDR) {
//                CurrTransaction.error_occured = 1;
//            } else {
//                CurrTransaction.reg = &registers[data];
//                CurrTransaction.byte_cnt++;
//            }
//
//        } else {
//            if (CurrTransaction.reg != NULL) {
//                CurrTransaction.transaction_type = TRANSACTION_TYPE_REG_WRITE;
//                if (CurrTransaction.byte_cnt < CurrTransaction.reg->size + 1) {
//                    if (CurrTransaction.reg->permission) {
//                        CurrTransaction.reg->buf[CurrTransaction.byte_cnt++ - 1] = data;
//                    } else {
//                        CurrTransaction.error_occured = 1;
//                    }
//                } else if (CurrTransaction.byte_cnt == CurrTransaction.reg->size + 1) {
//                    CurrTransaction.crc = data;
//                    uint8_t tempcrc = CRC_Gen.maxim(CurrTransaction.reg->buf, CurrTransaction.reg->size);
//                    if (CurrTransaction.crc != tempcrc) {
//
//                    }
//                } else {
//                    CurrTransaction.error_occured = 1;
//                }
//            }
//        }
//    }
//
//}
//
//void i2c_slave_stop_irq(const void *const hw, volatile bustransaction_t *bustransaction) {
//    ((Sercom *) hw)->I2CS.INTFLAG.reg = SERCOM_I2CS_INTFLAG_PREC;
//    if (CurrTransaction.reg == NULL) {
//        CurrTransaction.error_occured = 1;
//    }
//    if (!CurrTransaction.error_occured) {
//        CurrTransaction.error_occured = 0;
//        evsys_transaction.reg = CurrTransaction.reg->buf;
//        evsys_transaction.transaction_type = CurrTransaction.transaction_type;
//        evsys_transaction.reg_size = CurrTransaction.reg->size;
//
//        CurrTransaction.reg = NULL;
//        CurrTransaction.byte_cnt = 0;
//        CurrTransaction.crc = 0;
//        EVSYS->CHANNEL.reg |= EVSYS_CHANNEL_SWEVT;
//    } else {
//        CurrTransaction.reg = NULL;
//        CurrTransaction.byte_cnt = 0;
//        CurrTransaction.crc = 0;
//        CurrTransaction.error_occured = 0;
//    }
//}
//
//
//void reassign_internal_register(i2c_slave_reg_t *new_reg) {
//    reg = new_reg;
//}
//
//
//void I2CSlaveDriver::init(uint8_t slave_addr) {
//    I2C_SLAVE_INIT(peripheral_, slave_addr, I2C_CLK_SOURCE_USE_DEFAULT, I2C_EXTRA_OPT_NONE);
//}
//
//void I2CSlaveDriver::set_external_register_buffer(i2c_slave_reg_t *reg) {
//    reassign_internal_register(reg);
//}
//
//void I2CSlaveDriver::recalculate_crc() {
//    for (uint8_t i = 0; i < NUM_OF_I2C_REGISTERS; i++) {
//        registers[i].crc = CRC_Gen.maxim(registers[i].buf, registers[i].size);
//    }
//}
//
//void I2CSlaveDriver::force_update_internal_buffer(uint8_t *reg, uint8_t reg_size) {
//    evsys_transaction.reg = reg;
//    evsys_transaction.reg_size = reg_size;
//    evsys_transaction.transaction_type = TRANSACTION_TYPE_REG_READ;
//    evsys_transaction.evsys_trigger = EVSYS_TRIGGER_MANUAL;
//    EVSYS->CHANNEL.reg |= EVSYS_CHANNEL_SWEVT;
//}
//
//void I2CSlaveDriver::force_update_external_buffer(uint8_t *reg, uint8_t reg_size) {
//    evsys_transaction.reg = reg;
//    evsys_transaction.reg_size = reg_size;
//    evsys_transaction.transaction_type = TRANSACTION_TYPE_REG_WRITE;
//    evsys_transaction.evsys_trigger = EVSYS_TRIGGER_MANUAL;
//    EVSYS->CHANNEL.reg |= EVSYS_CHANNEL_SWEVT;
//}