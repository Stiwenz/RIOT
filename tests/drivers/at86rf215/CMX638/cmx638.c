/*
 * cmx638.c
 *
 *  Created on: 12 сент. 2023 г.
 *      Author: Viacheslav
 */

#include "cmx638.h"
#include "stddef.h"
// #include "gpio.h"
#include "stdio.h"
#include "stdint.h"
// #include "stm32f4xx.h"
// #include "cmsis_os2.h"

#define TIMEOUT_RDY_REG 3000
#define MAX_FRAMES_QTY 4

#define SYSTICKCLOCK 1000000UL
#define SYSTICKPERUS (SYSTICKCLOCK / 1000000UL)

static void delay_us(uint16_t us)
{
    uint32_t ticks = SYSTICKPERUS * us;
    while (ticks != 0)
        ticks--;
    ;
}

static void cmx638_cs_set(bool value)
{
    // // HAL_GPIO_(CODEC_CS_GPIO_Port, CODEC_CS_Pin, value);
}

static void cmx638_spi_write(uint8_t *data, size_t size)
{
    for (size_t i = 0; i < size; ++i)
    {
        for (int bit = 7; bit >= 0; --bit)
        {
            // // HAL_GPIO_(CODEC_MOSI_GPIO_Port, CODEC_MOSI_Pin,
            //                   (data[i] >> bit) & 1); // send bit
            // // HAL_GPIO_(CODEC_CLK_GPIO_Port, CODEC_CLK_Pin,
            //                   GPIO_PIN_RESET);                                   // pull clock low
            delay_us(1);                                                         // wait for the data to be read
            // // HAL_GPIO_(CODEC_CLK_GPIO_Port, CODEC_CLK_Pin, GPIO_PIN_SET); // pull clock high
            delay_us(1);                                                         // wait for the data to be latched
        }
    }
    // // HAL_GPIO_(CODEC_CLK_GPIO_Port, CODEC_CLK_Pin, GPIO_PIN_RESET);
}

static void cmx638_spi_read(uint8_t *data, size_t size)
{
    for (size_t i = 0; i < size; ++i)
    {
        data[i] = 0;
        for (int bit = 7; bit >= 0; --bit)
        {
            // HAL_GPIO_(CODEC_CLK_GPIO_Port, CODEC_CLK_Pin,
                            //   GPIO_PIN_RESET); // pull clock low
            delay_us(1);                       // wait for the data to be read
            // if (HAL_GPIO_ReadPin(CODEC_MISO_GPIO_Port, CODEC_MISO_Pin))
            // {
            //     data[i] |= (1 << bit);
            // }
            // HAL_GPIO_(CODEC_CLK_GPIO_Port, CODEC_CLK_Pin, GPIO_PIN_SET); // pull clock high
            delay_us(1);                                                         // wait for the data to be latched
        }
    }
    // HAL_GPIO_(CODEC_CLK_GPIO_Port, CODEC_CLK_Pin, GPIO_PIN_RESET);
}

// Function for writing an 8-bit value to a register
void cmx638_reg_write_8(uint8_t reg_addr, uint8_t value)
{
    cmx638_cs_set(false); // Set CS before the write operation
    uint8_t command[2] = {reg_addr, value};
    cmx638_spi_write(command, sizeof(command));
    cmx638_cs_set(true); // Reset CS after the write operation
}

// Function for writing a 16-bit value to a register
void cmx638_reg_write_16(uint8_t reg_addr, uint16_t value)
{
    cmx638_cs_set(false);
    uint8_t command[3] =
        {
            reg_addr,
            (uint8_t)(value >> 8),
            (uint8_t)(value & 0xFF),
        };
    cmx638_spi_write(command, sizeof(command));
    cmx638_cs_set(true);
}

// Function for reading an 8-bit value from a register
void cmx638_reg_read_8(uint8_t reg_addr, uint8_t *value)
{
    cmx638_cs_set(false);
    cmx638_spi_write(&reg_addr, 1);
    cmx638_spi_read(value, sizeof(uint8_t));
    cmx638_cs_set(true);
}

// Function for reading a 16-bit value from a register
void cmx638_reg_read_16(uint8_t reg_addr, uint16_t *value)
{
    cmx638_cs_set(false);
    cmx638_spi_write(&reg_addr, 1);

    // Read two bytes (16 bits) and combine them into a 16-bit value
    uint8_t temp[2];
    cmx638_spi_read(temp, sizeof(temp));

    *value = (uint16_t)((temp[0] << 8) | temp[1]);
    cmx638_cs_set(true);
}

// Function for reading N bytes from a register
void cmx638_reg_read_n(uint8_t reg_addr, uint8_t *data, size_t size)
{
    cmx638_cs_set(false);
    cmx638_spi_write(&reg_addr, sizeof(reg_addr));
    cmx638_spi_read(data, size);
    cmx638_cs_set(true);
}

// Function for writing N bytes to a register
void cmx638_reg_write_n(uint8_t reg_addr, uint8_t *data, size_t size)
{
    cmx638_cs_set(false);
    cmx638_spi_write(&reg_addr, sizeof(reg_addr));
    cmx638_spi_write(data, size);
    cmx638_cs_set(true);
}

void cmx638_input_gain_set(in_gain_lvl_e gain, bool input_switch_en)
{
    uint8_t gain_bits = (uint8_t)gain & 0x0F;
    uint8_t input_switch_bit = input_switch_en ? 0x80 : 0x00;
    uint8_t value_to_write = gain_bits | input_switch_bit;
    cmx638_reg_write_8(CMX638_REG_AIG, value_to_write);
}

void cmx638_output_gain_set(out_gain_lvl_e gain, bool output_switch_en)
{
    uint8_t gain_bits = (uint8_t)gain & 0x0F;
    uint8_t output_switch_bit = output_switch_en ? 0x80 : 0x00;
    uint8_t value_to_write = gain_bits | output_switch_bit;
    cmx638_reg_write_8(CMX638_REG_AOG, value_to_write);
}

void cmx638_reset(void)
{
    cmx638_reg_write_n(CMX638_REG_RESET, NULL, 0);
    // HAL_Delay(10);
}

void cmx638_config_set(config_reg_u config)
{
    cmx638_reg_write_8(CMX638_REG_VCFG, config.value);
}

config_reg_u cmx638_config_get(void)
{
    config_reg_u config;
    cmx638_reg_read_8(CMX638_REG_MVCFG, (uint8_t *)&config.value);
    return config;
}


int cmx638_set_delay_us(uint16_t delay_us)
{
    uint16_t status;
    if (delay_us < 125)
    {
        delay_us = 125;
    }
    else if (delay_us > 31875)
    {
        delay_us = 31875;
    }
    // Calculate the delay value in 125 µs increments
    uint8_t idd_val = (uint8_t) (delay_us / 125);

    // Ensure delay is within the valid range (1-31,875)


    // Write the delay value to the CMX638_REG_DELAY register
    cmx638_reg_write_8(CMX638_REG_IDD, idd_val);

    // Wait for bit 15 (RDY) to be set in the STATUS register
    int start_time = 100;
    while (!(status & (1 << 15)))
    {
        cmx638_reg_read_16(CMX638_REG_STATUS, &status);
        if (100 - start_time > TIMEOUT_RDY_REG)
        {
            printf("CMX638 rdy timeout\n");
            return -CMX638_ERROR_TIMEOUT;
        }
        // HAL_Delay(10);
    }
    return CMX638_OK;
}

int cmx638_init(void)
{
    cmx638_reset();

    //  Enable interrupts and set processor speed
    cmx638_reg_write_16(CMX638_REG_IRQENAB, 0xC107);
    cmx638_reg_write_16(CMX638_REG_CLOCK, 0x0005);
    cmx638_reg_write_8(CMX638_REG_DTMFATTEN, 0x00);

    // Wait for the SVC bit in the STATUS register to be set
    uint16_t status;
    uint32_t start_time = 100;
    while (!(status & (1 << 14)))
    {
        cmx638_reg_read_16(CMX638_REG_STATUS, &status);
        if (100 - start_time > TIMEOUT_RDY_REG)
        {
            printf("CMX638 rdy timeout\n");
            return -CMX638_ERROR_TIMEOUT;
        }
        // HAL_Delay(10);
    };
    cmx638_reg_write_8(CMX638_REG_POWERSAVE, 0x03);

    // HAL_Delay(100);
    // Configure the Vocoder
    cmx638_reg_write_8(CMX638_REG_VCFG, 0x37);
    status = 0;
    start_time = 100;
    while (!(status & (1 << 15)))
    {
        cmx638_reg_read_16(CMX638_REG_STATUS, &status);
        if (100 - start_time > TIMEOUT_RDY_REG)
        {
            printf("CMX638 rdy timeout\n");
            return -CMX638_ERROR_TIMEOUT;
        }
        // HAL_Delay(10);
    }

    uint8_t svcack;
    // HAL_Delay(100);
    cmx638_reg_read_8(CMX638_REG_SVCACK, &svcack);
    printf("CMX638 SVCACK = 0x%x\n", svcack);
    if (!(svcack & 0x01))
    {
        return -CMX638_ERROR_ACK;
    }
    return CMX638_OK;
}

// uint8_t cmx638_is_fec_set() {
//     // Read the MVCFG register
//     config_reg_u mvcfg;
//     cmx638_reg_read_8(CMX638_REG_MVCFG, (uint8_t *)&mvcfg.value);

//     // Check the 5th bit (FEC)
//     return mvcfg.bits.fec;
// }

int cmx638_get_package_size(void)
{
    int package_size  = 0;
    config_reg_u config = cmx638_config_get();
    if (config.bits.fec)
    { // If FEC is enabled
        if (config.bits.frames != FRAMES_3 && config.bits.frames != FRAMES_4)
        {
            printf("ERROR: FEC is enabled but the number of frames is not 3 or 4\n");
            return -CMX638_ERROR_FEC_SETTINGS;
        }
        if (config.bits.hdd)
        { // If Hard bits
            package_size = (config.bits.frames == FRAMES_3) ? 27 : 36;
        }
        else
        { // soft bits
            package_size = (config.bits.frames == FRAMES_3) ? 108 : 144;
        }
        return package_size;
    }
    uint8_t frames_qty = (config.bits.frames == FRAMES_4) ? 4 : config.bits.frames;
    // when bitrate = 2400, frame size = 27, but if bitrate = 2750, frame size = 36
    uint8_t frame_size = (config.bits.bitrate == BPS_2400) ? 27 :
                 (config.bits.bitrate == BPS_2750) ? 36 : 0;
    package_size = frame_size * frames_qty;
    return package_size;
}

int cmx638_start_encoding(void)
{
    // Start decoder running (stop encoder if it was running)
    uint16_t mvctrl = 0;
    cmx638_reg_read_16(CMX638_REG_MVCTRL, &mvctrl);

    // Set 1th bit to 1 and 1st bit to 1
    mvctrl |= (1 << 1);
    mvctrl &= ~(1 << 0);

    cmx638_reg_write_16(CMX638_REG_VCTRL, mvctrl);

    // Wait for control command to complete
    uint16_t status = 0;
    uint32_t start_time = 100;
    while (!(status & (1 << 15)))
    {
        cmx638_reg_read_16(CMX638_REG_STATUS, &status);
        if (100 - start_time > TIMEOUT_RDY_REG)
        {
            printf("CMX638 rdy timeout\n");
            return -CMX638_ERROR_TIMEOUT;
        }
        // osDelay(10);
    }
    return CMX638_OK;
}

int cmx638_encode_package(uint8_t *data, uint16_t size)
{
    int package_size = cmx638_get_package_size();
    if (size < package_size)
        return -CMX638_ERROR_BUFFER_SIZE;

//    //  Wait for control command to complete
    uint16_t status = 0;
    uint32_t start_time = 100;

    // Check the SVCACK register
    uint8_t svcack;
    cmx638_reg_read_8(CMX638_REG_SVCACK, &svcack);
    if (!(svcack & 0x01))
        return -CMX638_ERROR_ACK;

    // Wait for the ENCFRAME register to be ready
    status = 0;
    start_time = 100;
    while (!(status & 0x01))
    {
        cmx638_reg_read_16(CMX638_REG_STATUS, &status);
        if (100 - start_time > TIMEOUT_RDY_REG)
        {
            printf("CMX638 rdy timeout\n");
            return -CMX638_ERROR_TIMEOUT;
        }
    }

    // Read 27 bytes from the ENCFRAME register
    cmx638_reg_read_n(CMX638_REG_ENCFRAME, data, package_size);
    return package_size;
}

int cmx638_start_decoding(void)
{
    // Start decoder running (stop encoder if it was running)
    uint16_t mvctrl = 0;
    cmx638_reg_read_16(CMX638_REG_MVCTRL, &mvctrl);

    // Set 0th bit to 1 and 1st bit to 0
    mvctrl |= (1 << 0);
    mvctrl &= ~(1 << 1);

    cmx638_reg_write_16(CMX638_REG_VCTRL, mvctrl);

    // Wait for control command to complete
    uint16_t status = 0;
    uint32_t start_time = 100;
    while (!(status & (1 << 15)))
    {
        cmx638_reg_read_16(CMX638_REG_STATUS, &status);
        if (100 - start_time > TIMEOUT_RDY_REG)
        {
            printf("CMX638 rdy timeout\n");
            return -CMX638_ERROR_TIMEOUT;
        }
        // osDelay(10);
    }
    return CMX638_OK;
}

int cmx638_decode_package(uint8_t *data, uint16_t size)
{
    uint16_t package_size = cmx638_get_package_size();
    if (size != package_size)
        return -CMX638_ERROR_BUFFER_SIZE;

    // Check the SVCACK register
    uint8_t svcack;
    cmx638_reg_read_8(CMX638_REG_SVCACK, &svcack);
    if (!(svcack & 0x01))
        return -CMX638_ERROR_ACK;


    // Write the encoded frame data to the DECFRAME register
    cmx638_reg_write_n(CMX638_REG_DECFRAME, data, size);
    return package_size;
}

int cmx638_generate_dtmf(dtmf_tone_e tone, uint16_t duration_ms) {


    // Ensure the necessary bits in the VCTRL register are set for DTMF generation
    uint16_t vctrl;
    cmx638_reg_read_16(CMX638_REG_MVCTRL, &vctrl);

    // Set the Encoder DTMF Detect (EDTMFD) bit
    vctrl |= (1 << 11);

    // Write the modified value to the VCTRL register
    cmx638_reg_write_16(CMX638_REG_VCTRL, vctrl);
    // Check the RDY bit in the STATUS register
    uint16_t status;
    int start_time = 100;
    do {
        cmx638_reg_read_16(CMX638_REG_STATUS, &status);
        if (100 - start_time > TIMEOUT_RDY_REG) {
            printf("CMX638 RDY bit not set in STATUS register\n");
            return -CMX638_ERROR_TIMEOUT;
        }
        // HAL_Delay(10);
    } while (!(status & (1 << 15)));


    // Prepare data for DTMF generation.
    uint8_t duration_bits = duration_ms / 20; // 20 ms per frame
    if (duration_ms == 0) {
        duration_bits = 0;  // OFF
    } else if (duration_ms >= 280) {
        duration_bits = 15; // Continuous
    }

    uint8_t data = (duration_bits << 4) | tone;
    // Write data to the DTMF generation register
    cmx638_reg_write_8(CMX638_REG_SDTMF, data);
    return CMX638_OK;
}

int cmx638_set_dtmf_output_mode(dtmf_output_mode_e output_mode) {

    uint16_t vctrl;
    cmx638_reg_read_16(CMX638_REG_MVCTRL, &vctrl);
    if (output_mode == DTMF_OUTPUT_ON) {
        vctrl |= (0b11 << 5);
    } else {
        vctrl &= ~(0b11 << 5);
    }
    cmx638_reg_write_16(CMX638_REG_VCTRL, vctrl);
    // Check RDY bit in STATUS register
    uint16_t status;
    int start_time = 100;
    do {
        cmx638_reg_read_16(CMX638_REG_STATUS, &status);
        if (100 - start_time > TIMEOUT_RDY_REG) {
            printf("CMX638 RDY bit not set in STATUS register\n");
            return -CMX638_ERROR_TIMEOUT;
        }
        // HAL_Delay(10);
    } while (!(status & (1 << 15)));
    return CMX638_OK;
}

int cmx638_set_std_mode(std_mode_e mode)
{
    // Read the current value of the VCTRL register
    uint16_t vctrl;
    cmx638_reg_read_16(CMX638_REG_MVCTRL, &vctrl);

    // Set or clear the appropriate bits to enable or disable STD
    if (mode == STD_ON)
    {
        vctrl |= (1 << 12);      // Enable in encoder
        vctrl |= (0b11 << 7);    // Enable in decoder
    }
    else
    {
        vctrl &= ~(1 << 12);     // Disable in encoder
        vctrl &= ~(0b11 << 7);   // Disable in decoder
    }

    // Write the updated value back to the VCTRL register
    cmx638_reg_write_16(CMX638_REG_VCTRL, vctrl);
    return CMX638_OK;
}

