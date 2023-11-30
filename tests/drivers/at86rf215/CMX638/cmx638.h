/*
 * cmx638.h
 *
 *  Created on: 12 сент. 2023 г.
 *      Author: Viacheslav
 */

#ifndef CMX638
#define CMX638

#include "stdint.h"
#include "stdbool.h"
#include "cmx638_reg.h"



// write a fuction for think
typedef enum
{
    CMX638_OK = 0,
    CMX638_ERROR_ACK,
    CMX638_ERROR_TIMEOUT,
    CMX638_ERROR_BUFFER_SIZE,
    CMX638_ERROR_FEC_SETTINGS
} cmx638_errors;

typedef enum
{
    INPUT_GAIN_0dB = 0,
    INPUT_GAIN_1_5dB,
    INPUT_GAIN_3dB,
    INPUT_GAIN_4_5dB,
    INPUT_GAIN_6dB,
    INPUT_GAIN_7_5dB,
    INPUT_GAIN_9dB,
    INPUT_GAIN_10_5dB,
    INPUT_GAIN_12dB,
    INPUT_GAIN_13_5dB,
    INPUT_GAIN_15dB,
    INPUT_GAIN_16_5dB,
    INPUT_GAIN_18dB,
    INPUT_GAIN_19_5dB,
    INPUT_GAIN_21dB,
    INPUT_GAIN_22_5dB,
    // Add more gain levels as needed
} in_gain_lvl_e;

typedef enum
{
    OUTPUT_GAIN_MINUS_14dB = 0,
    OUTPUT_GAIN_MINUS_12dB,
    OUTPUT_GAIN_MINUS_10dB,
    OUTPUT_GAIN_MINUS_8dB,
    OUTPUT_GAIN_MINUS_6dB,
    OUTPUT_GAIN_MINUS_4dB,
    OUTPUT_GAIN_MINUS_2dB,
    OUTPUT_GAIN_0dB,
    OUTPUT_GAIN_2dB,
    OUTPUT_GAIN_4dB,
    OUTPUT_GAIN_6dB,
    OUTPUT_GAIN_8dB,
    OUTPUT_GAIN_10dB,
    OUTPUT_GAIN_12dB,
    OUTPUT_GAIN_14dB,
    OUTPUT_GAIN_16dB,
} out_gain_lvl_e;

typedef enum
{
    FRAMES_4 = 0,
    FRAMES_1,
    FRAMES_2,
    FRAMES_3,
} frames_conf_e;

typedef enum
{
    BPS_2400 = 1,
    BPS_2750,
} bitrate_conf_e;


/*
 * CMX638 support 2 dtmf characters formats,
 * but the difference is only in the sequence,
 * in this driver version the 1st format is implemented
*/
typedef enum {
    DTMF_D = 0b0000,
    DTMF_1 = 0b0001,
    DTMF_2 = 0b0010,
    DTMF_3 = 0b0011,
    DTMF_4 = 0b0100,
    DTMF_5 = 0b0101,
    DTMF_6 = 0b0110,
    DTMF_7 = 0b0111,
    DTMF_8 = 0b1000,
    DTMF_9 = 0b1001,
    DTMF_0 = 0b1010,
    DTMF_ASTERISK = 0b1011,
    DTMF_HASH = 0b1100,
    DTMF_A = 0b1101,
    DTMF_B = 0b1110,
    DTMF_C = 0b1111
} dtmf_tone_e;

typedef enum {
    DTMF_OUTPUT_OFF = 0,  // Do not play DTMF tone through the speaker
    DTMF_OUTPUT_ON        // Play DTMF tone through the speaker
} dtmf_output_mode_e;

typedef enum {
    STD_OFF = 0,
    STD_ON = 1
} std_mode_e;

/**
 * @union config_reg_u
 * @brief Configuration register union.
 *
 * This union defines the configuration settings for the system. Each bit in the
 * union represents a specific configuration parameter.
 */
typedef union
{
    struct
    {
        /**
         * @brief Number of frames in a packet.
         *
         * Defines the number of frames in a packet.
         * If FEC is enabled, use a size of 3 or 4, otherwise, an error will occur.
         *
         * Bits 1-0 of the configuration register.
         */
        frames_conf_e frames : 2;

        /**
         * @brief Bitrate configuration.
         *
         * Defines the bitrate configuration.
         * - 01: 2400 b/s
         * - 10: 2750 b/s
         *
         * Bits 3-2 of the configuration register.
         */
        bitrate_conf_e bitrate : 2;

        /**
         * @brief Forward Error Correction (FEC) setting.
         *
         * - 1: FEC enabled
         * - 0: FEC disabled
         *
         * Bit 4 of the configuration register.
         */
        bool fec : 1;

        /**
         * @brief FEC mode setting.
         *
         * Defines the mode of operation for FEC:
         * - 1: FEC in hard-bit packet mode (weaker correction, packet sizes of 27 (3 frames) or 36 (4 frames))
         * - 0: FEC in soft-bit packet mode (stronger correction, but much larger packet sizes of 108 (3 frames) or 144 (4 frames))
         *
         * Bit 5 of the configuration register.
         */
        bool hdd : 1;

        /**
         * @brief Discontinuous Transmission (DTX) setting.
         *
         * This feature is not implemented. Always set to 0.
         *
         * Bit 6 of the configuration register.
         */
        bool dtx : 1;

        /**
         * @brief DTMF (Dual Tone Multi-Frequency) format setting.
         *
         * Defines the format for DTMF. Only the first format is implemented, so always set this value to 0.
         *
         * Bit 7 of the configuration register.
         */
        bool dtmff : 1;

    } bits;

    /**
     * @brief Raw value of the configuration register.
     */
    uint8_t value;

} config_reg_u;


/**
 * @brief Initialize the CMX638 module.
 *
 * This function performs the necessary steps to initialize the CMX638 module.
 *
 * @return CMX638_OK on success, negative error code on failure.
 */
int cmx638_init(void);

/**
 * @brief Set the input gain of the CMX638 module.
 *
 * @param gain The desired gain level.
 * @param input_switch_en Enable or disable the input switch.
 */
void cmx638_input_gain_set(in_gain_lvl_e gain, bool input_switch_en);

/**
 * @brief Set the output gain of the CMX638 module.
 *
 * @param gain The desired gain level.
 * @param output_switch_en Enable or disable the output switch.
 */
void cmx638_output_gain_set(out_gain_lvl_e gain, bool output_switch_en);

/**
 * @brief Set the configuration of the CMX638 module.
 *
 * @param config The configuration structure to be set.
 */
void cmx638_config_set(config_reg_u config);

/**
 * @brief Get the current configuration of the CMX638 module.
 *
 * @return The current configuration structure.
 */
config_reg_u cmx638_config_get(void);

/**
 * @brief Start the encoding process in the CMX638 module.
 *
 * @return CMX638_OK on success, negative error code on failure.
 */
int cmx638_start_encoding(void);

/**
 * @brief Start the decoding process in the CMX638 module.
 *
 * @return CMX638_OK on success, negative error code on failure.
 */
int cmx638_start_decoding(void);

/**
 * @brief Encode a package using the CMX638 module.
 *
 * @param data Pointer to the data to be encoded.
 * @param size Size of the data to be encoded.
 * @return Size of the encoded package on success, negative error code on failure.
 */
int cmx638_encode_package(uint8_t *data, uint16_t size);

/**
 * @brief Decode a package using the CMX638 module.
 *
 * @param data Pointer to the encoded data.
 * @param size Size of the encoded data.
 * @return Size of the decoded package on success, negative error code on failure.
 */
int cmx638_decode_package(uint8_t *data, uint16_t size);

/**
 * @brief Set the DTMF output mode of the CMX638 module.
 *
 * @param output_mode Desired DTMF output mode.
 * @return CMX638_OK on success, negative error code on failure.
 */
int cmx638_set_dtmf_output_mode(dtmf_output_mode_e output_mode);

/**
 * @brief Generate a DTMF tone using the CMX638 module.
 *
 * @param tone The desired DTMF tone.
 * @param duration_ms Duration for which the tone should be generated.
 * @return CMX638_OK on success, negative error code on failure.
 */
int cmx638_generate_dtmf(dtmf_tone_e tone, uint16_t duration_ms);


/**
 * @brief Set the mode for Single Tones Detection (STD) in the CMX638 codec.
 *
 * @param mode Mode to set the STD feature (either STD_ON or STD_OFF).
 * @return int Returns CMX638_OK on success, or an error code on failure.
 */
int cmx638_set_std_mode(std_mode_e mode);
// uint8_t cmx638_is_fec_set();

#endif /* CMX638 */
