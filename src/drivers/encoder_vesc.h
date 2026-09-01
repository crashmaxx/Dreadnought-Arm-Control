/*
    VESC Internal Encoder Driver Header
    
    This driver uses the VESC's internal position feedback as the encoder source.
*/

#ifndef ENCODER_VESC_H
#define ENCODER_VESC_H

#include "encoder_interface.h"

// VESC encoder interface
extern const encoder_interface_t vesc_encoder_interface;

#endif // ENCODER_VESC_H
