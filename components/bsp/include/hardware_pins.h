/**
 * @file hardware_pins.h
 * @brief Hardware Pin Definitions for ESP32-DEVKITC-VIE.
 * @details Maps abstract signal names to specific physical GPIO pin numbers on ESP32-DEVKITC-VIE.
 * @author yong-lun.lee
 * @date
 */

#pragma once

#ifndef HARDWARE_PINS_H
#define HARDWARE_PINS_H

// ---------------------------------------------------------------------------
// User Inputs and Indicators
// ---------------------------------------------------------------------------
#define BOOT_BUTTON_PIN     0     // User Button
#define LED_SIG_PIN        18     // LED Strip Indicator Signal

// ---------------------------------------------------------------------------
// Liquid Level Sensor
// ---------------------------------------------------------------------------
#define ADC_PIN            23     // ADC Pin for Liquid Level Sensor

// ---------------------------------------------------------------------------
// Moteck LD3 Linear Actuator (Potentiometer & Motor)
// ---------------------------------------------------------------------------
#define M_POT_PIN          34
#define MD1_PWM_PIN        26
#define MD1_PWM_DIR        22

// ---------------------------------------------------------------------------
// JY300 Water Pump Motor
// ---------------------------------------------------------------------------
#define MD2_PWM_PIN        19

// ---------------------------------------------------------------------------
// CWX-15Q Motorized Valves Relay Control
// ---------------------------------------------------------------------------
#define REL_IN1_PIN        25
#define REL_IN2_PIN        33
#define REL_IN3_PIN        32

// ---------------------------------------------------------------------------
// K24 Flow Sensor RS485 Communication
// ---------------------------------------------------------------------------
#define TTL_DRE_PIN        15
#define TTL_DI_PIN         21
#define TTL_RO_PIN         36

#endif