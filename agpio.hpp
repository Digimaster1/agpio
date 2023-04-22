// ***************************************************************************************
// agpio.hpp - single file, header only c++ gpio library
// for Odroid boards N2Plus, XU4
// Odroid wiki and Tomek Szczesny, Bernhard Bablok code based.
// Author: Digimaster
// License: GPL3
// See odroid wiki for register map
// N2x boards: https://wiki.odroid.com/odroid-n2/software/gpio_register_map
// XU4 boards: https://wiki.odroid.com/odroid-xu4/application_note/gpio/memory_mapped_gpio
// ***************************************************************************************
// main.cpp:
/* 
#include <iostream>
#include "agpio.hpp"

int main(int argc, char *argv[])
{
    if (!gpioInit()) {
        std::cout << "GPIO MMAP Cannot init, (sudo?)" << std::endl;
        return EXIT_FAILURE;
    }

    // N2Plus blinking example
    // setPinToOutput(PIN_33);
    // while (true) {
    //    clearPin(PIN_33);
    //    usleep(1000000);
    //    setPin(PIN_33);
    //    usleep(1000000);
    // }

    // N2Plus normally open pin has a high level of 3.3v. Short pin33 to pin34(GND) to get 0. Please be alert with circuits!
    setPinToInput(PIN_33);
    while (true) {
        std::cout << (int) gpioRead(PIN_33);
        std::fflush(stdout);
        usleep(100000);
        std::cout << "\r";
    }

    std::cout << std::endl;
    return EXIT_SUCCESS;
}
*/
#ifndef AGPIO_H
#define AGPIO_H

#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <chrono>
#include <sys/select.h>

#define N2PLUS
//#define XU4

#if defined(N2PLUS)
// N2Plus GPIOX chip registers
#define INP_OFFS    0x118
#define OUT_OFFS    0x117
#define SEL_OFFS    0x116
#define REG_MAP     0xFF634000
#define PULLEN_OFFS 0x4A
#define PULLST_OFFS 0x3C
// pins
#define PIN_35      6
#define PIN_33      5
#define PIN_31      15
#define PIN_29      14
// define more pins here ...
#elif defined(XU4)
// Some of XU4 registers
#define REG_MAP     0x13400000
#define GPX1_OFFS   0x0C20
#define GPX2_OFFS   0x0C40
#define GPX3_OFFS   0x0C60
#define DATA_OFFS   0x0004
// pins
#define PIN_20      4
#define PIN_21      5
#define PIN_25      7
#define PIN_15      2
// define more pins here ...
#endif



static volatile uint32_t *gpiomap;

bool gpioInit()
{
    int gpiomem = open("/dev/mem", O_RDWR | O_SYNC);
    if (gpiomem >= 0) {
        gpiomap = static_cast<volatile uint32_t *>(mmap(NULL, 4096, (PROT_READ | PROT_WRITE), MAP_SHARED, gpiomem, REG_MAP));
        close(gpiomem);
        return gpiomap >= 0;
    }
    return false;
}

#if defined(N2PLUS)

inline void setPinToOutput(uint8_t pin)
{
    *(gpiomap + SEL_OFFS) &= ~(1 << pin);
    __sync_synchronize();
}

inline void setPinToInput(uint8_t pin)
{
    *(gpiomap + SEL_OFFS) |= (1 << pin);
    __sync_synchronize();
}

inline void setPin(uint8_t pin)
{
    *(gpiomap + OUT_OFFS) |= (1 << pin); // Set pin high
}

inline void clearPin(uint8_t pin)
{
    *(gpiomap + OUT_OFFS) &= ~(1 << pin); // Set pin low
}

inline bool gpioRead(uint8_t pin)
{
    return  (*(gpiomap + INP_OFFS) & (1 << pin)) != 0;
}

// As far as I know, the n2plus board presumably has all its pins hardware pulled up or down, see wiki.
// Anyhow this func has no effect on N2Plus board, for N2 it's not tested yet.
// highPull = true for pull up level to high, highPull = false for pull up level to low.
void setPullMode(const bool highPull, uint8_t pin)
{
    *(gpiomap + PULLEN_OFFS) |= (1 << pin);
    if (highPull)
        *(gpiomap + PULLST_OFFS) |= (1 << pin); // set pull up to high
    else
        *(gpiomap + PULLST_OFFS) &= ~(1 << pin); // set pull up to low  
    __sync_synchronize();
    //std::cout << "pull " << *(gpiomap + PULLST_OFFS) << std::endl;
}

void clearPullMode(uint8_t pin)
{
    *(gpiomap + PULLEN_OFFS) &= ~(1 << pin);
}

#elif defined(XU4)


uint32_t getOffset(uint8_t pin)
{
    uint32_t SEL_OFFS = 0;
    if (pin == PIN_20) SEL_OFFS = GPX2_OFFS;
    else if (pin == PIN_21) SEL_OFFS = GPX2_OFFS;
    else if (pin == PIN_25) SEL_OFFS = GPX1_OFFS;
    else if (pin == PIN_15) SEL_OFFS = GPX1_OFFS;
    return SEL_OFFS;
}

uint32_t getBit(uint8_t pin)
{
    uint32_t bit = 0;
    if (pin == PIN_20) bit = 16;
    else if (pin == PIN_21) bit = 20;
    else if (pin == PIN_25) bit = 28;
    else if (pin == PIN_15) bit = 8;
    return bit;
}

inline void setPinToOutput(uint8_t pin)
{
    uint32_t SEL_OFFS = getOffset(pin);
    if (SEL_OFFS == 0) return;
    *(gpiomap + (SEL_OFFS >> 2)) |= (0x1 << getBit(pin));
    __sync_synchronize();
}

inline void setPinToInput(uint8_t pin)
{
    uint32_t SEL_OFFS = getOffset(pin);
    if (SEL_OFFS == 0) return;
    *(gpiomap + (SEL_OFFS >> 2)) &= ~(0x1 << getBit(pin));
    __sync_synchronize();
}

inline void setPin(uint8_t pin)
{
    uint32_t SEL_OFFS = getOffset(pin) + DATA_OFFS;
    *(gpiomap + (SEL_OFFS >> 2)) |= (0x1 << pin);
}

inline void clearPin(uint8_t pin)
{
    uint32_t SEL_OFFS = getOffset(pin) + DATA_OFFS;
    *(gpiomap + (SEL_OFFS >> 2)) &= ~(0x1 << pin);
}

inline bool gpioRead(uint8_t pin)
{
    uint32_t SEL_OFFS = getOffset(pin) + DATA_OFFS;
    return (*(gpiomap + (SEL_OFFS >> 2)) & (0x1 << pin)) != 0;
}

// highPull = true for pull up level to high, highPull = false for pull up level to low.
void setPullMode(const bool highPull, uint8_t pin)
{
    uint32_t SEL_OFFS = getOffset(pin) + 0x0008;
    if (highPull)
        *(gpiomap + (SEL_OFFS >> 2)) |= (0x3 << (2*pin));
    else
        *(gpiomap + (SEL_OFFS >> 2)) |= (0x1 << (2*pin));
}

void clearPullMode(uint8_t pin)
{
    uint32_t SEL_OFFS = getOffset(pin) + 0x0008;
    *(gpiomap + (SEL_OFFS >> 2)) &= ~(0x3 << (2*pin));
}

#endif

#endif