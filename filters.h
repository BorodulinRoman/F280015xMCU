#ifndef FILTERS_H_
#define FILTERS_H_

#include <stdint.h>
#include <stdbool.h>
#include "device.h"  // For GPIO-related function
// Function Prototypes
bool gpio_stability_filter(uint32_t duration_ms, uint8_t threshold_percentage, uint32_t gpio_pin);

#endif // FILTERS_H_
