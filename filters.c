#include "filters.h"
#include "globals_and_gpio.h"
#include "timers.h"

// Generic filter function to check GPIO stability
bool gpio_stability_filter(uint32_t duration_ms, uint8_t threshold_percentage, uint32_t gpio_pin)
{
    uint32_t startTime = getSystemTime();
    uint32_t highCount = 0;
    uint32_t sampleCount = 0;

    while ((getSystemTime() - startTime) < duration_ms)
    {
        sampleCount++;
        if (GPIO_readPin(gpio_pin)) // Check if the given GPIO pin is high
        {
            highCount++;
        }
    }

    // Calculate the percentage of time the GPIO pin was high
    uint8_t percentage = (highCount * 100) / sampleCount;

    return (percentage >= threshold_percentage);
}
