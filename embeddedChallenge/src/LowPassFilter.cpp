// LowPassFilter.cpp
#include "LowPassFilter.h"

LowPassFilter::LowPassFilter(float alpha) : alpha(alpha), lastFilteredX(0) {}

void LowPassFilter::update(float rawX) {
    lastFilteredX = alpha * rawX + (1 - alpha) * lastFilteredX;
}

float LowPassFilter::getFiltered() const { return lastFilteredX; }