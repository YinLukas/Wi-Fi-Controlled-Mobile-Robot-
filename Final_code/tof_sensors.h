#pragma once
#include <Wire.h>
#include "VL53L1X.h"

void initToF();
void updateToF();

extern int tof[2];   // tof[0] ~ tof[2]