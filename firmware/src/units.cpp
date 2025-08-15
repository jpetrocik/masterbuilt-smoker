#include "config.h"

int units_toLocalTemperature(double value)
{
#ifndef FERINHEIT
    return value;
#endif
#ifdef FERINHEIT
    return value * 1.8 + 32;
#endif
}

double units_fromLocalTemperature(double value)
{
#ifndef FERINHEIT
  return value;
#endif
#ifdef FERINHEIT
  return (value - 32) / 1.8;
#endif
}
