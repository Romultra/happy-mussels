// my_functions.h
#ifndef UTILS_H
#define UTILS_H


/// GPIO pins for pump driver channels (visible to entire sketch)
extern const int C1_pin;
extern const int C2_pin;

/// Call once from setup() to configure PWM on C1/C2
void initPumpPWM();

/// Drive the pump at the given percentage (60.0–100.0% safe range).
/// Prints a warning if % is outside [60.0, 100.0].
/// Uses floating point for finer duty cycle control.
bool setPumpIntensity(double intensityPercent);

/// Drive the pump with 8-bit speed control (0-255).
/// Maps 0-255 to the pump's working range (60-100% duty cycle).
/// 0 = pump off, 255 = full speed.
bool setPumpSpeed(int speed8Bit);


/// Read the thermistor temperature in Celsius (uses lookup table and Steinhart-Hart)
float readThermistorTemp();

/// Read the thermistor resistance in ohms (uses lookup table)
float readThermistorResistance();


#endif