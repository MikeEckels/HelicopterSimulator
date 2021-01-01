#include "Mathematics.h"

float Mathematics::Constrain(float v, float minimum, float maximum) {
	if (v > maximum)
	{
		v = maximum;
	}
	else if (v < minimum)
	{
		v = minimum;
	}

	return v;
}

double Mathematics::DegreesToRadians(double degrees) {
	return degrees / (180.0 / PI);
}

double Mathematics::RadiansToDegrees(double radians) {
	return radians * (180.0 / PI);
}

String Mathematics::DoubleToCleanString(double value) {
	return String(value, 4);
}

void Mathematics::CleanPrint(int values, ...) {
  va_list valueList;
	String printOut = "";
  va_start(valueList, values);

  for (int i = 0; i <= values; i++) {
    double value = va_arg(valueList, double);
    printOut += Mathematics::DoubleToCleanString(value);
  }

  va_end(valueList);

	Serial.println(printOut);
}

bool Mathematics::IsNaN(double value) {
	return value != value;
}

bool Mathematics::IsInfinite(double value) {
	return isinf(value);
}

bool Mathematics::IsFinite(double value) {
	return !isinf(value);
}

int Mathematics::Sign(double value) {
		return (0 < value) - (value < 0);
}
