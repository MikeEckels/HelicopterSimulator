#pragma once

#include <math.h>
#include <Arduino.h>

class Mathematics {
public:
	//static const double PI = atan(1) * 4;
  static constexpr double DOUBLE_EPSILON = pow(2, -1022);
  static constexpr double DOUBLE_MAX = pow(2,1024)-pow(2,971);

	static float Constrain(float v, float minimum, float maximum);
	static double DegreesToRadians(double degrees);
	static double RadiansToDegrees(double radians);
	static String DoubleToCleanString(double value);
	static void CleanPrint(int values, ...);
	static bool IsNaN(double);
	static bool IsInfinite(double);
	static bool IsFinite(double);
	static int Sign(double);
};
