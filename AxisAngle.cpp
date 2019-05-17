#include "AxisAngle.h"

AxisAngle::AxisAngle(double rotation, double x, double y, double z) {
	Rotation = rotation;
	Axis = Vector3D(x, y, z);
}

AxisAngle::AxisAngle(double rotation, Vector3D axis) {
	Rotation = rotation;
	Axis = axis;
}

String AxisAngle::ToString() {
	String r = Mathematics::DoubleToCleanString(Rotation);
	String x = Mathematics::DoubleToCleanString(Axis.X);
	String y = Mathematics::DoubleToCleanString(Axis.Y);
	String z = Mathematics::DoubleToCleanString(Axis.Z);

	return r + ": [" + x + " " + y + " " + z + "]";
}
