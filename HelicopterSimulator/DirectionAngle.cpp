#include "DirectionAngle.h"

DirectionAngle::DirectionAngle(double rotation, double x, double y, double z) {
	Rotation = rotation;
	Direction = Vector3D(x, y, z);
}

DirectionAngle::DirectionAngle(double rotation, Vector3D direction) {
	Rotation = rotation;
	Direction = direction;
}

String DirectionAngle::ToString() {
	String r = Mathematics::DoubleToCleanString(Rotation);
	String x = Mathematics::DoubleToCleanString(Direction.X);
	String y = Mathematics::DoubleToCleanString(Direction.Y);
	String z = Mathematics::DoubleToCleanString(Direction.Z);

	return r + ": [" + x + " " + y + " " + z + "]";
}
