#include "BulletLink.h"

#include "Framework.h"
#include "Math/Math.h"
#include "Math\MathTypes\MathTypes.h"

Angle::Angle(btMatrix3x3 mat) {
	mat.getEulerYPR(yaw, pitch, roll);
	pitch *= -1;
	roll *= -1;
}

btMatrix3x3 Angle::ToMatrix() const {
	btMatrix3x3 mat;
	mat.setEulerYPR(yaw, -pitch, -roll);
	return mat;
}

Vec Angle::GetForwardVector() const {
	float
		cy = cosf(yaw),
		cp = cosf(-pitch),
		sp = sinf(-pitch);
	return Vec(cy * cp, cy * sp, -sp);
}

void Angle::NormalizeFix() {
	yaw		= Math::WrapNormalizeFloat(yaw, M_PI);
	pitch	= Math::WrapNormalizeFloat(pitch, M_PI / 2);
	roll	= Math::WrapNormalizeFloat(roll, M_PI);
}