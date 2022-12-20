#include "Car.h"
#include "../../RLConst.h"

// Update our internal state from bullet and return it
CarState Car::GetState() {

	btTransform rbTransform = _rigidBody->getWorldTransform();

	_internalState.pos = rbTransform.getOrigin() * BT_TO_UU;

	rbTransform.getRotation().getEulerZYX(_internalState.angles.yaw, _internalState.angles.pitch, _internalState.angles.roll);

	_internalState.vel = _rigidBody->getLinearVelocity() * BT_TO_UU;

	_internalState.angVel = _rigidBody->getAngularVelocity();

	return _internalState;
}

// Update our bullet stuff to this new state, replace our internal state with it
void Car::SetState(const CarState& state) {
	btTransform rbTransform = _rigidBody->getWorldTransform();

	rbTransform.setOrigin(state.pos * UU_TO_BT);

	btQuaternion quat;
	quat.setEulerZYX(state.angles.yaw, state.angles.pitch, state.angles.roll);
	rbTransform.setRotation(quat);

	_rigidBody->setWorldTransform(rbTransform);

	_rigidBody->setLinearVelocity(state.vel * UU_TO_BT);
	_rigidBody->setAngularVelocity(state.angVel);

	_internalState = state;
}

void Car::_PreTickUpdate() {
	assert(_bulletVehicle->getNumWheels() == 4);

	// Prevent the car's RB from becoming inactive
	_rigidBody->setActivationState(ACTIVE_TAG);

	_bulletVehicle->updateVehicle(TICKTIME);

	float forwardSpeed = _bulletVehicle->getForwardSpeed();
	float absForwardSpeed = abs(forwardSpeed);

	bool jumpPressed = controls.jump && !_internalState.lastControls.jump;

	int numWheelsInContact = 0;
	for (int i = 0; i < 4; i++)
		numWheelsInContact += _bulletVehicle->m_wheelInfo[i].m_raycastInfo.m_isInContact;

	{ // Increase/decrease handbrake value from input
		if (controls.handbrake) {
			_internalState.handbrakeVal += RLConst::POWERSLIDE_RISE_RATE * TICKTIME;
		} else {
			_internalState.handbrakeVal -= RLConst::POWERSLIDE_FALL_RATE * TICKTIME;
		}
		_internalState.handbrakeVal = CLAMP(_internalState.handbrakeVal, 0, 1);
	}

	{ // Update steering
		float absForwardSpeedUU = absForwardSpeed * BT_TO_UU;
		float steerAngle = RLConst::STEER_ANGLE_FROM_SPEED_CURVE.GetOutput(absForwardSpeedUU);

		if (_internalState.handbrakeVal) {
			steerAngle +=
				(RLConst::POWERSLIDE_STEER_ANGLE_FROM_SPEED_CURVE.GetOutput(absForwardSpeedUU) - steerAngle)
				* _internalState.handbrakeVal;
		}

		steerAngle *= controls.steer;
		_bulletVehicle->m_wheelInfo[0].m_steerAngle = steerAngle;
		_bulletVehicle->m_wheelInfo[1].m_steerAngle = steerAngle;
	}

	{ // Update friction
		for (int i = 0; i < 4; i++) {
			auto& wheel = _bulletVehicle->m_wheelInfo[i];
			if (wheel.m_raycastInfo.m_groundObject) {

				Vec
					vel = _rigidBody->getLinearVelocity(),
					angularVel = _rigidBody->getAngularVelocity();

				Vec
					latDir = wheel.m_worldTransform.getBasis().getColumn(1),
					longDir = latDir.cross(wheel.m_raycastInfo.m_contactNormalWS);

				float frictionCurveInput = 0;

				Vec wheelDelta = wheel.m_raycastInfo.m_hardPointWS - _rigidBody->getWorldTransform().getOrigin();

				auto crossVec = (angularVel.cross(wheelDelta) + vel) * BT_TO_UU;

				float baseFriction = abs(crossVec.dot(latDir));

				// Significant friction results in lateral slip
				if (baseFriction > 5)
					frictionCurveInput = baseFriction / (abs(crossVec.dot(longDir)) + baseFriction);

				float latFriction = RLConst::LAT_FRICTION_CURVE.GetOutput(frictionCurveInput);
				float longFriction = RLConst::LONG_FRICTION_CURVE.GetOutput(frictionCurveInput);

				if (_internalState.handbrakeVal) {
					float handbrakeAmount = _internalState.handbrakeVal;

					latFriction *= (RLConst::HANDBRAKE_LAT_FRICTION_FACTOR_CURVE.GetOutput(latFriction) - 1) * handbrakeAmount + 1;
					longFriction *= (RLConst::HANDBRAKE_LONG_FRICTION_FACTOR_CURVE.GetOutput(frictionCurveInput) - 1) * handbrakeAmount + 1;
				} else {
					longFriction = 1; // If we aren't powersliding, it's not scaled down
				}

				bool isContactSticky = controls.throttle != 0;

				if (isContactSticky) {
					// Keep current friction values
				} else {
					// Scale friction down with non-sticky friction curve
					float nonStickyScale = RLConst::NON_STICKY_FRICTION_FACTOR_CURVE.GetOutput(wheel.m_raycastInfo.m_contactNormalWS.z());
					latFriction *= nonStickyScale;
					longFriction *= nonStickyScale;
				}

				wheel.m_latFriction = latFriction;
				wheel.m_longFriction = longFriction;
			}
		}
	}

	{ // Update throttle/brake forces
		float driveSpeedScale = RLConst::DRIVE_SPEED_TORQUE_FACTOR_CURVE.GetOutput(absForwardSpeed * BT_TO_UU);

		float realThrottle = controls.throttle;
		float realBrake = 0;

		if (controls.handbrake > 0) {
			// Real throttle is unchanged from the input throttle when powersliding
		} else {
			float absThrottle = abs(controls.throttle);

			if (absThrottle >= RLConst::THROTTLE_DEADZONE) {
				if (absForwardSpeed > 0 && SGN(controls.throttle) != SGN(forwardSpeed)) {
					// Full brake is applied if we are trying to drive in the opposite direction
					realBrake = 1;

					if (absForwardSpeed > 0.01f) {
						// Kill actual throttle (we can't throttle and break at the same time, even backwards)
						realThrottle = 0;
					}
				}
			} else {
				// No throttle, we are coasting
				realThrottle = 0;

				// Apply coasting brake, we full-break when coasting very slowly
				bool shouldFullStop = (absForwardSpeed < (RLConst::STOPPING_FORWARD_VEL* UU_TO_BT));
				realBrake = shouldFullStop ? 1 : RLConst::COASTING_BRAKE_FACTOR;
			}
		}

		if (numWheelsInContact < 3)
			driveSpeedScale /= 4;

		float driveEngineForce = realThrottle * (RLConst::THROTTLE_TORQUE_AMOUNT * UU_TO_BT) * driveSpeedScale;
		float driveBrakeForce = realBrake * (RLConst::BRAKE_TORQUE_AMOUNT * UU_TO_BT);
		for (int i = 0; i < 4; i++) {
			_bulletVehicle->m_wheelInfo[i].m_engineForce = driveEngineForce;
			_bulletVehicle->m_wheelInfo[i].m_brake = driveBrakeForce;
		}
	}

	if (numWheelsInContact >= 3) { // Grounded, apply sticky forces
		Vec downwardsDir = _bulletVehicle->getDownwardsDirFromWheelContacts();

		bool sticky = controls.throttle != 0 || absForwardSpeed > 25;
		float stickyForceScale = sticky ? (-abs(downwardsDir.z()) + 1.5f) : 0.5f;

		_rigidBody->applyImpulse(downwardsDir * stickyForceScale * -RLConst::GRAVITY_Z * UU_TO_BT, btVector3(0, 0, 0));

	} else { // Not grounded, apply air control
		using namespace RLConst;

		auto basis = _rigidBody->getWorldTransform().getBasis();
		btVector3
			pitchDir = -basis.getColumn(1),
			yawDir = basis.getColumn(2),
			rollDir = -basis.getColumn(0);

		// Net torque to apply to the car
		btVector3 torque;

		if (controls.handbrake) {
			std::swap(controls.roll, controls.yaw);
		}

		if (controls.pitch || controls.yaw || controls.roll) {
			torque =
				controls.pitch * pitchDir * CAR_AIR_CONTROL_TORQUE.x() +
				controls.yaw * yawDir * CAR_AIR_CONTROL_TORQUE.y() +
				controls.roll * rollDir * CAR_AIR_CONTROL_TORQUE.z();
		} else {
			torque = { 0, 0, 0 };
		}

		auto angVel = _rigidBody->getAngularVelocity();
		float
			dampPitch = pitchDir.dot(angVel) * CAR_AIR_CONTROL_DAMPING.x() * (1 - abs(controls.pitch)),
			dampYaw = yawDir.dot(angVel) * CAR_AIR_CONTROL_DAMPING.y() * (1 - abs(controls.yaw)),
			dampRoll = rollDir.dot(angVel) * CAR_AIR_CONTROL_DAMPING.z();

		btVector3 damping =
			(yawDir * dampYaw) +
			(pitchDir * dampPitch) +
			(rollDir * dampRoll);

		_rigidBody->setAngularVelocity(
			_rigidBody->getAngularVelocity() + (torque - damping) * CAR_TORQUE_SCALE * TICKTIME
		);
	}
}

void Car::_ApplyPhysicsRounding() {
	_internalState = GetState();

	_internalState.pos = Math::RoundVec(_internalState.pos, 0.01);
	_internalState.vel = Math::RoundVec(_internalState.vel, 0.01);
	_internalState.angVel = Math::RoundVec(_internalState.angVel, 0.00001);
	
	SetState(_internalState);
}

void Car::_PostTickUpdate() {
	{ // Update isOnGround
		int wheelsWithContact = 0;
		for (int i = 0; i < 4; i++)
			wheelsWithContact += _bulletVehicle->m_wheelInfo[i].m_raycastInfo.m_isInContact;

		_internalState.isOnGround = wheelsWithContact >= 3;
	}

	{ // Update jump
		using namespace RLConst;
		if (_internalState.isOnGround && !_internalState.isJumping)
			_internalState.hasJumped = false;

		if (_internalState.isJumping) {
			if (controls.jump) { // Jump held
				if (_internalState.jumpTimer <= JUMP_MAX_TIME) {
					// Continue jump
				} else {
					_internalState.isJumping = false;
				}
			} else { // Jump released
				// We must remain jumping for at least JUMP_MIN_TIME
				_internalState.isJumping = _internalState.jumpTimer <= JUMP_MIN_TIME;
			}
		} else if (_internalState.isOnGround && controls.jump && !_internalState.lastControls.jump) {
			// Start jumping
			_internalState.isJumping = true;
			_internalState.jumpTimer = 0;
			btVector3 jumpStartForce = _bulletVehicle->getUpVector() * JUMP_IMMEDIATE_FORCE * UU_TO_BT;
			_rigidBody->applyCentralImpulse(jumpStartForce * CAR_MASS_BT);
		}

		if (_internalState.isJumping) {
			_internalState.jumpTimer += TICKTIME;

			if (_internalState.jumpTimer > JUMP_MIN_TIME) {
				// Apply extra long-jump force
				btVector3 extraJumpForce = _bulletVehicle->getUpVector() * JUMP_ACCEL * UU_TO_BT * TICKTIME;
				_rigidBody->applyCentralImpulse(extraJumpForce * CAR_MASS_BT);
			}
		} else {
			_internalState.jumpTimer = 0;
		}
	}

	{ // Update flip/double jump
		using namespace RLConst;
		if (_internalState.isOnGround) {
			_internalState.hasDoubleJumped = false;
			_internalState.hasFlipped = false;
		} else {
			// TODO: ...
		}
	}
	_internalState.lastControls = controls;

	{ // Limit velocities
		using namespace RLConst;

		if (_rigidBody->getLinearVelocity().length2() > (CAR_MAX_SPEED * CAR_MAX_SPEED))
			_rigidBody->setLinearVelocity(_rigidBody->getLinearVelocity().normalized() * RLConst::CAR_MAX_SPEED);

		if (_rigidBody->getAngularVelocity().length2() > (CAR_MAX_ANG_SPEED * CAR_MAX_ANG_SPEED))
			_rigidBody->setAngularVelocity(_rigidBody->getAngularVelocity().normalized() * CAR_MAX_ANG_SPEED);
	}
}

Car* Car::_AllocateCar() {
	return new Car();
}

Car::~Car() {
	delete _bulletVehicle;
	delete _bulletVehicleRaycaster;
	delete _rigidBody;
	delete _compoundShape;
	delete _childHitboxShape;
}