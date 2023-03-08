#include "Motor.h"

DCMotor::DCMotor(float max_speed, float gear_ratio)
    : MAX_SPEED {max_speed}, gear_box(gear_ratio), encoder{nullptr}, driver{nullptr} {}

void DCMotor::updateStates() {
    encoder->calculateStates();
    position = gear_box.correction_factor * (encoder->getPosition()) / gear_box.ratio;
    velocity = gear_box.correction_factor * (encoder->getVelocity()) / gear_box.ratio;
}

void DCMotor::changePositionSetPoint(float pos_sp) {
    /// \todo add clauses that check if the value is posible.
    position_sp = pos_sp;
}

void DCMotor::changeVelocitySetPoint(float vel_sp) {
    /// \todo Implement anti-windup
    // Raw control signal clamping. This allows wind-up in the controler's signal calculation
    if (vel_sp > MAX_SPEED){
        velocity_sp = MAX_SPEED;
    } else if (vel_sp < -MAX_SPEED) {
        velocity_sp = -MAX_SPEED;
    } else {
        velocity_sp = vel_sp;
    }
}

void DCMotor::calculateControl() {
    switch (control_type) {
        case ControlType::position:
            
            break;
        case ControlType::velocity:
            
            break;
    }
}

void DCMotor::linkEncoder(Encoder *_encoder) {
    encoder = _encoder;
}

void DCMotor::linkDriver(DCMotorDriver *_driver) {
    driver = _driver;
}




    

