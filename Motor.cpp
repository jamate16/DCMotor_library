#include "Motor.h"

DCMotor::DCMotor(float max_speed, float gear_ratio, float gear_ratio_correction_factor)
    : MAX_SPEED {max_speed}, gear_box(gear_ratio), encoder{nullptr}, driver{nullptr} {
    gear_box.changeCorrectionFactor(gear_ratio_correction_factor); /// \todo Should be in the construtor of gear_box object

    position_sp = 0;
    velocity_sp = 0;
}

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
    /// \todo Implement anti-windup in the controller, not here. But for now this is fine.
    // Raw control signal clamping. This allows wind-up in the controler's signal calculation
    if (vel_sp > MAX_SPEED){
        velocity_sp = MAX_SPEED;
    } else if (vel_sp < -MAX_SPEED) {
        velocity_sp = -MAX_SPEED;
    } else {
        velocity_sp = vel_sp;
    }
}

/// \todo review all this function, there are things that should go in other files or refactored
void DCMotor::calculateControl() {
    float sp;
    float pv;

    switch (control_type) {
        case MotorControlType::position:
            sp = position_sp;
            pv = position;
            break;
        case MotorControlType::velocity:
            sp = velocity_sp;
            pv = velocity;
            break;
    }

    float control_signal = controller->calculateControl(sp-pv);

    /// \todo This part should be implemented in the encoder class
    // Prepare values for passing onto the motor driver
    float dc_setpoint = control_signal;
    MotorDir motor_dir {MotorDir::cw};
    if (control_signal < 0) {
        motor_dir = MotorDir::ccw;
        dc_setpoint *= -1;
    }

    // To avoid dead-zone of the motor
    dc_setpoint = ((100-39.0)/100)*dc_setpoint + 39.0;

    // Inject enough voltage to excite rotor when motor has stopped and still isn't at desired position. This is enough to make the rotor move again.
    if (velocity == 0) dc_setpoint = 63;        

    driver->setPWMsDC(motor_dir, dc_setpoint);
}

void DCMotor::setControlType(MotorControlType cont_type) {
    control_type = cont_type;
}

void DCMotor::linkEncoder(Encoder *_encoder) {
    encoder = _encoder;
}

void DCMotor::linkDriver(DCMotorDriver *_driver) {
    driver = _driver;
}

// Keep in mind the controller type should correspond with the type of control specified for the motor
void DCMotor::linkController(PIDController *_controller) {
    controller = _controller;
}

float DCMotor::getPosition() {return position; };
float DCMotor::getVelocity() {return velocity; };
