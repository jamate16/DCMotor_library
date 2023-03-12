#pragma once

#include "Encoder.h"
#include "Driver.h"
#include "GearBox.h"
#include "PID.h"

enum class MotorControlType {
    position,
    velocity
};

class DCMotor {
public: // Public attributes
    const float MAX_SPEED; /// Max speed of motor. Should be known at the time of DCMotor object creation

    float position_sp; /// in rad. Desired output shaft position
    float velocity_sp; /// in rad/s. Desired output shaft velocity

    GearBox gear_box;

    MotorControlType control_type;
    /*
    the objects below are created elsewhere, this is why they are implemented as pointers.
    In order to be used both inside of the this object and outside, where they were created,
    this way no errors are introduced by creating a copy of those objects
    */
    Encoder *encoder; // Pointer so that it can only point to Encoder objects
    DCMotorDriver *driver;
    PIDController *controller;

public: // Public methods
    /**
     * @brief Construct a new DCMotor object
     * 
     * @param max_speed Maximum speed of the output shaft of your motor
     * @param gear_ratio In case your motor has a gearbox
     */
    DCMotor(float max_speed, float gear_ratio=1.f, float gear_ratio_corr_factor=1.f);

    void setControlType(MotorControlType cont_type); /// \todo evaluate if init() is the best way to call this method
    void linkEncoder(Encoder *_encoder);
    void linkDriver(DCMotorDriver *_driver);
    void linkController(PIDController *_controller);

    void changePositionSetPoint(float pos_sp);
    void changeVelocitySetPoint(float vel_sp);
    void calculateControl();
    void updateStates();

    float getPosition();
    float getVelocity();


private:
    float position; /// Absolute angle of output shaft
    float velocity; /// Speed and direction of output shaft. [+]: CCW, [-]: CW
};