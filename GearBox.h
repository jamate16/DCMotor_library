#pragma once

class GearBox {
public: // Public attributes
    float ratio;
    float correction_factor; /// Due to mechanical fenomena the ratio isn't always the one specified on the motor's data sheet

public: // Public methods
    GearBox(float ratio);

    /**
     * @brief Set the correction factor for the gearbox ratio.
     * 
     * @param factor Calculated like: (real motor speed) / (estimated speed by microcontroller)
     */
    void changeCorrectionFactor(float factor);
};