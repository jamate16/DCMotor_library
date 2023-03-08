#include "GearBox.h"

GearBox::GearBox(float ratio)
    : ratio {ratio}, correction_factor {1} {}

void GearBox::changeCorrectionFactor(float factor){
    correction_factor = factor;
}
