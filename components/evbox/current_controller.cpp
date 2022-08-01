#include "current_controller.h"

CurrentController::CurrentController(double *input, double *output, double *target, double kp)
{
    output_ = output;
    input_ = input;
    target_ = target;
    kp_ = kp;

    out_min = 0;
    out_max = 255;
}

void CurrentController::compute()
{
    /*Compute all the working error variables*/
    double input = *input_;
    double error = *target_ - input;
    double output = input + kp_ * error;

    if (output > out_max)
        output = out_max;
    else if (output < out_min)
        output = out_min;

    *output_ = output;
}

void CurrentController::set_output_limits(double min, double max)
{
    if (min >= max)
        return;
    out_min = min;
    out_max = max;
}
