
class CurrentController
{
public:
  CurrentController(double *, double *, double *, double);

  void compute();
  void set_output_limits(double, double); // * clamps the output to a specific range. 0-255 by default

private:
  // Proportional tuning parameter
  double kp_;

  // Pointers to the input, output, and target variables
  // This creates a hard link between the variables and the controller
  double *input_;
  double *output_;
  double *target_;
  
  double out_min;
  double out_max;
};
