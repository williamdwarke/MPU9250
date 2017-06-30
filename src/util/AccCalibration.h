void take_sample(long* sample_out);
void calibrate_model_matrices();
void find_delta();
void calibrate_model();
void reset_calibration_matrices();

extern long *data;
extern int samp_capacity;
extern int n_samp;

//Calibration parameters
extern double beta[6];
