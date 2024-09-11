class PID {
public :
	PID(double kp, double ki, double kd, double alpha = 0.1) : Kp(kp), Ki(ki), Kd(kd), alpha(alpha), previous(0), integral(0), smoothedOutput(0) {}

	double calc(double error) {
		double proportional = error;                   //defining proportional error for calculations
		integral += error * dt;                        //integral error over time dt is defined in void loop which is the 
		double derivative = (error - previous) / dt;   //drevative error calculated by rate of change
		previous = error;                              //overwriting old reading for further calculations
		
		return  (kp * proportional) + (ki * integral) + (kd * derivative);
    }
	double filter(double rawOutput) {

		smoothedOutput = alpha * rawOutput + (1 - alpha) * smoothedOutput;
		return smoothedOutput;
	}

private:
	double Kp, Ki, Kd; //proportional integral drevative multiiplying factors
	double previous, integral; 
	double alpha;  // Smoothing factor
	double smoothedOutput;
};

double dt, last_time;
double output = 0;
double setpoint = 75.00;
#define motorPin 9
#define reading A0
// Create a PID instance
PID pid(2.0, 5.0, 1.0, 0.1);

void setup() {
	pinMode(motorPin, OUTPUT);

}
void loop() {
	// Read the current speed (input)
	double input = analogRead(A0);

	// Compute the PID output
	double rawOutput = pid.calc(input);

	// Apply the smoothing filter
	double filteredOutput = pid.filter(rawOutput);

	// Constrain the output to be within PWM limits (0-255)
	filteredOutput = constrain(filteredOutput, 0, 255);

	// Set the motor speed
	analogWrite(motorPin, filteredOutput);

	delay(10);  // Short delay for stability
}