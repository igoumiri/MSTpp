#include "MST.hpp"

extern MST::Vector acquireSensorOutput();
extern void sendControlInput(const MST::Vector &);

int main(int argc, char *argv[]) {
	// check command line arguments
	if (argc < 2) {
		std::cerr << "Usage: " << argv[0] << " <config.toml>" << std::endl;
		return 1;
	}

	// parse config file
	MST::Config config(argv[1]);

	// initialize the reference signal, the converters, and the controller
	MST::ReferenceOutput reference_output(config["reference"]);
	MST::InputConverter input_converter(config["input"]);
	MST::Controller base_controller(config["control"]);
	MST::OutputConverter output_converter(config["output"]);

	// connect them together
	MST::System &controller = base_controller >> output_converter;

	// load time parameters
	const double t0 = config["time"].getDouble("t0");
	const double tf = config["time"].getDouble("tf");
	const double dt = config["time"].getDouble("dt");

	// example usage in a loop
	for (double t = t0; t < tf; t += dt) {
		// acquire sensor output
		const MST::Vector sensor_output = acquireSensorOutput();

		// convert reference and sensor_output
		const MST::Vector r = input_converter(t, reference_output(t));
		const MST::Vector y = input_converter(t, sensor_output);

		// run controller
		const MST::Vector control_input = controller(t, r, y);

		// send control input to actuators
		sendControlInput(control_input);
	}
}