#include "MST.hpp"
#include <cstdlib>
#include <iostream>

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
	MST::System &controller =
	    input_converter >> base_controller >> output_converter;

	// load test data
	const double t0 = config["time"].getDouble("t0");
	const double tf = config["time"].getDouble("tf");
	const double dt = config["time"].getDouble("dt");
	const std::string so_dat = config["example"].getString("sensor_output");
	const std::string ei_dat = config["example"].getString("expected_input");
	const MST::Matrix y = MST::Utils::load_dat<MST::Matrix>(so_dat).transpose();
	const MST::Matrix u = MST::Utils::load_dat<MST::Matrix>(ei_dat).transpose();
	const size_t n = y.cols();

	// example usage in a loop
	double t = t0;
	for (size_t i = 0; i < n && t < tf; ++i) {
		const MST::Vector sensor_output = y.col(i);
		const MST::Vector expected_input = u.col(i);

		const MST::Vector control_input =
		    controller(t, reference_output(t), sensor_output);

		// compare with test data
		const double error = (control_input - expected_input).norm();
		if (error > 1e-3) {
			printf("Step %4zu [t = %.5f]: error = %f\n", i, t, error);
		}

		t += dt;
	}
}