#include "MST.hpp"
#include <chrono>
#include <iostream>

typedef std::chrono::high_resolution_clock Clock;

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
	double avg_error = 0;
	double max_error = 0;
	const Clock::time_point t_start = Clock::now();
	for (size_t i = 0; i < n; ++i) {
		// select test data
		const MST::Vector sensor_output = y.col(i);
		const MST::Vector expected_input = u.col(i);

		// run controller
		const MST::Vector control_input =
		    controller(t, reference_output(t), sensor_output);

		// compare with test data
		const double error = (control_input - expected_input).norm();
		avg_error += error;
		if (error > max_error) {
			max_error = error;
		}

		t += dt;
	}
	const Clock::time_point t_stop = Clock::now();
	const std::chrono::duration<double, std::micro> t_avg =
	    (t_stop - t_start) / n;
	const double Dt = t_avg.count();
	avg_error /= n;

	std::cout << "Avg. error: " << avg_error << std::endl;
	std::cout << "Max. error: " << max_error << std::endl;
	std::cout << "Avg. time per iteration: " << Dt << "µs" << std::endl;
}