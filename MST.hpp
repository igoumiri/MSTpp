#ifndef MST_CONTROLLER_HPP
#define MST_CONTROLLER_HPP 1

#include "linterp.h"
#include "toml.h"
#include <Eigen/Core>
#include <boost/numeric/odeint.hpp>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

namespace MST { // The MST namespace contains the classes used for control.

using std::string;
using namespace boost::numeric::odeint;
namespace pl = std::placeholders;

// Matrix and Vector are shortcuts for Eigen types
using Matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using Vector = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using Eigen::Vector2d;

namespace Utils {
template <typename T> T load_dat(const std::string &path) {
	std::ifstream indata(path);
	std::string line;
	std::vector<double> values;
	size_t rows = 0;
	while (std::getline(indata, line)) {
		std::stringstream lineStream(line);
		std::string cell;
		while (std::getline(lineStream, cell, ' ')) {
			values.push_back(std::stod(cell));
		}
		++rows;
	}
	return Eigen::Map<const T>(values.data(), rows, values.size() / rows);
}
} // namespace Utils

class Config {
public:
	Config(const char *filename) : Config(std::string(filename)) {}
	Config(const std::string &filename) {
		std::ifstream config_file(filename);
		toml::ParseResult parsed_config = toml::parse(config_file);

		// make sure config parsed correctly
		if (!parsed_config.valid()) {
			throw std::runtime_error(parsed_config.errorReason);
		}

		// extract control parameters
		m_config = parsed_config.value;
	}

	Config operator[](const std::string &path) {
		return Config(m_config[path]);
	}

	double getDouble(const std::string &name) const {
		return m_config.get<double>(name);
	}

	std::string getString(const std::string &name) const {
		return m_config.get<std::string>(name);
	}

	// Load a column vector from a TOML array.
	// All rows of the TOML array must have exactly one element.
	Vector getVector(const std::string &name) const {
		const toml::Array &rows = m_config.get<toml::Array>(name);
		const size_t n = rows.size();
		Vector vec(n);
		for (int i = 0; i < n; ++i) {
			const toml::Value row = rows[i];
			vec(i) = row.get<double>(0);
		}
		return vec;
	}

	// Load a matrix from a TOML array.
	// All rows of the TOML array must have the same length.
	Matrix getMatrix(const std::string &name) const {
		const toml::Array &rows = m_config.get<toml::Array>(name);
		const size_t n = rows.size();
		const size_t p = rows.at(0).size();
		Matrix mat(n, p);
		for (int i = 0; i < n; ++i) {
			const toml::Value row = rows.at(i);
			for (int j = 0; j < p; ++j) {
				mat(i, j) = row.get<double>(j);
			}
		}
		return mat;
	}

private:
	toml::Value m_config;

	Config(const toml::Value &config) : m_config(config) {}
};

class System {
public:
	template <typename T> System &operator>>(const T &next) {
		if (m_next) {
			*m_next >> next;
		} else {
			m_next = std::make_shared<T>(next);
		}
		return *this;
	}

	Vector operator()(const double t, const Vector &v) {
		const Vector output = step(t, v, Vector());
		if (m_next) {
			return (*m_next)(t, output, Vector());
		}
		return output;
	}

	Vector operator()(const double t, const Vector &ref, const Vector &actual) {
		const Vector output = step(t, ref, actual);
		if (m_next) {
			return (*m_next)(t, output, actual);
		}
		return output;
	}

private:
	std::shared_ptr<System> m_next;

	virtual Vector step(const double t, const Vector &ref,
	                    const Vector &actual) = 0;
};

class Controller : public System {
private:
	// Matrices of coefficients
	Matrix A;
	Matrix B;
	Matrix C;
	Matrix D;
	Matrix F;
	Matrix K;
	Matrix Ki;
	Matrix L;

	// Precalculated matrices
	Matrix AmLC;
	Matrix BmLD;

	// Desired values
	Vector ud;
	Vector xd;

	// Natural equilibrium
	Vector r0;
	Vector u0;

	// Saturation
	Vector u_min;
	Vector u_max;

	// State of the controller
	Vector xh;
	Vector xi;

	// Time step
	double dt;

public:
	Controller(const Config &config) {
		A = config.getMatrix("A");
		B = config.getMatrix("B");
		C = config.getMatrix("C");
		D = config.getMatrix("D");
		F = config.getMatrix("F");
		K = config.getMatrix("K");
		Ki = config.getMatrix("Ki");
		L = config.getMatrix("L");

		AmLC = A - L * C;
		BmLD = B - L * D;

		ud = config.getVector("ud");
		xd = config.getVector("xd");

		r0 = config.getVector("r0");
		u0 = config.getVector("u0");

		u_min = config.getVector("u_min");
		u_max = config.getVector("u_max");

		dt = config.getDouble("dt");

		xh.setZero(A.rows());
		xi.setZero(C.rows());
	}

	virtual Vector step(const double t, const Vector &r, const Vector &y) {
		if (r.rows() == 0 || r.cols() == 0) {
			return Vector();
		}

		Vector u = F * (r - r0) - K * xh - Ki * xi;

		// Saturation
		u = (u + u0).cwiseMax(u_min).cwiseMin(u_max) - u0;

		xh = AmLC * xh + BmLD * u + L * (y - r0);
		xi += dt * (y - r);

		return u + u0;
	}
};

class OutputConverter : public System {
public:
	OutputConverter(const Config &config) {
		m_coeff(0) = config.getDouble("coeff_phi");
		m_coeff(1) = config.getDouble("coeff_theta");
		m_R(0) = config.getDouble("R_phi");
		m_R(1) = config.getDouble("R_theta");
		m_L(0) = config.getDouble("L_phi");
		m_L(1) = config.getDouble("L_theta");
		m_dt = config.getDouble("dt");
		m_shot = Utils::load_dat<Matrix>(config.getString("shot_data"));
		m_count = 1;
	}

	virtual Vector step(const double, const Vector &V, const Vector &) {
		if (V.rows() == 0 || V.cols() == 0) {
			m_V = m_shot.row(m_count);
		} else {
			m_V = V;
		}
		m_stepper.do_step(std::bind(&OutputConverter::primaryCurrentDot, this,
		                            pl::_1, pl::_2, pl::_3),
		                  m_I, 0.0, m_dt);
		++m_count;

		return m_I;
	}

private:
	// state
	Vector2d m_I;
	Vector2d m_V;

	// parameters
	Vector2d m_coeff;
	Vector2d m_R;
	Vector2d m_L;
	double m_dt;
	Matrix m_shot;
	size_t m_count;

	runge_kutta_dopri5<Vector2d, double, Vector2d, double, vector_space_algebra>
	    m_stepper;

	void primaryCurrentDot(const Vector2d &I, Vector2d &dIdt,
	                       const double /*t*/) {
		dIdt(0) = (m_coeff(0) * m_V(0) - m_R(0) * I(0)) / m_L(0);
		dIdt(1) = (m_coeff(1) * m_V(1) - m_R(1) * I(1)) / m_L(1);
	}
};

class InputConverter : public System {
public:
	InputConverter(const Config &config) {
		Vector F(Utils::load_dat<Vector>(config.getString("F")));
		Vector Ip(Utils::load_dat<Vector>(config.getString("Ip")));
		Matrix lambda0(Utils::load_dat<Matrix>(config.getString("lambda0")));
		Matrix flux(Utils::load_dat<Matrix>(config.getString("flux")));

		const auto grid_iter_list = {F.data(), Ip.data()};
		const auto grid_sizes = {F.size(), Ip.size()};
		const size_t num_elements = F.size() * Ip.size();

		lambda0.transposeInPlace();
		flux.transposeInPlace();

		m_lambda0.reset(new InterpMultilinear<2, double>(
		    grid_iter_list.begin(), grid_sizes.begin(), lambda0.data(),
		    lambda0.data() + num_elements));
		m_flux.reset(new InterpMultilinear<2, double>(
		    grid_iter_list.begin(), grid_sizes.begin(), flux.data(),
		    flux.data() + num_elements));
	}

	virtual Vector step(const double, const Vector &r, const Vector &) {
		if (r.rows() == 0 || r.cols() == 0) {
			return Vector();
		}
		Vector r1(2);
		r1(0) = lambda0(r(0), r(1));
		r1(1) = flux(r(0), r(1));
		return r1;
	}

	double lambda0(const double F, const double Ip) const {
		const std::array<double, 2> args = {{F, Ip}};
		return m_lambda0->interp(args.begin());
	}

	double flux(const double F, const double Ip) const {
		const std::array<double, 2> args = {{F, Ip}};
		return m_flux->interp(args.begin());
	}

private:
	std::unique_ptr<InterpMultilinear<2, double>> m_lambda0;
	std::unique_ptr<InterpMultilinear<2, double>> m_flux;
};

class ReferenceOutput {
public:
	ReferenceOutput(const Config &config) {
		const double F1 = config.getDouble("F1");
		const double F2 = config.getDouble("F2");
		const double Ip1 = config.getDouble("Ip1");
		const double Ip2 = config.getDouble("Ip2");

		m_t1 = config.getDouble("t1");
		m_t2 = config.getDouble("t2");

		m_r1 = Vector2d(F1, Ip1);
		m_r2 = Vector2d(F2, Ip2);
	}

	Vector operator()(const double t) const {
		if (t < m_t1) {
			return Vector(); // FIXME openloop
		} else if (t < m_t2) {
			return m_r1;
		} else {
			return m_r2;
		}
	}

private:
	double m_t1;
	double m_t2;
	Vector2d m_r1;
	Vector2d m_r2;
};

} // namespace MST

#endif