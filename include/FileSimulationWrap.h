/** @file FileSimulation.h Contains a class interface for file-driven 
    simulation.
*/
#include "SimulationInterface.h"
#include "SimulationDecoratorBase.h"
#include "Units.h"
#include "BicycleInterface.h"
#include <fstream>
#include <stdexcept>
#include <string>
#include <iosfwd>
#include <sstream>

class FileSimulationWrap : public SimulationDecoratorBase {
	std::fstream fs;
	std::stringstream str;
	std::string s;
	Time simTime;
	Time lastTime;

	/** @brief Parses a line from file.
	 *  The line pattern looks like this string
	 *  `<time_ms> <x> <y> <z> <float> <float> <float> <velocity_m_s> <float> <float> <angle_degrees> <float> <float>`
	 *  there:
	 *   + time_ms looks like unsigned integer time in ms;
	 *   + x, y, z is the float coords of tractor, in the ECEF system (which convertible to NED system);
	 *   + float is ignored float numbers;
	 *   + velocity_m_s is the tractor velocity;
	 *   + angle_degrees is the tractor's azimuth;
	 */
	void parse(const std::string &s)
	{
		float tmp;
		UnitPointT<float> pt = { 0 };
		str << s;
		// now extract exact values from the stream.
		// time (ms), float x, y, z, float, float, float, velocity (m/s), float, float, angle, float, float
		str >> tmp;
		lastTime = simTime;
		simTime = tmp;
		str >> pt.x >> pt.y >> pt.z; // todo: operator >>
		// skip 3 unused fields
		str >> tmp;
		str >> tmp;
		str >> tmp;
		// speed
		str >> tmp;
		bicycle->setSpeed(tmp);
		// unused
		str >> tmp;
		str >> tmp;
		str >> tmp;
		bicycle->setFrontWheelRotation(tmp);
		// ignore last 2 floats.
		bicycle->advance(0.0); // force update angles and coords.
	}
public:

	FileSimulationWrap(std::shared_ptr<SimulationInterface> &simulation, const char *filename)
		: SimulationDecoratorBase(simulation)
	{
		s.reserve(50);
		fs.open(filename);

		if (!fs.is_open()) {
			throw std::runtime_error("File I/O error.");
		}
	}

	virtual void advance(const Time &step) override
	{
		if (fs) {
			getline(fs, s);
			parse(s);
		}
		decorated->advance(step);
	}
};
