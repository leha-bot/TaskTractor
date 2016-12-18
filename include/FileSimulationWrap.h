/** @file FileSimulation.h Contains a class interface for file-driven 
 *   simulation.
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
#include <chrono>

/** @brief Convert units using the next formula: to = from / Ratio::num * Ratio::den;
 *         (Ratio::num is the numerator, Ratio::den is the denominator).
 *  @tparam Ratio Is the fractional type which has numerator and denominator.
 *  @note This template function won't work if Ratio::num and Ratio::den is equal 0 or
          T1 and T2 is same and Ratio::num == Ratio::den
 */
template < typename T1, typename T2, typename Ratio,
	typename = std::enable_if < (!std::is_same<T1, T2>::value
	|| Ratio::num != Ratio::den)
	&& Ratio::num != 0 && Ratio::den != 0> ::type >
void convert_units(const T1 &from, T2 &to)
{
	to = from / (T1) Ratio::num * Ratio::den;
}

/* @brief template specialization for convert_units which converts unsigned long ms to Time.
template<>
void convert_units<unsigned long, Time, >(const unsigned long &from, Time &to)
{
	to = from / 1000.0;
}
*/

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
		Time time_tmp;
		unsigned long ms;
		UnitPointT<float> pt = { 0 };
		str << s;
		// now extract exact values from the stream.
		// time (ms), float x, y, z, float, float, float, velocity (m/s), float, float, angle, float, float
		
		
		str >> ms;

		// convert msecs to secs.
		convert_units<unsigned long, Time, std::kilo>(ms, time_tmp);

		lastTime = simTime;
		simTime = time_tmp;
		str >> pt.x >> pt.y >> pt.z; // todo: operator >>

		bicycle->setCoords(pt);
		
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
		str >> tmp;
		str >> tmp;
		bicycle->advance(0.0); // force update angles and coords.
	}
public:

	FileSimulationWrap(std::shared_ptr<SimulationInterface> &simulation, const char *filename)
		: SimulationDecoratorBase(simulation), simTime(0), lastTime(0)
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

	virtual void advance() override
	{
		Time delta = simTime - lastTime;
		advance(delta < 0 ? 0.0 : delta);
	}
};
