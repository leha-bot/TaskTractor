/** @file FileSimulation.h Contains a class interface for file-driven 
    simulation.
*/
#include "SimulationInterface.h"
#include "Units.h"
#include "BicycleInterface.h"
#include <fstream>
#include <stdexcept>
#include <string>
#include <iosfwd>
#include <sstream>

class FileSimulation : public SimulationInterface {
	std::fstream fs;
	std::string s;
	std::stringstream str;
	Time simTime;
	Time lastTime;
	std::shared_ptr<BicycleInterface> simulated;
	SimulationObserverGroup observers;

public:
	// typedef BicycleInterface::TimeUnit Time;
	typedef BicycleInterface::CoordUnit Speed;
	typedef BicycleInterface::CoordUnit Length;
	typedef BicycleInterface::AngleDegrees AngleDegrees;

	FileSimulation(const Time &simStep, const AngleDegrees &angleStep,
		const Speed &speedStep, const Length length,
		const char *filename, std::shared_ptr<BicycleInterface> simulated,
		const SimulationObserverGroup &observers);

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
		simulated->setSpeed(tmp);
		// unused
		str >> tmp;
		str >> tmp;
		str >> tmp;
		simulated->setFrontWheelRotation(tmp);
		// ignore last 2 floats.
		simulated->advance(0.0); // force update angles and coords.
	}

	virtual void advance(const Time &step) override
	{
		if (fs) {
			getline(fs, s);

		}
	}


	virtual void turnLeft() override
	{
		throw std::logic_error("The method or operation is not implemented.");
	}


	virtual void turnRight() override
	{
		throw std::logic_error("The method or operation is not implemented.");
	}


	virtual void accelerate() override
	{
		throw std::logic_error("The method or operation is not implemented.");
	}


	virtual void deccelerate() override
	{
		throw std::logic_error("The method or operation is not implemented.");
	}

};
