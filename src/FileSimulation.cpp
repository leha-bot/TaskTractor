/** @file FileSimulation.cpp Contains implementation of File-driven simulation.
*/
#include "FileSimulation.h"

FileSimulation::FileSimulation(const Time &simStep, const AngleDegrees &angleStep,
	const Speed &speedStep, const Length length, const char *filename, 
	std::shared_ptr<BicycleInterface> simulated, const SimulationObserverGroup &observers)
	: simulated(simulated), observers(observers)
{
	s.reserve(40);

	fs.open(filename);

	if (!fs.is_open()) {
		throw std::runtime_error("File I/O error.");
	}
}
