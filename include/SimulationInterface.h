/** @file SimulationInterface.h Contains a simulation controller interface
 definition.
*/
#pragma once
#include "Units.h"
#include "BicycleInterface.h"
#include <memory>
#include <list>

class SimulationInterface {
public:
	virtual void advance(const Time &step) = 0;
	virtual void turnLeft() = 0;
	virtual void turnRight() = 0;
	virtual void accelerate() = 0;
	virtual void deccelerate() = 0;
	virtual ~SimulationInterface() {}
};

class SimulationObserver {
public:
	typedef BicycleInterface::TimeUnit Time;
	virtual void notify(const std::shared_ptr<BicycleInterface> &sim,
		const char *tag, const Time &simTime, const Time &delta) = 0;
};

typedef std::shared_ptr<SimulationObserver> ObserverPtr;

class SimulationObserverGroup : public SimulationObserver {
	std::list<ObserverPtr > observers;

public:
	virtual void notify(const std::shared_ptr<BicycleInterface> &sim,
		const char *tag, const Time &simTime,
		const Time &delta) override
	{
		for (auto iter : observers) {
			iter->notify(sim, tag, simTime, delta);
		}
	}

	void add(ObserverPtr &observer)
	{
		observers.push_back(observer);
	}

	void remove(ObserverPtr &observer)
	{
		observers.remove(observer);
	}
};
