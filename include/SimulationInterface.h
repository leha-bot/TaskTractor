/** @file SimulationInterface.h Contains a simulation controller interface
 definition.
*/
#pragma once
#include "Units.h"
#include "BicycleInterface.h"
#include <memory>
#include <list>

// forward declaration for typedef.
class SimulationInterface;
class SimulationObserver;

typedef std::shared_ptr<SimulationObserver> SimObserverPtr;
typedef std::shared_ptr<SimulationInterface> SimPtr;
typedef std::shared_ptr<BicycleInterface> BicyclePtr;

class SimulationInterface {
public:
	typedef BicycleInterface::TimeUnit Time;
	typedef BicycleInterface::CoordUnit Speed;
	typedef BicycleInterface::CoordUnit Length;
	typedef BicycleInterface::AngleDegrees AngleDegrees;

	virtual void advance(const Time &step) = 0;
	virtual void advance() = 0;
	virtual void turnLeft() = 0;
	virtual void turnRight() = 0;
	virtual void accelerate() = 0;
	virtual void deccelerate() = 0;
	virtual ~SimulationInterface() {}

	// virtual getters
	virtual const Speed &getSpeedDelta() = 0;
	virtual const AngleDegrees &getAngleDelta() = 0;
	virtual const Time &getTimeDelta() = 0;
	virtual const Time &getSimulationTime() = 0;

protected:
	BicycleInterface *getSimulatedFromInstance(SimPtr &iface)
	{
		return iface->getSimulated();
	}

	virtual BicycleInterface *getSimulated() = 0;
};

class SimulationObserver {
public:
	typedef SimulationInterface::Time Time;
	virtual void notify(const BicyclePtr &sim,
		const char *tag, const Time &simTime, const Time &delta) = 0;
};

/** @brief A simple single-threaded Observer s group.
 */
class SimulationObserverGroup : public SimulationObserver {
	std::list<SimObserverPtr> observers;

public:
	virtual void notify(const BicyclePtr &sim,
		const char *tag, const Time &simTime,
		const Time &delta) override
	{
		for (auto iter : observers) {
			iter->notify(sim, tag, simTime, delta);
		}
	}

	void add(SimObserverPtr &observer)
	{
		observers.push_back(observer);
	}

	void remove(SimObserverPtr &observer)
	{
		observers.remove(observer);
	}
};

/** @brief The base class for different UI implementations.
 *
 */
class SimulationUi : public SimulationObserver,
	public std::enable_shared_from_this<SimulationUi> {

public:
	virtual void notify(const BicyclePtr &sim, const char *tag,
		const Time &simTime, const Time &delta) = 0;

	virtual void loop() = 0;
};

/** @brief A lightweight Builder interface for minimizing dependencies.
 *         Extracted from class SimulationBuilder.
 */
class SimulationLightBuilder {
public:
	virtual void addObserver(SimObserverPtr &ptr) = 0;
	virtual SimPtr build() = 0;

};