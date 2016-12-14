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
	BicycleInterface *getSimulatedFromInstance(std::shared_ptr<SimulationInterface> iface)
	{
		return iface->getSimulated();
	}

	virtual BicycleInterface *getSimulated() = 0;
};

class SimulationObserver {
public:
	typedef BicycleInterface::TimeUnit Time;
	virtual void notify(const std::shared_ptr<BicycleInterface> &sim,
		const char *tag, const Time &simTime, const Time &delta) = 0;
};

typedef std::shared_ptr<SimulationObserver> ObserverPtr;

class SimulationObserverGroup : public SimulationObserver {
	std::list<ObserverPtr> observers;

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

/** @brief An decorator class which add speed change and angle change handlers.
 */
class SimulationWithHandlers : public SimulationInterface {
public:
	/** @brief Handles all units' changes. The Simulation calls method
	*   handle() on calling certain methods.
	*/
	template <typename UnitT>
	class Handler {
	public:
		typedef UnitT Unit;
		/** @brief Handles parameters changing.
		 *  @param oldValue Old value.
		 *  @param delta Parameter change delta.
		 *  @param newValue New value. On function call newValue == oldValue + delta.
		 *         Function may change this value, but it must returns true on this.
		 *  @param notify If true, then SimulationWithHandlers call SimulationInterface::advance(0.0)
		*/
		virtual bool handle(const Unit &oldValue, const Unit &delta, Unit &newValue, bool &notify) = 0;
	};
protected:
	template <typename T>
	class DefaultHandler : public Handler<T> {
	public:
		virtual bool handle(const T &oldValue, const T &delta, T &newValue, bool &notify)
		{
			notify = false;
			return false;
		}
	};
	std::shared_ptr<SimulationInterface> iface;
	std::shared_ptr<Handler<BicycleInterface::CoordUnit>> speedChangeHandler;
	std::shared_ptr<Handler<BicycleInterface::AngleDegrees>> angleChangeHandler;


	BicycleInterface *biciface;

public:
	typedef Handler<Speed> SpeedHandler;
	typedef Handler<AngleDegrees> AngleHandler;
	virtual BicycleInterface *getSimulated() override
	{
		return this->biciface;
	}


	virtual const Speed & getSpeedDelta() override
	{
		throw std::logic_error("The method or operation is not implemented.");
	}


	virtual const AngleDegrees & getAngleDelta() override
	{
		throw std::logic_error("The method or operation is not implemented.");
	}


	virtual const Time & getTimeDelta() override
	{
		throw std::logic_error("The method or operation is not implemented.");
	}


	virtual const Time & getSimulationTime() override
	{
		throw std::logic_error("The method or operation is not implemented.");
	}

	/*addSpeedConstraint, addAngle, removeConstraint, changeConstraint*/
	virtual void setSpeedChangeHandler(std::shared_ptr<SpeedHandler> &h)
	{
		speedChangeHandler = h;
	}

	virtual void setAngleChangeHandler(std::shared_ptr<AngleHandler> &h)
	{
		angleChangeHandler = h;
	}

	// routed methods

	virtual void advance(const Time &step) override
	{
		iface->advance(step);
	}

	virtual void advance() override
	{
		iface->advance();
	}


	virtual void turnLeft() override
	{
		auto oldVal = biciface->getFrontWheelRotation();
		auto delta = iface->getAngleDelta();
		decltype (oldVal) newVal = oldVal + delta;
		bool notify = false;
		bool handled = angleChangeHandler->handle(oldVal, delta, newVal, notify);
		if (handled) {
			if (oldVal != newVal) {
				biciface->setFrontWheelRotation(newVal);
			}
			if (notify) {
				this->advance(0.0);
			}
		} else {
			iface->turnLeft();
		}
	}

	virtual void turnRight() override
	{
		auto oldVal = biciface->getFrontWheelRotation();
		auto delta = iface->getAngleDelta();
		decltype (oldVal) newVal = oldVal - delta;
		bool notify = false;
		bool handled = angleChangeHandler->handle(oldVal, delta, newVal, notify);
		if (handled) {
			if (oldVal != newVal) {
				biciface->setFrontWheelRotation(newVal);
			}
			if (notify) {
				this->advance(0.0);
			}
		} else {
			iface->turnRight();
		}
	}


	virtual void accelerate() override
	{
		auto oldVal = biciface->getSpeed();
		auto delta = iface->getSpeedDelta();
		decltype (oldVal) newVal = oldVal + delta;
		bool notify = false;
		bool handled = speedChangeHandler->handle(oldVal, delta, newVal, notify);
		if (handled) {
			if (oldVal != newVal) {
				biciface->setFrontWheelRotation(newVal);
			}
			if (notify) {
				this->advance(0.0);
			}
		} else {
			iface->accelerate();
		}
	}


	virtual void deccelerate() override
	{
		auto oldVal = biciface->getSpeed();
		auto delta = iface->getSpeedDelta();
		decltype (oldVal) newVal = oldVal - delta;
		bool notify = false;
		bool handled = speedChangeHandler->handle(oldVal, delta, newVal, notify);
		if (handled) {
			if (oldVal != newVal) {
				biciface->setFrontWheelRotation(newVal);
			}
			if (notify) {
				this->advance(0.0);
			}
		} else {
			iface->deccelerate();
		}
	}

	SimulationWithHandlers(std::shared_ptr<SimulationInterface> iface)
		:iface(iface)
	{
		this->biciface = getSimulatedFromInstance(iface);
		speedChangeHandler = std::make_shared<DefaultHandler<Speed>>();
		angleChangeHandler = std::make_shared<DefaultHandler<AngleDegrees>>();
		assert(this->biciface != nullptr);
	}
};