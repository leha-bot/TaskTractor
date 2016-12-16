#include "SimulationInterface.h"
#include "SimulationDecoratorBase.h"
#pragma once

/** @brief An decorator class which add speed change and angle change handlers.
 */
class SimulationWithHandlers : public SimulationDecoratorBase {
public:
	/** @brief Handles all units' changes. The class SimulationWithHandlers 
	 *  calls method handle() on calling certain methods.
	 */
	template <typename UnitT>
	class Handler {
	public:
		typedef UnitT Unit;
		/** @brief Handles parameters changing.
		 *  @param[in] oldValue Old value.
		 *  @param[in] delta Parameter change delta.
		 *  @param[inout] newValue New value. On function call newValue == oldValue + delta.
		 *                Function may change this value, but it must returns true on this.
		 *  @param[out] notify If true, then SimulationWithHandlers 
		 *              will call SimulationInterface::advance(0.0)
		 *  @returns true if Unit change has been handled (then SimulationWithHandlers will call set-method for
		 *           a changed value (but if only changed newValue != oldValue);
		 *           false if Unit change hasn't be handled (then SimulationWithHandlers will call 
		 *	     the underlying SimulationInterface simulation method).
		 */
		virtual bool handle(const Unit &oldValue, const Unit &delta, Unit &newValue, bool &notify) = 0;
	};

	typedef Handler<Speed> SpeedHandler;
	typedef Handler<AngleDegrees> AngleHandler;
	typedef std::shared_ptr<SpeedHandler> SpeedHandlerPtr;
	typedef std::shared_ptr<AngleHandler> AngleHandlerPtr;

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

	SpeedHandlerPtr speedChangeHandler;
	AngleHandlerPtr angleChangeHandler;

public:
	SimulationWithHandlers(SimPtr &iface)
		: SimulationDecoratorBase(iface)
	{
		speedChangeHandler = std::make_shared<DefaultHandler<Speed>>();
		angleChangeHandler = std::make_shared<DefaultHandler<AngleDegrees>>();
		assert(this->bicycle != nullptr);
	}

	// Handlers

	virtual void setSpeedChangeHandler(SpeedHandlerPtr &h)
	{
		speedChangeHandler = h;
	}

	virtual void setAngleChangeHandler(AngleHandlerPtr &h)
	{
		angleChangeHandler = h;
	}

	// Active logic.

	virtual void turnLeft() override
	{
		auto oldVal = bicycle->getFrontWheelRotation();
		auto delta = decorated->getAngleDelta();
		decltype (oldVal) newVal = oldVal + delta;
		bool notify = false;
		bool handled = angleChangeHandler->handle(oldVal, delta, newVal, notify);
		if (handled) {
			if (oldVal != newVal) {
				bicycle->setFrontWheelRotation(newVal);
			}
			if (notify) {
				this->advance(0.0);
			}
		} else {
			decorated->turnLeft();
		}
	}

	virtual void turnRight() override
	{
		auto oldVal = bicycle->getFrontWheelRotation();
		auto delta = decorated->getAngleDelta();
		decltype (oldVal) newVal = oldVal - delta;
		bool notify = false;
		bool handled = angleChangeHandler->handle(oldVal, delta, newVal, notify);
		if (handled) {
			if (oldVal != newVal) {
				bicycle->setFrontWheelRotation(newVal);
			}
			if (notify) {
				this->advance(0.0);
			}
		} else {
			decorated->turnRight();
		}
	}

	virtual void accelerate() override
	{
		auto oldVal = bicycle->getSpeed();
		auto delta = decorated->getSpeedDelta();
		decltype (oldVal) newVal = oldVal + delta;
		bool notify = false;
		bool handled = speedChangeHandler->handle(oldVal, delta, newVal, notify);
		if (handled) {
			if (oldVal != newVal) {
				bicycle->setSpeed(newVal);
			}
			if (notify) {
				this->advance(0.0);
			}
		} else {
			decorated->accelerate();
		}
	}

	virtual void deccelerate() override
	{
		auto oldVal = bicycle->getSpeed();
		auto delta = decorated->getSpeedDelta();
		decltype (oldVal) newVal = oldVal - delta;
		bool notify = false;
		bool handled = speedChangeHandler->handle(oldVal, delta, newVal, notify);
		if (handled) {
			if (oldVal != newVal) {
				bicycle->setSpeed(newVal);
			}
			if (notify) {
				this->advance(0.0);
			}
		} else {
			decorated->deccelerate();
		}
	}
};

