/** @file SimulationVoidDecorator.h Contains a generic class-router for 
 *        implementing custom Decorators.
 */
#pragma once
#include "SimulationInterface.h"

class SimulationDecoratorBase : public SimulationInterface {
protected:
	SimPtr decorated;
	BicycleInterface *bicycle;
public:
	SimulationDecoratorBase(SimPtr &sim)
		: decorated(sim)
	{
		bicycle = this->getSimulatedFromInstance(sim);
	}

	virtual void advance(const Time &step) override
	{
		decorated->advance(step);
	}

	virtual void advance() override
	{
		decorated->advance();
	}

	virtual void turnLeft() override
	{
		decorated->turnLeft();
	}

	virtual void turnRight() override
	{
		decorated->turnRight();
	}

	virtual void accelerate() override
	{
		decorated->accelerate();
	}

	virtual void deccelerate() override
	{
		decorated->deccelerate();
	}

	virtual const Speed & getSpeedDelta() override
	{
		return decorated->getSpeedDelta();
	}

	virtual const AngleDegrees & getAngleDelta() override
	{
		return decorated->getAngleDelta();
	}

	virtual const Time & getTimeDelta() override
	{
		return decorated->getTimeDelta();
	}

	virtual const Time & getSimulationTime() override
	{
		return decorated->getSimulationTime();
	}

	virtual SimPtr getDecorated()
	{
		return decorated;
	}

protected:
	virtual BicycleInterface *getSimulated() override
	{
		return this->bicycle;
	}
};