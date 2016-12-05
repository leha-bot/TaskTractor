/** @file main.cpp Contains entry point for Traktor Simulation program using 
 *  the Kinematic Bicycle Model.
 *  
 */
#include <string>
#include <type_traits>
#include <limits>
#include <math.h>
#include <memory>
#include <iostream>
#include <list>

/** 

struct BigInt {
	int a;
	int b;
};

template <>
struct std::is_arithmetic<BigInt> {
	static const bool value = true;
};
*/

template <typename CoordUnitT>
struct UnitPoint {
	CoordUnitT x;
	CoordUnitT y;
};

/*
template <typename T>
using enable_if_arithmetic = typename std::enable_if< std::is_arithmetic<T>::value, T >::type;
*/

template <typename Num >
Num rads_to_degs(const Num& n)
{
	static const Num PI = 3.1415926535897932384626433832795;
	return n / PI * 180.0f;
}

template <typename Num >
Num degs_to_rads(const Num& n)
{
	static const Num PI = 3.1415926535897932384626433832795;
	return n / 180.0f * PI;
}

/** @brief A template abstract class for all our simulated Bicycles.
 *
 */
template <typename CoordUnitT, 
	typename TimeUnitT,
	typename AngleDegreesT>
class BicycleT {
public:
	typedef UnitPoint<CoordUnitT> UnitPointT;
	typedef CoordUnitT CoordUnit;
	typedef TimeUnitT TimeUnit;
	typedef AngleDegreesT AngleDegrees;

	virtual void advance(const TimeUnitT &units) = 0;
	virtual void setFrontWheelRotation(const AngleDegreesT &units) = 0;
	virtual void setSpeed(const CoordUnitT &units) = 0;
	virtual void modifySpeed(const CoordUnitT &delta) = 0;
	virtual void modifyFrontWheelRotation(const AngleDegreesT &delta) = 0;
	virtual const CoordUnitT getSpeed() = 0;
	virtual const AngleDegreesT getFrontWheelRotation() = 0;
//	template<typename U = Coords>
	virtual void getFrontWheelCoords(UnitPointT &pt) = 0;
//	template<typename U = Coords>
	virtual void getRearWheelCoords(UnitPointT &pt) = 0;
};

class BicycleInterface: public BicycleT<float, float, float> {

};

class SimpleBicycle : public BicycleInterface {
	
	float x, y, speed;
	float angle_rads;
	float angle_degs;
	
	// optimizing vars.
	float cos_angle;
	float sin_angle;
	float x_sp_cos_angle;
	float y_sp_sin_angle;

	void calc_speed_optimize_vars()
	{
		x_sp_cos_angle = cos_angle * speed;
		y_sp_sin_angle = sin_angle * speed;
	}

	template<typename T>
	void fix_eps(T &t)
	{
		if (fabs(t) < std::numeric_limits<T>::epsilon()) {
			t = 0.0;
		}
	}

	void calc_angle_optimize_vars()
	{
		cos_angle = cos(angle_rads);
		sin_angle = sin(angle_rads);
		fix_eps(cos_angle);
		fix_eps(sin_angle);
		calc_speed_optimize_vars();
	}

public:
	SimpleBicycle()
	{
		x = 0;
		y = 0;
		angle_rads = 0;
		angle_degs = 0;
		speed = 0;
		sin_angle = 0;
		cos_angle = 1;
		x_sp_cos_angle = 0;
		y_sp_sin_angle = 0;
	}

	virtual void advance(const TimeUnit &units) override
	{
		x += units * x_sp_cos_angle;
		y += units * y_sp_sin_angle;
		// x += speed * units * cos(angle_rads);
		// y += speed * units * sin(angle_rads);
	}

	virtual void setFrontWheelRotation(const AngleDegrees &rotation) override
	{
		angle_degs = rotation;
		angle_rads = degs_to_rads(rotation);
		calc_angle_optimize_vars();
	}

	virtual void setSpeed(const float &units) override
	{
		speed = units;
		calc_speed_optimize_vars();
	}

	virtual const float getSpeed() override
	{
		return speed;
	}

	virtual const float getFrontWheelRotation() override
	{
		return rads_to_degs(angle_rads);
	}

	virtual void getFrontWheelCoords(UnitPointT &pt) override
	{
		pt.x = x;
		pt.y = y;
	}

	/*
	@note Assume that this bicycle is actually unicycle.
	*/
	virtual void getRearWheelCoords(UnitPointT &pt) override
	{
		getFrontWheelCoords(pt);
	}

	virtual void modifySpeed(const CoordUnit &delta) override
	{
		setSpeed(speed + delta);
	}


	virtual void modifyFrontWheelRotation(const AngleDegrees &delta) override
	{
		setFrontWheelRotation(angle_degs + delta);
	}

};

class SimulationObserver {
public:
	typedef BicycleInterface::TimeUnit Time;
	virtual void notify(const std::shared_ptr<BicycleInterface> &sim, const char *tag,
		const Time &simTime, const Time &delta) = 0;
};

class InfoLog : public SimulationObserver {
public:
	void log();

	virtual void notify(const std::shared_ptr<BicycleInterface> &sim, const char *tag,
		const Time &simTime, const Time &delta) override
	{
		BicycleInterface::UnitPointT pt;
		sim->getFrontWheelCoords(pt);
		std::cout << "SimLog: Tag " << tag;
		std::cout << "; Time: " << simTime << "(" << delta << ")";
		std::cout << "Bicycle pos: x: " << pt.x << ", y: " << pt.y
			<< "; angle: " << sim->getFrontWheelRotation() << std::endl;
	}

};

class SimulationObserverGroup : public SimulationObserver {
	std::list<std::shared_ptr<SimulationObserver> > observers;

public:
	virtual void notify(const std::shared_ptr<BicycleInterface> &sim,
		const char *tag, const Time &simTime,
		const Time &delta) override
	{
		for (auto iter : observers) {
			iter->notify(sim, tag, simTime, delta);
		}
	}
	
	void add(std::shared_ptr<SimulationObserver> &observer)
	{
		observers.push_back(observer);
	}

	void remove(const std::shared_ptr<SimulationObserver> &observer)
	{
		observers.remove(observer);
	}
};

class Simulation {
	SimulationObserverGroup observers;
	std::shared_ptr<BicycleInterface> simulated;
	typedef std::shared_ptr<SimulationObserver> ObserverPtr;
public:
	typedef BicycleInterface::TimeUnit Time;
//	typedef BicycleInterface::setFrontWheelRotation()

private:
	Time stepTime;
	Time stepAngle;
	Time stepSpeed;
	Time simulationTime;
public:

	Simulation(const Time &simStep, const float &angleStep, const Time &speedStep)
	{
		ObserverPtr simLogPtr = std::make_shared<InfoLog>();

		observers.add(simLogPtr);
		simulated = std::make_shared<SimpleBicycle>();
		stepTime = simStep;
		stepSpeed = speedStep;
		stepAngle = angleStep;
		simulationTime = 0.0;
	}

	void advance()
	{
		advance(stepTime);
	}

	void advance(const Time &step)
	{
		simulated->advance(step);
		simulationTime += step;
		observers.notify(simulated, "SimpleBicycle", simulationTime, step);
	}

	void turnLeft()
	{
		simulated->modifyFrontWheelRotation(stepAngle);
	}

	void turnRight()
	{
		simulated->modifyFrontWheelRotation(-stepAngle);
	}

	void accelerate()
	{
		simulated->modifySpeed(stepSpeed);
	}

	void deccelerate()
	{
		simulated->modifySpeed(-stepSpeed);
	}
};

//template <typename Unit, typename /*= std::enable_if< std::is_arithmetic<Unit>::value >::type */>
//Bicycle<Unit>::Bicycle(const Unit &length)
//{
//	this->length = length;
//}

int main(int argc, char *argv[])
{
	/*static_assert(std::is_fundamental<BigInt>::value, "fuck");
	static_assert(std::is_compound<BigInt>::value, "fuck");
	static_assert(std::is_class<BigInt>::value, "fuck");
*/
	// test
	// int length = 10;
	SimpleBicycle byc;
	Simulation sim(10.0f, 2.0f, 0.1f);
	bool flag = true;
	// int a = 0;
	char c;

	while (flag) {
		sim.advance();
		std::cin >> c;
		switch (c) {
		case 'w':
			sim.accelerate();
			break;
		case 's':
			sim.deccelerate();
			break;
		case 'a':
			sim.turnLeft();
			break;
		case 'd':
			sim.turnRight();
			break;
		case 'e':
			break;
		default:
			flag = false;
		}
	}

	return 0;
}