/** @file main.cpp Contains entry point for Tractor Simulation program using 
 *  the Kinematic Bicycle Model.
 *  
 * @mainpage Brief contents:
 *
 * SIMULATION MODEL
 * ================
 * The bicycle simulation logic is located in class SimpleBicycle (simplified 
 * model of bicycle without length), LengthBicycle (with length and vehicle 
 * angle which differs from front wheel angle (they're named 'delta' and 'Phi'
 * in article "Automotive...". These classes are derived from template class 
 * BicycleT<typename CoordUnitT, typename TimeUnitT, typename AngleDegreesT>, 
 * located in file BicycleInterface.h. We use template parameters for the 
 * ability to switch to another units with another semantics.
 * SIMULATION CONTROLLER
 * =====================
 * This is a class Simulation, which encapsulates the simulated bicycle model and
 * controls its behavior.
 *
 * Creating an instance of Simulation Controller
 * ---------------------------------------------
 * The Simulation instance is built via SimulationBuilder
 * interface which have chained setter methods and abstract method 
 * build(ObserverPtr uiView). The ObserverPtr is the 
 * std::shared_ptr<SimulationObserver> which is notified by the controller if
 * the simulation has been advanced. These ObserverPtr's are stored in the 
 * class ObserverGroup which is derived from Observer and it notifies all its 
 * children (a typical Composite Design Pattern implementation).
 *
 * Simulation controller logic
 * ---------------------------
 * The simulator controller has the Bicycle controlling methods:
 * + accelerate, deccelerate - for *unconstrained* change the Bicycle speed;
 * + turnLeft, turnRight - for *unconstrained* change the Bicycle front wheel
 *   angle;
 * + advance - for simulate the Bicycle movement with its parameters (speed, 
 *   angle, etc). In fact, the controller calls the advance() method of Bicycle.
 *
 * Simulation Controller Types
 * ---------------------------
 * There are two controller types:
 * + The 'active' simulation (class Simulation in this source) which accepts 
 *   the control messages from View and changes the Bicycle parameters;
 * + The 'file' simulation (class FileSimulation in FileSimulation.h/cpp, 
 *   INCOMPLETE) - which reads lines from file on each advance() call and 
 *   overrides the simulated model's parameters.
 * 
 * SIMULATION LOG
 * ==============
 * Simulation Log is the class InfoLog derived from SimulationObserver which
 * has been notified while the controller advances the simulation.
 *
 * SIMULATION UI
 * =============
 * Simulation UI is derived from Observer and std::enable_shared_from_this
 * for the ability to create the std::shared_ptr and store in Simulation's 
 * ObserverGroup. The method loop() contains an input events loop.
 *
 * ConsoleSimulationUi controls
 * ------------------
 * + w, s - increase/decrease speed;
 * + a, d - turn left/right;
 * + e - advance simulation;
 * + q - quit.
 */
#include "SimulationInterface.h"
#include "Units.h"
#include <string>
#include <type_traits>
#include <limits>
#include <math.h>
#include <memory>
#include <iostream>
#include <list>
#include "BicycleInterface.h"
#include <condition_variable>
#include <assert.h>

template<typename T>
void fix_eps(T &t)
{
	if (fabs(t) < std::numeric_limits<T>::epsilon()) {
		t = 0.0;
	}
}

template<typename T>
void clamp_degrees(T &degs)
{
	if (degs >= 360.0) {
		degs -= 360.0;
	} else if (degs <= 0.0) {
		degs += 360.0;
	}
}

class SimpleBicycle : public BicycleInterface {
protected:
	float x, y;
private:
	float speed;
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
		return angle_degs;
	}

	virtual void getRearWheelCoords(UnitPoint &pt) override
	{
		pt.x = x;
		pt.y = y;
	}

	/**
	@note Assume that this bicycle is actually unicycle.
	*/
	virtual void getFrontWheelCoords(UnitPoint &pt) override
	{
		getRearWheelCoords(pt);
	}

	virtual void modifySpeed(const CoordUnit &delta) override
	{
		setSpeed(speed + delta);
	}


	virtual void modifyFrontWheelRotation(const AngleDegrees &delta) override
	{
		setFrontWheelRotation(angle_degs + delta);
	}

	virtual const float getLength() override
	{
		return 0.0f;
	}

	virtual const float getVehicleRotation() override
	{
		return getFrontWheelRotation();
	}

protected:
	const float getFrontWheelRotationRads()
	{
		return angle_rads;
	}
};

class LengthBicycle : public SimpleBicycle {
	CoordUnit length;
	float vehicle_angle_rads;
public:
	LengthBicycle(const CoordUnit& len) 
		: length(len)
	{
		vehicle_angle_rads = 0;
	}

	virtual void getFrontWheelCoords(UnitPoint &pt) override
	{
		getRearWheelCoords(pt);

		pt.x += this->length * cos(this->vehicle_angle_rads);
		pt.y += this->length * sin(this->vehicle_angle_rads);	
	}


	virtual void advance(const TimeUnit &units) override
	{
		float speed = getSpeed();
		float angle_fr = getFrontWheelRotationRads();
		float angle_deg = getFrontWheelRotation();
		float ds = speed * units;

		x += ds * cos(vehicle_angle_rads);
		y += ds * sin(vehicle_angle_rads);
		
		vehicle_angle_rads += tan(angle_fr) / length * ds;
	}


	virtual const CoordUnit getLength() override
	{
		return length;
	}

	virtual const AngleDegrees getVehicleRotation() override
	{
		return rads_to_degs(vehicle_angle_rads);
	}

};

class InfoLog : public SimulationObserver {
public:
	void log();

	virtual void notify(const std::shared_ptr<BicycleInterface> &sim, const char *tag,
		const Time &simTime, const Time &delta) override
	{
		BicycleInterface::UnitPoint pt;
		sim->getRearWheelCoords(pt);
		std::cout << "" << tag;
		std::cout << ": time: " << simTime << "(" << delta << ")";
		std::cout << ";pos rear: x: " << pt.x << ", y: " << pt.y;
		std::cout << "; speed: " << sim->getSpeed();
		sim->getFrontWheelCoords(pt);
		std::cout << "; pos frnt: x: " << pt.x << ", y: " << pt.y;
		std::cout << "; front angle: " << sim->getFrontWheelRotation()
			<< "; vehicle angle: " << sim->getVehicleRotation()
			<< "; Len: " << sim->getLength()
			<< ";" << std::endl;
	}
};

/* @brief Constructs instances of SimulationInterface (defined by derived classes)
  and BicycleInterface (defined by private member SimulationBuilder::type's value.
*/
class SimulationBuilder {
public:
	typedef BicycleInterface::CoordUnit CoordUnit;
//	typedef BicycleInterface::CoordUnit CoordUnit;
	typedef BicycleInterface::AngleDegrees AngleDegrees;
	enum BicycleType {
		Type_Simple,
		Type_Length
	};

protected:
	SimulationObserverGroup observers;

	BicycleInterface::CoordUnit length;
	BicycleInterface::CoordUnit speedStep;
	BicycleInterface::AngleDegrees angleStep;
	BicycleType type;

	std::shared_ptr<BicycleInterface> makeBicycle()
	{
		return this->type == Type_Simple ?
			std::make_shared<SimpleBicycle>() :
			std::make_shared<LengthBicycle>(this->length);
	}

	const char *getBicycleTag()
	{
		return this->type == Type_Simple ?
			"SimpleBicycle" :
			"LengthBicycle";
	}
public:
	SimulationBuilder()
		:length(0), speedStep(0), angleStep(0), type(Type_Simple){}

	SimulationBuilder &setBicycleLength(const CoordUnit &length)
	{
		this->length = length;
		return *this;
	}

	SimulationBuilder &setBicycleType(BicycleType type)
	{
		this->type = type;
		return *this;
	}
	
	SimulationBuilder &setSimulationSpeedStep(const CoordUnit &speedStep)
	{
		this->speedStep = speedStep;
		return *this;
	}

	SimulationBuilder &setSimulationAngleStep(const CoordUnit &angleStep)
	{
		this->angleStep = angleStep;
		return *this;
	}

	SimulationBuilder &addSimulationObserver(ObserverPtr ptr)
	{
		this->observers.add(ptr);
		return *this;
	}

	virtual std::shared_ptr<SimulationInterface> buildSimulation(
		std::shared_ptr<SimulationObserver> observerUi) = 0;
};

class Simulation : public SimulationInterface {
	SimulationObserverGroup observers;
	std::shared_ptr<BicycleInterface> simulated;
private:
	Time stepTime;
	AngleDegrees stepAngle;
	Speed stepSpeed;
	Time simulationTime;
	const char *name;

public:
	Simulation(const Time &simStep, const AngleDegrees &angleStep,
		const Speed &speedStep, 
		std::shared_ptr<BicycleInterface> simulated,
		const char *simtag,
		const SimulationObserverGroup &observers)
		: observers(observers), simulated(simulated), 
		stepTime(simStep), stepAngle(angleStep),
		stepSpeed(speedStep),simulationTime(0.0),
		name(simtag)
	{
	}

	void advance()
	{
		advance(stepTime);
	}

	void advance(const Time &step)
	{
		simulated->advance(step);
		simulationTime += step;
		observers.notify(simulated, name, simulationTime, step);
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
protected:
	virtual BicycleInterface * getSimulated() override
	{
		return simulated.get();
	}

	virtual const Speed & getSpeedDelta() override
	{
		return stepSpeed;
	}

	virtual const AngleDegrees & getAngleDelta() override
	{
		return stepAngle;
	}

	virtual const Time & getTimeDelta() override
	{
		return stepTime;
	}

	virtual const Time & getSimulationTime() override
	{
		return simulationTime;
	}
};


class StepSimulationBuilder : public SimulationBuilder {

public:
	virtual std::shared_ptr<SimulationInterface> buildSimulation(
		std::shared_ptr<SimulationObserver> observerUi) override
	{
		this->observers.add(observerUi);
		/*
		const Time &simStep, const AngleDegrees &angleStep,
		const Speed &speedStep,
		std::shared_ptr<BicycleInterface> simulated,
		const char *simtag,
		const SimulationObserverGroup &observers
		*/
		return std::make_shared<Simulation>(0.05f,
			this->angleStep,
			this->speedStep,
			this->makeBicycle(), this->getBicycleTag(),
			this->observers);
	}

	// void setSimulation
};

/** @todo It will be the base class for different UI implementations.
 *
*/
class SimulationUi : public SimulationObserver {

public:
	virtual void notify(const std::shared_ptr<BicycleInterface> &sim,
		const char *tag, const Time &simTime, const Time &delta) = 0;
};

class ConsoleSimulationUi : public SimulationObserver,
public std::enable_shared_from_this<ConsoleSimulationUi> {
	std::shared_ptr<SimulationWithHandlers> sim;
	std::shared_ptr<SimulationBuilder> builder;
	// std::condition_variable var;
	bool quit;
	bool speedChangeEnabled;
	bool angleChangeEnabled;
	ConstraintUnit <BicycleInterface::CoordUnit> speedConstraints;
	ConstraintUnit <BicycleInterface::AngleDegrees> angleConstraints;
	
	template <typename T>
	class ConstraintsHandler : public SimulationWithHandlers::Handler<T> {
		const ConstraintUnit<Unit> &u;
	public:
		ConstraintsHandler(const ConstraintUnit<Unit> &u)
			: u(u)
		{
		}

		bool handle(const Unit &oldValue, const Unit &delta, Unit &newValue, bool &notify) override
		{
			notify = false;
			int rng = u.in_range(newValue);
			if (rng == 0) {
				return false;
			} else if (rng == 1) {
				newValue = u.upper();
			} else {
				newValue = u.lower();
			}
			return true;
		}
	};

	enum ConstraintState {
		ConstraintState_Disabled, ConstraintState_Enabled,
		ConstraintState_Upper, ConstraintState_Lower
	};

public:
	ConsoleSimulationUi(std::shared_ptr<SimulationBuilder> builder)
		: builder(builder), quit(false), speedConstraints(0.0, 20.0),
		angleConstraints(-30.0, 30.0)

	{
	}

	virtual void notify(const std::shared_ptr<BicycleInterface> &bicycle,
		const char *tag, const Time &simTime, const Time &delta) override
	{
		bicycle->getSpeed();
	}

	void loop()
	{
		char c;
		ObserverPtr obs = 
			std::static_pointer_cast<SimulationObserver>(this->shared_from_this());
		std::cout << "ConsoleUi usage: \n w<Enter> - accelerate;\n"
			" s<Enter> - deccelerate;\n a<Enter> - turn left;\n"
			" d<Enter> - turn right;\n e<Enter> - advance;\n"
			" q<Enter> - quit. You can also input an control "
			"string like \"wwwwwwwaaaaasssssddd\" - it will be a "
			"correct commands sequence." << std::endl;
		sim = std::make_shared<SimulationWithHandlers>(builder->buildSimulation(obs));
		
		typedef ConstraintsHandler<SimulationInterface::Speed> SpHdnl;
		typedef ConstraintsHandler<SimulationInterface::AngleDegrees> AnglHndl;
		/*SpHdnl spHndl(speedConstraints);
		AnglHndl anglHndl(angleConstraints);
		*/// std::shared_ptr<
		std::shared_ptr<SimulationWithHandlers::SpeedHandler> spptr = std::make_shared<SpHdnl>(speedConstraints);
		assert(_CrtCheckMemory() != 0);
		std::shared_ptr<SimulationWithHandlers::AngleHandler> anptr = std::make_shared<AnglHndl>(angleConstraints);
		assert(_CrtCheckMemory() != 0);
		sim->setAngleChangeHandler(anptr);
		sim->setSpeedChangeHandler(spptr);

		// sim = std::shared_ptr<SimulationInterface>(new SimulationWithHandlers(builder->buildSimulation(obs)));

		while (!quit) {
			sim->advance();
			std::cin >> c;
			switch (c) {
			case 'w':
				sim->accelerate();
				break;
			case 's':
				sim->deccelerate();
				break;
			case 'a':
				sim->turnLeft();
				break;
			case 'd':
				sim->turnRight();
				break;
			case 'e':
				break;
			case 'q':
			default:
				quit = true;
			}
		}
	}
};

int main(int argc, char *argv[])
{
	// Simulation sim(10.0f, 2.0f, 0.1f, 10.0);
	ObserverPtr simLogPtr = std::make_shared<InfoLog>();
	StepSimulationBuilder build;
	build.addSimulationObserver(simLogPtr)
		.setBicycleLength(10.0)
		.setBicycleType(SimulationBuilder::Type_Length)
		.setSimulationAngleStep(2.0)
		.setSimulationSpeedStep(10.0);
	
	// after this line the object 'build' is not valid.
	std::shared_ptr<SimulationBuilder> b = std::shared_ptr<StepSimulationBuilder>(&build);

	std::shared_ptr<ConsoleSimulationUi> ui = std::make_shared<ConsoleSimulationUi>(b);
	
	// note: it was removed because std::shared_from_this can't work with stack
//	ConsoleSimulationUi ui = ConsoleSimulationUi(b);

	// debug check. will be removed.
#ifdef _MSC_VER
	assert(_CrtCheckMemory() != 0);
#endif
	ui->loop();

	return 0;
}
