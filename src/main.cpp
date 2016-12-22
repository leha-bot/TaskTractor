/** @file main.cpp Contains entry point for Tractor Simulation program using 
 *  the Kinematic Bicycle Model.
 *  
 * @mainpage
 * # Compiling
 * 
 * ## Windows
 * Use the MS Visual Studio 2013 and compile the solution TaskTractor.sln;
 * If the Visual Studio 2013 is not installed, you can download the Visual Studio
 * Build Tools from site: https://www.microsoft.com/ru-ru/download/details.aspx?id=40760
 *
 * ## Linux
 * 
 * Make sure that you have the GCC and Make utilities and have required libraries.
 * There are some errors with multithreading
 * (see https://bugs.launchpad.net/ubuntu/+source/gcc-defaults/+bug/1228201)
 *
 * Brief contents:
 * This is a simple project which demonstrates the physical simulation based on
 * the article "Automatic Steering Methods for Autonomous Automobile Path Tracking".
 * 
 * LIMITATIONS
 * ===========
 * - The simulation is single-threaded (except the decorated InfoLog which checked only on Windows);
 * - The simulation did not implement the units conversion (but we 
 *   can simply changes the template parameter types to anothers);
 * - The simulation did not use the considered SMat.h (but also we can 
 *   change the default template argument);
 * - There are some plans for implementing GLFW GUI and some helper toolbars,
 *   but they didn't implemented (there are GLFW stubs);
 * - The simulation is command-line-based and works in step mode. You input the
 *   character (and press ENTER) or character string, the simulation will
 *   process it and you can input the another command (or q to quit);
 * - There are some plans to implement the command-line arguments parsing based
 *   on my old modified classes, but it didn't implemented yet;
 * - There are plans for cleaning, refactoring, documenting and moving all 
 *   classes to another modules from main.cpp.
 * - Implement Android bridge for GLFW and NativeActivity and push it to
 *   upstream (glfw.org);
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
 *
 * SIMULATION CONTROLLER
 * =====================
 * This is a class Simulation, which encapsulates the simulated bicycle model and
 * controls its behavior.
 *
 * Creating an instance of Simulation Controller
 * ---------------------------------------------
 * The Simulation instance is built via SimulationBuilder
 * interface which have chained setter methods and method 
 * build(ObserverPtr uiView). The ObserverPtr is the 
 * std::shared_ptr<SimulationObserver> which is notified by the controller if
 * the simulation has been advanced. These ObserverPtr's are stored in the 
 * class ObserverGroup which is derived from Observer and it notifies all its 
 * children (a typical Composite Design Pattern implementation).
 *
 * ### Multi-threading Observer
 * There is an experimental multi-threaded decorator for SimulationObserver 
 * which runs its
 * private method run() in separate std::thread and its method is waiting for 
 * the std::conditional_variable. While the master object (i.e. Simulation)
 * calls SimulationObserver::notify, this class sets the special guarding flag,
 * copying references for all arguments and calls method notify_one() of 
 * its conditional variable. This variable wakes up the sleeping thread and 
 * it checks flag, and if it is true, then calls method notify() of the 
 * decorated interface with stored args. Note that this class is designed for
 * queue-based queries. If the main thread calls notify() for this thread twice
 * and thread did not finish its work, then will be a race condition.
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
 * + f - toggle file / simulation mode;
 * + q - quit.
 */
#include "SimulationInterface.h"
#include "SimulationWithHandlers.h"
#include "FileSimulationWrap.h"
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
#include <ratio>
#include "DualSimulation.h"

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

/** @brief A simple "unicycle" model which assumes that length is 0, and rear 
 *        wheel and front wheel is equivalent.
 */
class SimpleBicycle : public BicycleInterface {
protected:
	UnitPoint position;
	//float x, y;
private:
	Speed speed;
	float angle_rads;
	AngleDegrees angle_degs;
	
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
		position.x = 0;
		position.y = 0;
		position.z = 0;
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
		position.x += units * x_sp_cos_angle;
		position.y += units * y_sp_sin_angle;
		// x += speed * units * cos(angle_rads);
		// y += speed * units * sin(angle_rads);
	}

	virtual void setFrontWheelRotation(const AngleDegrees &rotation) override
	{
		angle_degs = rotation;
		angle_rads = degs_to_rads(rotation);
		calc_angle_optimize_vars();
	}

	virtual void setSpeed(const Speed &units) override
	{
		speed = units;
		calc_speed_optimize_vars();
	}

	/** @note Assume that this bicycle is actually unicycle in this simplified class.
	 *
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
		pt = position;
	}

	virtual const float getVehicleRotation() override
	{
		return getFrontWheelRotation();
	}

	virtual void setVehicleRotation(const AngleDegrees &units) override
	{
		setFrontWheelRotation(units);
	}

	virtual void setCoords(const UnitPoint &pt) override
	{
		this->position = pt;
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
		// float angle_deg = getFrontWheelRotation();
		float ds = speed * units;

		position.x += ds * cos(vehicle_angle_rads);
		position.y += ds * sin(vehicle_angle_rads);
		
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

	virtual void setVehicleRotation(const AngleDegrees &units) override
	{
		vehicle_angle_rads = degs_to_rads(units);
	}

};

class InfoLog : public SimulationObserver {
public:

	virtual void notify(const std::shared_ptr<BicycleInterface> &sim,
		const char *tag, const Time &simTime, const Time &delta) override
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
		BicyclePtr simulated,
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

/** @brief Constructs instances of SimulationInterface (defined by parameters
 *         and BicycleInterface (defined by private member
 *         SimulationBuilder::BicycleType 's value.
 */
class SimulationBuilder : public SimulationLightBuilder {
public:
	typedef BicycleInterface::CoordUnit CoordUnit;
	typedef BicycleInterface::AngleDegrees AngleDegrees;
	
	enum BicycleType {
		Type_Simple,
		Type_Lengthy
	};

protected:
	SimulationObserverGroup observers;

	BicycleInterface::CoordUnit length;
	BicycleInterface::CoordUnit speedStep;
	BicycleInterface::AngleDegrees angleStep;
	BicycleType type;
	std::string filename;
	bool dual;

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
		:length(0), speedStep(0), angleStep(0), type(Type_Simple),
		dual(false) {}

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

	/** @note This method has been added for enabling the chained calls.
	 *
	 */
	SimulationBuilder &addSimulationObserver(SimObserverPtr &ptr)
	{
		this->observers.add(ptr);
		return *this;
	}

	SimulationBuilder &setFilename(const std::string &fname)
	{
		this->filename = fname;
		return *this;
	}

	/** @brief Sets the dual decorators mode.
	 *  @see DualSimulation.
	 */
	SimulationBuilder &setDual(bool dual)
	{
		this->dual = dual;
		return *this;
	}

	std::shared_ptr<SimulationInterface>
		buildSimulation(SimObserverPtr &observerUi)
	{
		this->observers.add(observerUi);

		return build();
	}

	// Inherited methods

	virtual void addObserver(SimObserverPtr &ptr) override
	{
		addSimulationObserver(ptr);
	}

	virtual SimPtr build() override
	{
		/*
		const Time &simStep, const AngleDegrees &angleStep,
		const Speed &speedStep,
		std::shared_ptr<BicycleInterface> simulated,
		const char *simtag,
		const SimulationObserverGroup &observers
		*/
		BicyclePtr bicycle = this->makeBicycle();
		const char *tag = this->getBicycleTag();
		SimPtr ret = std::make_shared<Simulation>(0.05f,
			this->angleStep,
			this->speedStep,
			bicycle, tag,
			this->observers);
		if (!this->filename.empty()) {
			SimPtr filesimptr = std::make_shared<FileSimulationWrap>(ret, this->filename.c_str());
			if (this->dual) {
				SimPtr ret2 = std::make_shared<Simulation>(0.05f,
					this->angleStep,
					this->speedStep,
					bicycle, tag,
					this->observers);

				SimPtr dualptr = std::make_shared<DualSimulation>(filesimptr, ret2);
				return dualptr;
			}
			return filesimptr;
		}

		return ret;
	}
};

void check_mem()
{
#ifdef _MSC_VER
	assert(_CrtCheckMemory());
#endif
}

/** @brief The simple Console interface. It works on all platforms supporting the stdlibc++.
 */
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
		
		typedef typename SimulationWithHandlers::Handler<T>::Unit Unit;
		
		const ConstraintUnit<Unit> &u;
	public:
		ConstraintsHandler(const ConstraintUnit<Unit> &u)
			: u(u)
		{
		}

		bool handle(const Unit &oldValue, const Unit &delta,
			Unit &newValue, bool &notify) override
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
		bool dual = true;
		char c;
		SimObserverPtr obs = 
			std::static_pointer_cast<SimulationObserver>(this->shared_from_this());

		std::cout << "ConsoleUi usage: \n w<Enter> - accelerate;\n"
			" s<Enter> - deccelerate;\n a<Enter> - turn left;\n"
			" d<Enter> - turn right;\n e<Enter> - advance;\n"
			" f<Enter> - toggle the file / simulation mode;\n"
			" q<Enter> - quit. You can also input an control "
			"string like \"wwwwwwwaaaaasssssddd\" - it will be a "
			"correct commands sequence." << std::endl;
		
		std::cout << "Now we are using a file simulation." << std::endl;
		
		builder->addObserver(obs);

		auto p = builder->build();
		sim = std::make_shared<SimulationWithHandlers>(p);
		
		typedef ConstraintsHandler<SimulationInterface::Speed> SpHdnl;
		typedef ConstraintsHandler<SimulationInterface::AngleDegrees> AnglHndl;
		
		std::shared_ptr<SimulationWithHandlers::SpeedHandler> spptr = std::make_shared<SpHdnl>(speedConstraints);
		
		std::shared_ptr<SimulationWithHandlers::AngleHandler> anptr = std::make_shared<AnglHndl>(angleConstraints);
		
		sim->setAngleChangeHandler(anptr);
		sim->setSpeedChangeHandler(spptr);

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
			case 'f':
				if (dual) {
					DualSimulation *dualptr = dynamic_cast<DualSimulation*>(sim->getDecorated().get());
					assert(dualptr != nullptr);
					int res = dualptr->flip();
					assert(res >= 0 && res <= 1);
					const char *info[] = {
						"Flipped to FileSim.",
						"Flipped to Sim",
					};
					std::cout << info[res] << std::endl;
				}
				break;
			case 'q':
			default:
				quit = true;
			}
		}
	}
};

#include <thread>

/** @brief Encapsulates Observer's logic in separate thread.
 */
class ThreadedObserverDecorator : public SimulationObserver {
	std::thread t;
	std::mutex m;
	std::condition_variable var;
	SimObserverPtr sim;
	bool need_exit;
	bool notified;

	const BicyclePtr *byc;
	const char *tag;
	const Time *simTime;
	const Time *delta;

	// private functions.
	void run()
	{
		std::unique_lock<std::mutex> lk(m);
		while (!need_exit)
		{
			var.wait(lk);
			if (!notified) {
				continue;
			}
			sim->notify(*byc, tag, *simTime, *delta);
		}
	}

	/** @note This method is not thread-safe.
	 */
	void setArgs(const BicyclePtr &sim, const char *tag,
		const Time &simTime, const Time &delta)
	{
		byc = &sim;
		this->tag = tag;
		this->simTime = &simTime;
		this->delta = &delta;
	}

public:
	ThreadedObserverDecorator(SimObserverPtr sim)
		:
		sim(sim), need_exit(0),
		notified(0),
		byc(nullptr), tag(nullptr),
		simTime(nullptr), delta(nullptr)
	{
		t = std::thread(&ThreadedObserverDecorator::run, this);
	}

	virtual void notify(const BicyclePtr &sim, const char *tag,
		const Time &simTime, const Time &delta) override
	{
		setArgs(sim, tag, simTime, delta);
		notified = true;
		var.notify_one();
	}

	/** @brief Signals the worker thread that we need to exit. */
	void needToExit()
	{
		need_exit = true;
		var.notify_one();
	}

	~ThreadedObserverDecorator()
	{
		needToExit();
		if (t.joinable())
		{
			t.join();
		}
	}
};

int main(int argc, char *argv[])
{

	// control flags
#ifdef _MSC_VER
	bool glfw = false;
	bool mt = false;
#else 
	bool mt = false;
#endif
	SimObserverPtr simLogPtr = std::make_shared<InfoLog>();

	if (mt) {
		try {
			// can throw.
			SimObserverPtr simMtLogPtr =
				std::make_shared<ThreadedObserverDecorator>(simLogPtr);
			simLogPtr = simMtLogPtr;

		} catch (std::system_error &e) {
			mt = false;
			std::cerr << "Catched std::system_error. what()" << e.what() <<
				"Multithreading Disabled." << std::endl;
		}
	}

	SimulationBuilder builder;
	builder.addSimulationObserver(simLogPtr)
		.setBicycleLength(10.0)
		.setBicycleType(SimulationBuilder::Type_Lengthy)
		.setSimulationAngleStep(2.0)
		.setSimulationSpeedStep(10.0);
	
	builder.setDual(true)
		.setFilename("sample.dat");

	// after this line the object 'build' is not valid (rvalue refs)
	std::shared_ptr<SimulationLightBuilder> b = 
		std::shared_ptr<SimulationBuilder>(&builder);
#ifdef _MSC_VER
	if (glfw) {
		std::shared_ptr<SimulationUi> getGlfwUI(std::shared_ptr<SimulationLightBuilder> &ptr);

		auto ui = getGlfwUI(b);
		ui->loop();
	}
#endif
	std::shared_ptr<ConsoleSimulationUi> ui =
		std::make_shared<ConsoleSimulationUi>(std::static_pointer_cast<SimulationBuilder>(b));
	
	// note: it was removed because std::shared_from_this can't work with stack
//	ConsoleSimulationUi ui = ConsoleSimulationUi(b);

	// debug check. will be removed.
#ifdef _MSC_VER
	assert(_CrtCheckMemory() != 0);
#endif
	ui->loop();
	return 0;
}
