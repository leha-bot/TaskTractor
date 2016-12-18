/* @file DualSimulation.h Contains declaration of the DualSimulation class 
 *       which can flip the simulations.
 * @see DualSimulation::flip()
 */

#include "SimulationInterface.h"
#include "SimulationDecoratorBase.h"

class DualSimulation : public SimulationDecoratorBase {
protected:
	bool flipped = false;
	SimPtr second;
public:
	DualSimulation(SimPtr &first, SimPtr &second)
		: SimulationDecoratorBase(first),
		second(second)
	{}

	virtual int flip()
	{
		decorated.swap(second);
		flipped = !flipped;
		return flipped;
	}

};