/** @file BicycleInterface.h Contains a shared Bicycle interface.
*/
#pragma once

template <typename CoordUnitT>
struct UnitPointT {
	CoordUnitT x;
	CoordUnitT y;
	CoordUnitT z;
};

/** @brief A template abstract class for all our simulated Bicycles.
*
*/
template <typename CoordUnitT,
	typename TimeUnitT,
	typename AngleDegreesT>
class BicycleT {
public:
	typedef UnitPointT<CoordUnitT> UnitPoint;
	typedef CoordUnitT CoordUnit;
	typedef TimeUnitT TimeUnit;
	typedef AngleDegreesT AngleDegrees;

	virtual void advance(const TimeUnitT &units) = 0;
	virtual void setFrontWheelRotation(const AngleDegreesT &units) = 0;
	virtual void setSpeed(const CoordUnitT &units) = 0;
	virtual void modifySpeed(const CoordUnitT &delta) = 0;
	virtual void modifyFrontWheelRotation(const AngleDegreesT &delta) = 0;

	// Getters.

	virtual const CoordUnitT getSpeed() = 0;  // v
	virtual const CoordUnitT getLength() = 0; // L
	virtual const AngleDegreesT getFrontWheelRotation() = 0; // delta
	virtual const AngleDegreesT getVehicleRotation() = 0; // phi
	virtual void getFrontWheelCoords(UnitPoint &pt) = 0; // { Xf, Yf }
	virtual void getRearWheelCoords(UnitPoint &pt) = 0;  // { X , Y  }
};

class BicycleInterface : public BicycleT<float, float, float> {

};
