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
	typedef CoordUnit Speed;

	virtual void advance(const TimeUnit &units) = 0;
	virtual void setFrontWheelRotation(const AngleDegrees &units) = 0;
	virtual void setSpeed(const Speed &units) = 0;
	virtual void modifySpeed(const CoordUnit &delta) = 0;
	virtual void modifyFrontWheelRotation(const AngleDegrees &delta) = 0;

	// Getters.
	// Note: returning by-value is intentionally left here.

	virtual const Speed getSpeed() = 0;  // v
	virtual const CoordUnit getLength() = 0; // L
	virtual const AngleDegrees getFrontWheelRotation() = 0; // delta
	virtual const AngleDegrees getVehicleRotation() = 0; // phi
	virtual void getFrontWheelCoords(UnitPoint &pt) = 0; // { Xf, Yf }
	virtual void getRearWheelCoords(UnitPoint &pt) = 0;  // { X , Y  }
};

class BicycleInterface : public BicycleT<float, float, float> {

};