/** @file Units.h Contains some typedefs for units.
*/
#pragma once

typedef float TimeUnit;
typedef TimeUnit Time;

#ifdef _DEBUG
#include <assert.h>
#else
#define assert(f)
#endif

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

template<typename T>
class ConstraintUnit {
	T minv, maxv, v;
	typedef const T &crefT;
public:
	ConstraintUnit(crefT minval, crefT maxval)
		: minv(minval), maxv(maxval)
	{
		assert(maxv > minv);
	}

	void lower(crefT val)
	{
		assert(val < maxv);
		minv = val;
	}

	crefT lower() const
	{
		return minv;
	}

	void upper(crefT val)
	{
		assert(val > minv);
		maxv = val;
	}

	crefT upper() const
	{
		return maxv;
	}

	crefT clamped() const
	{
		return v > maxv ? maxv :
			v < minv ? minv : v;
	}

	crefT value() const
	{
		return v;
	}

	void clamp(T &val) const
	{
		val = val > maxv ? maxv :
			val < minv ? minv : val;
	}

	crefT clamp()
	{
		v = clamped();
		return v;
	}

	int in_range(crefT val) const
	{
		return val >= maxv ? 1 :
			val <= minv ? -1 : 0;
	}

	crefT setval(crefT newv)
	{
		v = newv;
		return clamp();
	}
};