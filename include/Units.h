/** @file Units.h Contains some typedefs for units.
*/
#pragma once
typedef float TimeUnit;
typedef TimeUnit Time;

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