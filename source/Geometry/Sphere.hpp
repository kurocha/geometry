//
//  Geometry/Sphere.h
//  This file is part of the "Euclid" project, and is released under the MIT license.
//
//  Created by Samuel Williams on 18/10/06.
//  Copyright (c) 2006 Samuel Williams. All rights reserved.
//
//

#pragma once

#include "Shape.hpp"

namespace Geometry
{
	template <std::size_t D, typename NumericT>
	class Line;
	
	template <std::size_t D, typename NumericT>
	class LineSegment;
	
	/// A sphere is defined by its center point and its radius. It is fast for using with collision detection, as the comparison between a sphere and other geometric objects is typically very fast due to the simple nature of a sphere.
	template <std::size_t D, typename NumericT = RealT>
	class Sphere
	{
	protected:
		Vector<D, NumericT> _center;
		NumericT _radius;

		typedef typename RealTypeTraits<NumericT>::RealT RealT;

	public:
		/// Undefined constructor
		Sphere() = default;

		/// Zero constructor. Creates a sphere centered at coordinate zero, with radius zero.
		Sphere(const NumericT & radius) : _center(ZERO), _radius(radius)
		{
		}

		/// Full constructor. Supply all parameters.
		Sphere(Vector<D, NumericT> center, NumericT radius) : _center(center), _radius(radius)
		{
		}

		/// @returns the center of the sphere.
		const Vector<D, NumericT> & center() const
		{
			return _center;
		}

		/// Sets the center of the sphere.
		void set_center(const Vector<D> & center)
		{
			_center = center;
		}

		/// @returns the radius of the sphere.
		const NumericT & radius() const
		{
			return _radius;
		}

		/// Sets the center of the sphere.
		void set_radius(const NumericT & radius)
		{
			_radius = radius;
		}

		/// Displacement returns the vector between the two circles.
		RealT distance_between_edges(const Sphere<D, NumericT> & other, Vector<D, NumericT> & displacement) const
		{
			displacement = _center - other._center;
			
			auto total_radius = _radius + other._radius;
			return displacement.length() - RealT(total_radius);

		}

		/// Displacement returns the vector between the two centers.
		RealT distance_from_point(const Vector<D, NumericT> & point, Vector<D, NumericT> & displacement) const
		{
			displacement = point - _center;

			return displacement.length() - RealT(_radius);
		}

		Intersection intersects_with(const Line<D, NumericT> &line, RealT & entry_time, RealT & exit_time) const
		{
			//Optimized method sphere/ray intersection
			auto dst = line.point() - _center;
		
			RealT b = dst.dot(line.direction());
			RealT c = dst.dot(dst) - (_radius * _radius);
		
			// If d is negative there are no real roots, so return
			// false as ray misses sphere
			RealT d = b * b - c;
		
			if (d == 0.0) {
				entry_time = (-b) - std::sqrt(d);
				exit_time = entry_time;
				return Intersection::TOUCH;
			}
		
			if (d > 0) {
				entry_time = (-b) - std::sqrt(d);
				exit_time = (-b) + std::sqrt(d);
				return Intersection::OVERLAP;
			} else {
				return Intersection::DISJOINT;
			}
		}
		
		Intersection intersects_with(const LineSegment<D, NumericT> & segment, RealT & entry_time, RealT & exit_time) const
		{
			auto line = segment.to_line();
		
			Intersection result = intersects_with(line, entry_time, exit_time);
		
			// time_at_intersection is the time in unit length from segment.start() in direction segment.offset()
			// We will normalize the time in units of segment.offset()s, so that segment.point_at_time(time_at_intersection)
			// works as expected
		
			RealT n = segment.offset().length();
			entry_time = entry_time / n;
			exit_time = exit_time / n;
		
			// The line segment intersects the sphere at one or two points:
			if ((entry_time >= 0.0 && entry_time <= 1.0) || (exit_time >= 0.0 && exit_time <= 1.0)) {
				return result;
			}
		
			// Line segment is completely within sphere:
			if (entry_time < 0.0 && exit_time > 1.0) {
				return Intersection::ENCLOSED;
			}
		
			// The line segment did not intersect.
			return Intersection::DISJOINT;
		}
		
		Intersection intersects_with(const Sphere & other, Vector<D, NumericT> & displacement) const
		{
			auto d = distance_between_edges(other, displacement);
		
			if (d < 0.0) {
				return Intersection::OVERLAP;
			} else if (d == 0.0) {
				return Intersection::TOUCH;
			} else {
				return Intersection::DISJOINT;
			}
		}
		
		Intersection intersects_with(const Vector<D, NumericT> & point, Vector<D, NumericT> & displacement) const
		{
			auto d = distance_from_point(point, displacement);
		
			if (d < 0.0) {
				return Intersection::OVERLAP;
			} else if (d == 0.0) {
				return Intersection::TOUCH;
			} else {
				return Intersection::DISJOINT;
			}
		}
	};

	typedef Sphere<2> Sphere2;
	typedef Sphere<3> Sphere3;

	extern template class Sphere<2>;
	extern template class Sphere<3>;
}
