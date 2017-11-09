//
//  Geometry/Geometry.h
//  This file is part of the "Euclid" project, and is released under the MIT license.
//
//  Created by Samuel Williams on 11/11/08.
//  Copyright (c) 2008 Samuel Williams. All rights reserved.
//
//

#pragma once

#include "Point.hpp"

#include <array>

namespace Geometry
{
	/// A general abstraction of a shape in D-space with P points. Used as a base class for several shapes. Provides access to points and some general functions.
	template <std::size_t D, std::size_t P, typename NumericT>
	struct Shape : public std::array<Point<D, NumericT>, P>
	{
		using VectorT = Vector<D, NumericT>;
		using PointT = Point<D, NumericT>;

		Shape() = default;

		template <typename... ArgumentsT>
		Shape(ArgumentsT... arguments) : std::array<Point<D, NumericT>, P>{{arguments...}} {}

		VectorT center() const
		{
			VectorT total((*this)[0]);
			
			for (std::size_t i = 1; i < P; i++)
				total += (*this)[i];

			return total / (NumericT)P;
		}
	};

	/// Surface normal for any 2-dimentional shape:
	template <std::size_t P, typename NumericT>
	Vector<3, NumericT> surface_normal(const Shape<2, P, NumericT> & shape)
	{
		return {0, 0, 1};
	}

	/// Surface normal for 3-dimentional triangles:
	template <typename NumericT>
	Vector<3, NumericT> surface_normal(const Shape<3, 3, NumericT> & shape)
	{
		return surface_normal(shape[0], shape[1], shape[2]);
	}

	/// An intersection test generally has three kinds of results which are distinct. There was no intersection, the edges touched, or the shapes overlapped.
	enum Intersection
	{
		/// There is no geometric intersection.
		DISJOINT = 0,

		/// Edges of the shapes touch, but the shapes themselves do not overlap.
		TOUCH = 16,

		/// The shapes intersect.
		OVERLAP = 32,

		/// The shape being tested is completely embedded.
		ENCLOSED = 64
	};

	enum Direction
	{
		LEFT   = 1 << 0, // - x-axis
		RIGHT  = 1 << 1, // + x-axis
		BOTTOM = 1 << 2, // - y-axis
		TOP    = 1 << 3, // + y-axis
		NEAR   = 1 << 4, // - z-axis
		FAR    = 1 << 5 // + z-axis
	};
	
	// /*
	//     Geometry is very inter-dependant, for example, a plane can be constructed from a triangle, but a triangle also relies on planes for intersection tests. Therefore, we predefine all general geometry classes here.
	//  */
	// 
	// template <std::size_t D, typename NumericT = RealT>
	// class Triangle;
	// 
	// template <std::size_t D, typename NumericT = RealT>
	// class Sphere;
	// 
	// template <std::size_t D, typename NumericT = RealT>
	// class Plane;
	// 
	// template <std::size_t D, typename NumericT = RealT>
	// class Line;
	// 
	// template <std::size_t D, typename NumericT = RealT>
	// class LineSegment;
	// 
	// template <std::size_t D, typename NumericT = RealT>
	// class Box;
}
