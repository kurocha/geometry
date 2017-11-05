//
//  Point.hpp
//  This file is part of the "Geometry" project and released under the MIT License.
//
//  Created by Samuel Williams on 4/11/2017.
//  Copyright, 2017, by Samuel Williams. All rights reserved.
//

#pragma once

#include <Numerics/Vector.hpp>

namespace Geometry
{
	// Represents a geometric point in space.
	template <std::size_t D, typename NumericT>
	class Point : public Numerics::Vector<D, NumericT>
	{
	public:
		using VectorT = Numerics::Vector<D, NumericT>;
		using Numerics::Vector<D, NumericT>::Vector;
		
		Point(const Numerics::Vector<D, NumericT> & vector) : Numerics::Vector<D, NumericT>(vector) {}
		
		/// Geometric comparison.
		/// @returns true if all components are numerically lesser than the others.
		bool operator<(const VectorT & other) const noexcept
		{
			for (std::size_t i = 0; i < D; ++i)
				if ((*this)[i] >= other[i])
					return false;

			return true;
		}

		/// Geometric comparison.
		/// @returns true if all components are numerically greater than the others.
		bool operator>(const VectorT & other) const noexcept
		{
			for (std::size_t i = 0; i < D; ++i)
				if ((*this)[i] <= other[i])
					return false;

			return true;
		}

		/// Geometric comparison.
		/// @returns true if all components are numerically lesser than or equal to the others.
		bool operator<=(const VectorT & other) const noexcept
		{
			for (std::size_t i = 0; i < D; ++i)
				if ((*this)[i] > other[i])
					return false;

			return true;
		}
		
		/// Geometric comparison.
		/// @returns true if all components are numerically greater than or equal to the others.
		bool operator>=(const VectorT & other) const noexcept
		{
			for (std::size_t i = 0; i < D; ++i)
				if ((*this)[i] < other[i])
					return false;

			return true;
		}
	};
	
	template <std::size_t D, typename NumericT>
	Point<D, NumericT> point(const Numerics::Vector<D, NumericT> & vector)
	{
		return vector;
	}
}
