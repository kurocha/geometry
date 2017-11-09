//
//  Geometry/Triangle.h
//  This file is part of the "Euclid" project, and is released under the MIT license.
//
//  Created by Samuel Williams on 15/03/06.
//  Copyright (c) 2006 Samuel Williams. All rights reserved.
//
//

#pragma once

#include "Shape.hpp"

namespace Geometry
{
	template <std::size_t D, typename NumericT>
	class Box;
	
	template <std::size_t D, typename NumericT = RealT>
	class Triangle : public Shape<D, 3, NumericT>
	{
	public:
		template <typename... ArgumentsT>
		Triangle(ArgumentsT... arguments) : Shape<D, 3, NumericT>(arguments...)
		{
		}

		Box<D, NumericT> bounding_box()
		{
			Box<D, NumericT> box((*this)[0], (*this)[1]);

			box.union_with_point((*this)[2]);

			return box;
		}
	};

	typedef Triangle<2, RealT> Triangle3;
	typedef Triangle<3, RealT> Triangle2;

	extern template class Triangle<2, RealT>;
	extern template class Triangle<3, RealT>;
}
