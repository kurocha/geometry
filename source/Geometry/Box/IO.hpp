//
//  Geometry/AlignedBox.IO.hpp
//  This file is part of the "Euclid" project, and is released under the MIT license.
//
//  Created by Samuel Williams on 30/08/2017.
//  Copyright (c) 2017 Samuel Williams. All rights reserved.
//
//

#pragma once

#include "../Box.hpp"
#include <iostream>
#include <Numerics/Vector/IO.hpp>

namespace Geometry {
	template <std::size_t D, typename NumericT>
	std::ostream & operator<<(std::ostream & output, const Box<D, NumericT> & box)
	{
		return output
			<< '{'
			<< box.min()
			<< ','
			<< box.max()
			<< '}';
	}
	
	template <std::size_t D, typename NumericT>
	std::istream & operator>>(std::istream & input, Box<D, NumericT> & box)
	{
		return input
			>> expect_character<'{'>
			>> box.min()
			>> expect_character<','>
			>> box.max()
			>> expect_character<'}'>;
	}
}
