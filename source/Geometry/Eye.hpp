//
//  Geometry/Viewport.h
//  This file is part of the "Euclid" project, and is released under the MIT license.
//
//  Created by Samuel Williams on 5/06/11.
//  Copyright (c) 2011 Samuel Williams. All rights reserved.
//

#pragma once

#include "Line.hpp"
#include "Box.hpp"

namespace Geometry
{
	/**
		We assume a right-hand coordinate system (all modern graphics APIs use this) with a clip box going from 0..1.
	*/
	template <typename NumericT = RealT>
	struct Eye {
		Vector<3, NumericT> origin;
		Line<3, NumericT> forward;
		Vector<3, NumericT> up;
		
		struct Transformation {
			Matrix<4, 4, NumericT> inverse_projection_matrix;
			Matrix<4, 4, NumericT> inverse_view_matrix;
			NumericT near, far;
			
			Vector<4, NumericT> convert_from_projection_space_to_object_space(Vector<4, NumericT> coordinate) const;
			
			/// Calculate the object-space coordinates when given the window's viewport and a point in the viewport.
			Eye convert_from_viewport_space_to_object_space (const Box<2, NumericT> & viewport, const Vector<2, NumericT> & point) const;

			/// Calculate the object-space coordinates when given a projection-space coordinate on the near plane.
			Eye convert_from_normalized_space_to_object_space (Vector<3, NumericT> normalized_point) const;
		};
		
		/// Basically an implementation of gluLookAt, but slightly simpler due to the following constraints:
		/// - direction is already normalized and points from _origin in the direction we are currently looking in
		/// - up is already normalized
		static Matrix<4, 4, NumericT> look_at(const Vector<3, NumericT> & origin, const Vector<3, NumericT> & direction, const Vector<3, NumericT> & up);
		
		Matrix<4, 4, NumericT> look_at() const;
	};
	
	template <typename NumericT>
	typename Eye<NumericT>::Transformation eye_transformation(const Matrix<4, 4, NumericT> & projection_matrix, const Matrix<4, 4, NumericT> & view_matrix, NumericT near = 0, NumericT far = 1) {
		return typename Eye<NumericT>::Transformation {
			inverse(projection_matrix),
			inverse(view_matrix),
			near, far
		};
	}
	
	extern template struct Eye<float>;
	extern template struct Eye<double>;
}
