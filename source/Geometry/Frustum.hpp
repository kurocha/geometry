//
//  Geometry/Frustum.h
//  This file is part of the "Euclid" project, and is released under the MIT license.
//
//  Created by Samuel Williams on 18/10/06.
//  Copyright (c) 2006 Samuel Williams. All rights reserved.
//
//

#pragma once

#include "Plane.hpp"

#include <Numerics/Matrix.hpp>

namespace Geometry
{
	template <typename NumericT = RealT>
	class Frustum {
	public:
		typedef Plane<3, NumericT> PlaneT;
		typedef Vector<3, NumericT> Vec3T;

	protected:
		PlaneT _planes[6];
		Vec3T _corners[8];

		Vec3T _near_center, _far_center;

		void build_frustrum_from_matrix (const Matrix<4, 4, NumericT> &);

	public:
		Frustum(const Matrix<4, 4, NumericT> & view_matrix) {
			build_frustrum_from_matrix(view_matrix);
		}

		const PlaneT & operator[] (std::size_t i) const {
			return _planes[i];
		}

		bool intersects_with (const Sphere<3, NumericT> & s) const;
		bool intersects_with (const Box<3, NumericT> & b) const;

		Box<3, NumericT> bounding_box() const;

		//bool intersects_with (const Triangle &t) const;

		//bool contains_point (const Vec3 &p) const;

		bool visible(Vec3T planar_normal);
	};

	typedef Frustum<> FrustumT;
	
	template <typename NumericT>
	inline Plane<3, NumericT> convert_plane_from_matrix_eqn (const NumericT & a, const NumericT & b, const NumericT & c, NumericT distance)
	{
		Vector<3, NumericT> normal(a, b, c);

		// Pseudo-normalize all components
		NumericT m = normal.length();

		normal /= m;
		distance /= m;

		return Plane<3, NumericT>(distance, normal);
	}

	template <typename NumericT>
	void Frustum<NumericT>::build_frustrum_from_matrix (const Matrix<4, 4, NumericT> & m)
	{
		// The constructor used for Plane will normalize its elements automatically

		// Left   (m3 + m0)
		_planes[0] = convert_plane_from_matrix_eqn(m.at(3, 0)+m.at(0, 0), m.at(3, 1)+m.at(0, 1), m.at(3, 2)+m.at(0, 2), (m.at(3, 3)+m.at(0, 3)));

		// Right  (m3 - m0)
		_planes[1] = convert_plane_from_matrix_eqn(m.at(3, 0)-m.at(0, 0), m.at(3, 1)-m.at(0, 1), m.at(3, 2)-m.at(0, 2), (m.at(3, 3)-m.at(0, 3)));

		// Top    (m3 - m1)
		_planes[2] = convert_plane_from_matrix_eqn(m.at(3, 0)-m.at(1, 0), m.at(3, 1)-m.at(1, 1), m.at(3, 2)-m.at(1, 2), (m.at(3, 3)-m.at(1, 3)));

		// Bottom (m3 + m1)
		_planes[3] = convert_plane_from_matrix_eqn(m.at(3, 0)+m.at(1, 0), m.at(3, 1)+m.at(1, 1), m.at(3, 2)+m.at(1, 2), (m.at(3, 3)+m.at(1, 3)));

		// Near
		_planes[4] = convert_plane_from_matrix_eqn(m.at(2, 0), m.at(2, 1), m.at(2, 2), m.at(2, 3));

		// Far    (m3 - m2)
		_planes[5] = convert_plane_from_matrix_eqn(m.at(3, 0)-m.at(2, 0), m.at(3, 1)-m.at(2, 1), m.at(3, 2)-m.at(2, 2), (m.at(3, 3)-m.at(2, 3)));

		// Calculate object-space points for box coordinates:
		Box<3, NumericT> clip_box(-1, 1);
		Vector<3, unsigned> k(2);

		auto inverse_view_matrix = inverse(m);
		for (std::size_t i = 0; i < 8; i += 1) {
			_corners[i] = inverse_view_matrix * clip_box.corner(k.distribute(i));
		}

		_near_center = inverse_view_matrix * Vec3T(0, 0, 1);
		_far_center = inverse_view_matrix * Vec3T(0, 0, -1);
	}

	template <typename NumericT>
	bool Frustum<NumericT>::intersects_with (const Sphere<3, NumericT> & s) const
	{
		for (std::size_t i = 0; i < 6; ++i) {
			NumericT d = _planes[i].distance_to_point(s.center());

			if (d <= -s.radius())
				return false;
		}

		return true;
	}

	template <typename NumericT>
	bool Frustum<NumericT>::intersects_with (const Box<3, NumericT> &b) const
	{
		return intersects_with(b.bounding_sphere());
	}

	template <typename NumericT>
	Box<3, NumericT> Frustum<NumericT>::bounding_box() const {
		Box<3, NumericT> box(_corners[0], _corners[7]);

		for (std::size_t i = 1; i < 7; i += 1)
			box.union_with_point(_corners[i]);

		return box;
	}

	/*
	 bool Frustum::contains_point (const Vec3 &p) const {
			for (std::size_t i = 0; i < 6; ++i) {
					RealT d = _planes[i].distance_to_point(p);

					if (d <= 0)
							return false;
			}

			return true;
	 }
	 */


	template <typename NumericT>
	bool Frustum<NumericT>::visible(Vec3T planar_normal) {
		return (_near_center - _far_center).normalize().dot(planar_normal) >= 0.0;
	}
	
	extern template class Frustum<RealT>;
}
