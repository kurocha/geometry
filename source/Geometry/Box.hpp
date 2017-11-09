//
//  Geometry/Box.h
//  This file is part of the "Euclid" project, and is released under the MIT license.
//
//  Created by Samuel Williams on 3/01/07.
//  Copyright (c) 2007 Samuel Williams. All rights reserved.
//
//

#pragma once

#include "Shape.hpp"

#include <Numerics/Matrix.hpp>
#include <Numerics/Interpolate.hpp>

namespace Geometry
{
	enum SubtractResolution {
		SUBTRACT_SMALLEST = 0,
		SUBTRACT_BIGGEST = 1,
		SUBTRACT_NEGATIVE = 2,
		SUBTRACT_POSITIVE = 3
	};

	enum BoxEdge {
		NEGATIVE_EDGE = 2,
		POSITIVE_EDGE = 3
	};

	template <std::size_t D, typename NumericT>
	class Sphere;

	/// An Axis Aligned Bounding Box.
	template <std::size_t D, typename NumericT = RealT>
	class Box
	{
	public:
		typedef Vector<D, NumericT> VectorT;
		typedef Vector<D, bool> BoolVectorT;

	protected:
		VectorT _min, _max;

		// This could be optimised by using a function ptr rather than a bool, but need to profile!
		inline bool compare_edge(const NumericT &a, const NumericT &b, bool allow_equality) const {
			if (allow_equality)
				return a <= b;
			else
				return a < b;
		}

	public:
		Box()
		{
		}

		Box(const Zero &) : _min(ZERO), _max(ZERO)
		{
		}

		Box (const Identity &, const NumericT &n = 1) : _min(ZERO), _max(n)
		{
		}

		Box (const VectorT & size) : _min(ZERO), _max(size) {
		}

		Box (const VectorT &min, const VectorT &max) : _min(min), _max(max)
		{
		}

		static Box from_center_and_size (const VectorT &center, const VectorT &size)
		{
			Box box;
			box.set_center_and_size(center, size);
			return box;
		}

		static Box from_origin_and_size (const VectorT &origin, const VectorT &size)
		{
			Box box;
			box.set_origin_and_size(origin, size);
			return box;
		}

		// Bounds _min and _max must infact be minima and maxima.
		// This function establishes this condition.
		void normalize_bounds () {
			VectorT c(_min);

			_min = _min.constrain(_max, false);
			_max = _max.constrain(c, true);

			for (std::size_t i = 0; i < D; i += 1) {
				_min[i] = std::min(_min[i], _max[i]);
				_max[i] = std::max(_min[i], _max[i]);
			}
		}

		// Copy Semantics
		template <typename M>
		Box (const Box<D, M> &other) : _min(other.min()), _max(other.max()) {}

		template <typename M>
		Box & operator= (const Box<D, M> &other) {
			_min = other.min(); _max = other.max();
		}

		/// Returns true if the box does not enclose any space.
		bool is_degenerate() const { return _min == _max; }

		/// Returns true if the box is in normal form - i.e. the minimum point is lesser than the maximum point.
		bool is_normal() const {return point(_min) < point(_max);}

		/// Test for exact equivalence
		bool operator==(const Box & other) const
		{
			return min() == other.min() && max() == other.max();
		}

		bool operator!=(const Box & other) const
		{
			return !(*this == other);
		}

		bool operator<(const Box & other) const
		{
			return _min < other._min && _max < other._max;
		}

		bool equivalent(const Box & other) const
		{
			return min().equivalent(other.min()) && max().equivalent(other.max());
		}

		/// Returns the minimum point of the box:
		const VectorT & min() const {return _min;}
		VectorT & min () {return _min;}
		
		/// Returns the maximum point of the box:
		const VectorT & max() const {return _max;}
		VectorT & max() {return _max;}

		/// Returns the origin of the box.
		/// @sa min()
		const VectorT& origin () const { return _min; }

		/// Returns the center of the box.
		VectorT center () const { return (_min + _max) / NumericT(2); }

		/// Returns the size of the box.
		VectorT size () const { return _max - _min; }

		/// Set a particular corner.
		void set_corner(const BoolVectorT & cnr, const VectorT & adj)
		{
			for (std::size_t axis = 0; axis < D; axis += 1)
				set_corner(cnr, axis, adj[axis]);
		}

		/// Set the value for a particular axis for a given corner.
		void set_corner (const BoolVectorT & cnr, const std::size_t & axis, const NumericT & amnt)
		{
			if (cnr[axis])
				_max[axis] = amnt;
			else
				_min[axis] = amnt;
		}

		/// Set the center and size of the box.
		/// @sa from_center_and_size()
		void set_center_and_size (const VectorT &center, const VectorT &size)
		{
			_min = center - (size/2);
			_max = center + (size/2);
		}

		/// Resize the box but keep the same origin.
		void set_size_from_origin (const VectorT & size)
		{
			_max = _min + size;
		}

		/// Set a new origin and size.
		void set_origin_and_size (const VectorT &origin, const VectorT &size)
		{
			_min = origin;
			this->set_size_from_origin(size);
		}

		/// Set a new origin, maintain the current size.
		/// @sa set_origin_and_size
		void set_origin (const VectorT & origin) {
			this->set_origin_and_size(origin, this->size());
		}

		/// Recenter the box, maintaining current size.
		Box& recenter_at (const VectorT & center)
		{
			Vector<D> half_current_size = size() / 2;
			_min = center - half_current_size;
			_max = center + half_current_size;

			return *this;
		}

		/// Translate the box in-place, maintaining the current size.
		Box& translate_by (const VectorT & offset)
		{
			_min += offset;
			_max += offset;

			return *this;
		}

		/// Return a copy of this box, translated by the given offset.
		Box translated_by (const VectorT & offset) const {
			return Box(*this).translate_by(offset);
		}

		/// Adjust an individual axis of the box by the given amount
		void shift_axis (std::size_t axis, const NumericT &amount) {
			_min[axis] += amount;
			_max[axis] += amount;
		}

		/// Return a particular corner of the box, given by an index vector
		/// The components of d must be either false or true, and this will select the value from either min or max respectively.
		inline VectorT corner (const BoolVectorT & d) const {
			VectorT result;

			for (std::size_t i = 0; i < D; i += 1)
				result[i] = d[i] ? _max[i] : _min[i];

			return result;
		}

		/// Expand the box to include the other point.
		Box& union_with_point (const VectorT & point) {
			_min = _min.constrain(point, false);
			_max = _max.constrain(point, true);

			return *this;
		}

		/// Expand the box to include the other box.
		Box& union_with_box (const Box & other) {
			Vector<D> t(ZERO);

			_min = _min.constrain(other.min(), false);
			_max = _max.constrain(other.max(), true);

			return *this;
		}

		/// Clip the current box to the given box. This function is very simple.
		/// Case 1: this and other do not intersect; result is box the same as other
		/// Case 2: this and other intersect partially; the result is that this will be resized to fit completely within other
		/// Case 3: this is completely within other; no change will occur.
		/// @sa intersects_with()
		Box& clip_to_box (const Box & other)
		{
			_min = _min.constrain(other._min, true);
			_max = _max.constrain(other._max, false);

			return *this;
		}

		/// Given an orientation, aligns this box within a superbox.
		void align_within_super_box (const Box & super_box, const Vector<D> & orientation)
		{
			for (std::size_t i = 0; i < D; ++i) {
				RealT width = _max[i] - _min[i];
				RealT super_width = super_box._max[i] - super_box._min[i];
				RealT scale = super_width - width;
				RealT offset = orientation[i] * scale;
				RealT distance = (super_box._min[i] + offset) - _min[i];

				shift_axis(i, distance);
			}
		}

		/// Returns the orientation of one box relative to another.
		Vector<D> orientation_of (const Box &other) const {
			Vector<D> o;

			for (std::size_t i = 0; i < D; ++i) {
				if (other._min[i] < _min[i])
					o[i] = 0;
				else if (other._max[i] > _max[i])
					o[i] = 1;
				else {
					RealT min_width = other._max[i] - other._min[i];
					RealT this_width = _max[i] - _min[i];
					RealT s = this_width - min_width;
					RealT offset = other._min[i] - _min[i];

					if (s == 0)
						o[i] = 0.5;
					else
						o[i] = offset / s;
				}
			}

			return o;
		}

		/// Find the absolute offset of a point within the box.
		Vector<D> offset_of(const Vector<D> & point) const {
			return point - _min;
		}

		/// Find the relative position of a point inside the box.
		Vector<D> relative_offset_of(const Vector<D> &point) const {
			return offset_of(point) / size();
		}

		/// Calculate an absolute point based on a relative offset.
		Vector<D> absolute_position_of(const Vector<D> &offset) const {
			return _min + (size() * offset);
		}

		/// Returns a sphere that encloses the entire box.
		Sphere<D, NumericT> bounding_sphere() const
		{
			return {center(), (center() - _min).length()};
		}

		/// Tests whether this is completely within other.
		/// @returns true when this is within other, depending on the includes_edges parameter.
		/// @sa contains_point()
		bool contains_box(const Box<D, NumericT> & other, bool includes_edges = true) const {
			return contains_point(other.min(), includes_edges) && contains_point(other.max(), includes_edges);
		}

		/// Tests whether the box contains a point.
		/// @returns true when the point is within the box, or on an edge, depending on the includes_edges parameter.
		bool contains_point(const Point<D, NumericT> & point, bool includes_edges = true) const
		{
			bool result = false;

			if (!includes_edges)
				result = (point < _max) && (point > _min);
			else
				result = (point <= _max) && (point >= _min);

			return result;
		}

		/// Tests whether this box intersects with another box.
		/// @returns true when the two boxes overlap, or edges touch, depending on includes_edges parameter.
		bool intersects_with (const Box<D, NumericT> & other, bool includes_edges = true) const
		{
			for (std::size_t i = 0; i < D; ++i) {
				if (compare_edge(_max[i], other._min[i], !includes_edges) || compare_edge(other._max[i], _min[i], !includes_edges))
					return false;
			}

			return true;
		}

		// Ordered subtraction methods
		void subtract_in_order(const Box & other, const Vector<D, std::size_t> & order) {
			subtract_in_order(other, order, Vector<D, unsigned>(SUBTRACT_SMALLEST));
		}

		void subtract_in_order(const Box & other, const Vector<D, std::size_t> & order, const Vector<D, unsigned> & cuts)
		{
			// This function is fairly complex, for a good reason - it does a fairly complex geometric operation.
			// This operation can be summarised as subtracting one box from another. The reason why this is complex is because there are many edge cases to
			// consider, and this function works for boxes in n std::size_ts.

			Vector<D, std::size_t> k(2);

			// Total number of corners for a given D.
			const std::size_t CORNERS = 1 << D;

			// We consider every corner and its opporsite.
			// We do this because we need to consider if the corner intersects the other shape.
			for (std::size_t c = 0; c < CORNERS; c += 1) {
				// We consider both the given corner and its opporsite
				BoolVectorT current_corner(k.distribute(c));
				BoolVectorT opporsite_corner(!current_corner);

				VectorT this_current_corner(corner(current_corner)), this_opporsite_corner(corner(opporsite_corner));
				VectorT other_current_corner(other.corner(current_corner)), other_opporsite_corner(other.corner(opporsite_corner));

				bool intersects = contains_point(other_current_corner, true) || other.contains_point(this_opporsite_corner, true);

				// We consider each axis for these corners
				for (std::size_t axis_index = 0; axis_index < D; axis_index += 1) {
					// We pick out the current axis
					std::size_t axis = order[axis_index];

					// We consider the lines on this axis
					NumericT a1 = this_current_corner[axis];
					NumericT a2 = this_opporsite_corner[axis];
					NumericT b1 = other_current_corner[axis];
					NumericT b2 = other_opporsite_corner[axis];

					// We copy the corner vectors just in case we need to swap them
					BoolVectorT c1(current_corner), c2(opporsite_corner);

					// We need to compare things relatively, so a1 should be smaller than a2.
					// if not, we swap everything around.
					if (a1 > a2) {
						std::swap(a1, a2);
						std::swap(b1, b2);
						std::swap(c1, c2);
					}

					if (b1 > a1 && b2 >= a2 && intersects) {
						// Remove the right hand segment a2 = b1
						set_corner(c2, axis, b1);
						break;
					}

					if (b1 <= a1 && b2 < a2 && intersects) {
						// Remove the left hand segment a1 = b2
						set_corner(c1, axis, b2);
						break;
					}

					if (a1 < b1 && a2 > b2 && intersects_with(other)) {
						// The line is being split in two by the subtracted segment.
						// We will choose the larger segment
						RealT left_side = b1 - a1;
						RealT right_side = a2 - b2;

						// We use cuts to determine how to resolve these cases with more than one result
						if (cuts[axis] == SUBTRACT_POSITIVE || (left_side > right_side && cuts[axis] == SUBTRACT_SMALLEST)) {
							// remove right side (positive side)
							set_corner(c2, axis, b1);
						} else {
							// remove left side (negative side)
							set_corner(c1, axis, b2);
						}

						break;
					}

					if (a1 > b1 && a2 < b2) {
						// This case is when we are subtracting more than we have available,
						// ie, both ends of the line are within the subtracting bounds
						// There is nothing we can do reasonably within this case, so we just ignore it
						// and hope for the best ^_^.
					}
				}
			}
		}

		// This just subtracts a single edge from another box, essentially a helper for subtract_using_translation
		bool subtract_edge(const Box & other, std::size_t axis, const BoxEdge & edge, const NumericT & offset = 0)
		{
			NumericT a, b, c;

			// Offset indicates the distance the edge must be from the other box if they are on top of each other, ie a == b, then a -> a + offset

			if (edge == NEGATIVE_EDGE) {
				a = _min[axis];
				b = other._max[axis];
				c = _max[axis];

				if ((b + offset) < c && compare_edge(a, b, !Numerics::equivalent(offset, 0))) {
					//std::cout << "Adjusting [" << axis << "] from " << a << " to " << b - offset << std::endl;
					_min[axis] = b + offset;
					return true;
				}
			} else {
				a = _max[axis];
				b = other._min[axis];
				c = _min[axis];

				if ((b + offset) > c && compare_edge(b, a, !Numerics::equivalent(offset, 0))) {
					//std::cout << "Adjusting [" << axis << "] from " << a << " to " << b - offset << std::endl;
					_max[axis] = b - offset;
					return true;
				}
			}

			//std::cout << "Did not adjust [" << axis << "] from " << a << " to " << b - offset << " other " << c << std::endl;
			return false;
		}

		// Translation based subtraction methods
		// These methods assume that only the edges specified by the orientation may overlap. For a more general
		// approach, the old methods may be more useful.
		// These methods remove the need for a lot of complex maths, and thus are faster. However, subtract_in_order
		// Is guaranteed to work in ALL cases.
		Box subtract_using_translation(const Box & from, const Box & to, const NumericT & offset)
		{
			VectorT orientation = from.orientation_of(to);
			Box translation = from;

			for (std::size_t i = 0; i < D; ++i) {
				Box to_copy = to;
				bool result = false;

				if (orientation[i] == 0.0) {
					result = to_copy.subtract_edge(*this, i, NEGATIVE_EDGE, offset);
				} else if (orientation[i] == 1.0) {
					result = to_copy.subtract_edge(*this, i, POSITIVE_EDGE, offset);
				}

				if (result) {
					to_copy.union_with_box(translation);

					result = to_copy.intersects_with(*this, false);

					if (!result)
						translation.union_with_box(to_copy);
				}
			}

			return translation;
		}
		
		Box bounding_box() const {
			return *this;
		}
	};

	typedef Box<2, RealT> Box2;
	typedef Box<3, RealT> Box3;
	typedef Box<2, int> Box2i;
	typedef Box<3, int> Box3i;
	typedef Box<2, unsigned> Box2u;
	typedef Box<3, unsigned> Box3u;

	/// Return an ortographic projection as described by the given Box.
	template <typename NumericT>
	Matrix<4, 4, NumericT> orthographic_projection_matrix (const Box<3, NumericT> & box) {
		return orthographic_projection_matrix(box.min(), box.max());
	}
}
