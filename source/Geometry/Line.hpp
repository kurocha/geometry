//
//  Geometry/Line.h
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
	class AlignedBox;
	
	template <std::size_t D, typename NumericT = RealT>
	class Line {
	protected:
		typedef typename RealTypeTraits<NumericT>::RealT RealT;
		typedef Vector<D, NumericT> VectorT;

		VectorT _point;
		VectorT _direction;

	public:
		Line() {}

		Line(const VectorT & direction) : _point(ZERO), _direction(direction)
		{
		}

		Line(const VectorT & point, const VectorT & direction) : _point(point), _direction(direction)
		{
		}

		const VectorT & point () const { return _point; }
		const VectorT & direction () const { return _direction; }

		void set_point (const VectorT & point) { _point = point; }
		void set_direction (const VectorT & direction) { _direction = direction; }

		VectorT point_at_time (const NumericT & t) const {
			return _point + (_direction * t);
		}

		/// Returns the time on the line where a point is closest to the given point.
		NumericT time_for_closest_point (const VectorT & p3) const {
			auto p1 = _point;
			auto p2 = _point + _direction;

			auto d = _direction.length_squared();
			NumericT t = 0;

			for (std::size_t i = 0; i < D; ++i)
				t += (p3[i] - p1[i]) * (p2[i] - p1[i]);

			return t / d;
		}

		VectorT point_for_closest_point (const VectorT & p) const {
			return point_at_time(time_for_closest_point(p));
		}

		RealT shortest_distance_to_point (const VectorT &p) const {
			return (p - point_for_closest_point(p)).length();
		}

		// Calculates the factor for line equations
		RealT factor (const RealT & v, std::size_t i) const {
			return RealT(v + _point[i]) / RealT(_direction[i]);
		}

		Line<D-1, NumericT> reduce() const {
			return Line<D-1, NumericT>(point().reduce(), direction().reduce());
		}

		bool equivalent (const Line<D, NumericT> & other) {
			// Are we pointing in the same direction
			if (!_direction.equivalent(other._direction))
				return false;

			// Is the distance between the parallel lines equivalent to zero?
			return Numerics::equivalent(shortest_distance_to_point(other._point), 0);
		}

		bool intersects_with (const Line<D, NumericT> & other, NumericT & this_time, NumericT & other_time) const;
		bool intersects_with (const AlignedBox<D, NumericT> & other, NumericT & t1, NumericT & t2) const;

		/// Helper function for intersection testing where less information is needed.
		bool intersects_with (const AlignedBox<D, NumericT> & other, VectorT & at) const
		{
			NumericT t1, t2;

			bool result = intersects_with(other, t1, t2);
			if (result) at = point_at_time(t1);

			return result;
		}

		///@todo Implement this function
		/// bool clip_to_box (const AlignedBox<D, NumericT> & other, LineSegment<D, NumericT> & segment) const;

		/// Construct a line given two points
		static Line from (const VectorT & from, const VectorT & to) {
			return Line(from, (to - from).normalize());
		}
	};
	
	template <typename NumericT>
	Vector<2, NumericT> normal(const Line<2, NumericT> & line)
	{
		return normal(line.direction());
	}
	
	typedef Line<2> Line2;
	typedef Line<3> Line3;

	extern template class Line<2, RealT>;
	extern template class Line<3, RealT>;

	template <std::size_t D, typename NumericT>
	inline Line<D, NumericT> operator+ (const Line<D, NumericT> & l, const Vector<D, NumericT> & v);

	template <std::size_t D, typename NumericT>
	Line<D, NumericT> operator- (const Line<D, NumericT> & l, const Vector<D, NumericT> & v);

	template <std::size_t D, typename NumericT = RealT>
	class LineSegment {
	public:
		typedef typename RealTypeTraits<NumericT>::RealT RealT;
		typedef Vector<D, NumericT> VectorT;

	protected:
		VectorT _start;
		VectorT _end;

	public:
		LineSegment () {}

		LineSegment (const VectorT & end) : _start(ZERO), _end(end)
		{
		}

		LineSegment (const VectorT & start, const VectorT & end) : _start(start), _end(end)
		{
		}

		LineSegment (const Line<D, NumericT> & line, const NumericT & start_time, const NumericT & end_time) : _start(line.point_at_time(start_time)), _end(line.point_at_time(end_time))
		{
		}

		VectorT point_at_time(const RealT & t) const {
			return _start + (offset() * t);
		}

		Line<D, NumericT> to_line () const {
			return Line<D, NumericT>(start(), offset().normalize());
		}

		/// Is this segment zero-length?
		bool is_degenerate () {
			return _start.equivalent(_end);
		}

		bool intersects_with (const AlignedBox<D, NumericT> & other, VectorT & at) const;
		bool intersects_with (const AlignedBox<D, NumericT> & other, NumericT & t1, NumericT & t2) const;
		bool intersects_with (const LineSegment<D, NumericT> & other, NumericT & this_time, NumericT & other_time) const;
		bool intersects_with (const LineSegment<D, NumericT> & other, LineSegment<D, NumericT> & overlap) const;

		const VectorT & start () const {
			return _start;
		}

		const VectorT & end () const {
			return _end;
		}

		VectorT & start () {
			return _start;
		}

		VectorT & end () {
			return _end;
		}

		VectorT center () {
			return (_start + _end) / 2.0;
		}

		VectorT offset () const {
			return _end - _start;
		}

		Vector<D> direction () const {
			return (_end - _start).normalize();
		}

		bool equivalent(const LineSegment & other) const {
			return Numerics::equivalent(_start, other._start) && Numerics::equivalent(_end, other._end);
		}

		bool clip(const AlignedBox<D, NumericT> & box, LineSegment & segment) const {
			NumericT t1, t2;

			if (intersects_with(box, t1, t2)) {
				segment = LineSegment(point_at_time(t1), point_at_time(t2));

				return true;
			}

			return false;
		}
	};
	
	template <typename NumericT>
	Vector<2, NumericT> normal(const LineSegment<2, NumericT> & line_segment)
	{
		return normal(line_segment.direction());
	}
	
	template <typename NumericT>
	bool line_intersection_test (const Line<2, NumericT> & lhs, const Line<2, NumericT> & rhs, NumericT & left_time, NumericT & right_time)
	{
		Vector<2, NumericT> t = lhs.direction();
		Vector<2, NumericT> o = rhs.direction();

		// http://local.wasp.uwa.edu.au/~pbourke/geometry/lineline2d/
		float denom = (o[Y] * t[X]) - (o[X] * t[Y]);

		// Quick return
		if (denom == (NumericT)0) return false;

		NumericT na = o[X] * (lhs.point()[Y] - rhs.point()[Y]) - o[Y] * (lhs.point()[X] - rhs.point()[X]);
		left_time = na / denom;

		NumericT nb = t[X] * (lhs.point()[Y] - rhs.point()[Y]) - t[Y] * (lhs.point()[X] - rhs.point()[X]);
		right_time = nb / denom;

		return true;
	}

	template <std::size_t D, typename NumericT>
	bool line_intersection_test (const Line<D, NumericT> & lhs, const Line<D, NumericT> & rhs, NumericT & left_time, NumericT & right_time)
	{
		Vector<2, NumericT>
			lhs_point = lhs.point(),
			lhs_dir = lhs.direction(),
			rhs_point = rhs.point(),
			rhs_dir = rhs.direction();

		Line<2, NumericT> lhs2d(lhs_point, lhs_dir), rhs2d(rhs_point, rhs_dir);
		if (line_intersection_test(lhs2d, rhs2d, left_time, right_time)) {
			// Collision occurred in 2-space, check in n-space
			Vector<D, NumericT> lhs_pt, rhs_pt;

			lhs_pt = lhs.point_at_time(left_time);
			rhs_pt = rhs.point_at_time(right_time);

			return lhs_pt.equivalent(rhs_pt);
		} else   {
			// No collision occurred.
			return false;
		}
	}

	template <std::size_t D, typename NumericT>
	bool Line<D, NumericT>::intersects_with (const Line<D, NumericT> & other, NumericT & this_time, NumericT & other_time) const
	{
		return line_intersection_test(*this, other, this_time, other_time);
	}

	template <typename NumericT>
	bool ray_slabs_intersection(NumericT start, NumericT dir, NumericT min, NumericT max, NumericT& tfirst, NumericT& tlast)
	{
		if (Numerics::equivalent(dir, 0))
			return (start < max && start > min);

		NumericT tmin = (min - start) / dir;
		NumericT tmax = (max - start) / dir;

		if (tmin > tmax) std::swap(tmin, tmax);

		if (tmax < tfirst || tmin > tlast)
			return false;

		if (tmin > tfirst) tfirst = tmin;

		if (tmax < tlast) tlast = tmax;

		return true;
	}

	template <std::size_t D, typename NumericT>
	bool Line<D, NumericT>::intersects_with(const AlignedBox<D, NumericT> &a, NumericT & t1, NumericT & t2) const {
		t1 = (NumericT)0;
		t2 = (NumericT)1;

		for (std::size_t i = 0; i < D; i += 1) {
			if (!ray_slabs_intersection(_point[i], _direction[i], a.min()[i], a.max()[i], t1, t2)) return false;
		}

		return true;
	}

	template <std::size_t D, typename NumericT>
	inline Line<D, NumericT> operator+ (const Line<D, NumericT> &l, const Vector<D, NumericT> &v) {
		return Line<D, NumericT>(l.point() + v, l.direction());
	}

	template <std::size_t D, typename NumericT>
	inline Line<D, NumericT> operator- (const Line<D, NumericT> &l, const Vector<D, NumericT> &v) {
		return Line<D, NumericT>(l.point() - v, l.direction());
	}

	template <std::size_t D, typename NumericT>
	bool LineSegment<D, NumericT>::intersects_with (const AlignedBox<D, NumericT> &other, VectorT & at) const
	{
		Vector<D, NumericT> d((end() - start()).normalize());
		Line<D, NumericT> l(start(), d);

		l.intersects_with(other, at);

		return (end() - at).normalize() == d;
	}

	template <std::size_t D, typename NumericT>
	bool LineSegment<D, NumericT>::intersects_with (const AlignedBox<D, NumericT> & other, NumericT & t1, NumericT & t2) const
	{
		Vector<D, NumericT> d(end() - start());
		Line<D, NumericT> l(start(), d);

		return l.intersects_with(other, t1, t2);
	}

	/// Intersection between two line segments in 1-space.
	/// See http://eli.thegreenplace.net/2008/08/15/intersection-of-1d-segments/ for more information.
	/// @returns true if segments overlapped, and the overlapping portion in <tt>overlap</tt>
	template <typename NumericT>
	bool segments_intersect(const Vector<2, NumericT> & s1, const Vector<2, NumericT> & s2, Vector<2, NumericT> & overlap)
	{
		if (s1[1] >= s2[0] && s2[1] >= s1[0]) {
			overlap[0] = std::max(s1[0], s2[0]);
			overlap[1] = std::min(s1[1], s2[1]);

			return true;
		} else   {
			return false;
		}
	}

	template <typename NumericT>
	bool line_intersection_test (const LineSegment<1, NumericT> & lhs, const LineSegment<1, NumericT> & rhs, LineSegment<1, NumericT> & overlap)
	{
		NumericT lmin, lmax, rmin, rmax;

		lmin = lhs.start()[X];
		lmax = lhs.end()[X];

		if (lmin > lmax) std::swap(lmin, lmax);

		rmin = rhs.start()[X];
		rmax = rhs.end()[X];

		if (rmin > rmax) std::swap(rmin, rmax);

		Vector<2, NumericT> o;
		if (segments_intersect(vector(lmin, lmax), vector(rmin, rmax), o)) {
			overlap = LineSegment<1, NumericT>(vector(o[0]), vector(o[1]));
			return true;
		} else   {
			return false;
		}
	}

	template <typename NumericT>
	bool line_intersection_test (const LineSegment<2, NumericT> & lhs, const LineSegment<2, NumericT> & rhs, NumericT & left_time, NumericT & right_time)
	{
		Vector<2, NumericT> t = lhs.offset();
		Vector<2, NumericT> o = rhs.offset();

		// http://local.wasp.uwa.edu.au/~pbourke/geometry/lineline2d/
		float denom = (o[Y] * t[X]) - (o[X] * t[Y]);

		// Quick return
		if (denom == 0.0f) return false;

		float na = o[X] * (lhs.start()[Y] - rhs.start()[Y]) - o[Y] * (lhs.start()[X] - rhs.start()[X]);
		left_time = na / denom;

		// Quick return
		if (left_time < 0.0 || left_time > 1.0) return false;

		float nb = t[X] * (lhs.start()[Y] - rhs.start()[Y]) - t[Y] * (lhs.start()[X] - rhs.start()[X]);
		right_time = nb / denom;

		// Quick return
		if (right_time < 0.0 || right_time > 1.0) return false;

		return true;
	}

	template <std::size_t D, typename NumericT>
	bool line_intersection_test (const LineSegment<D, NumericT> & lhs, const LineSegment<D, NumericT> & rhs, NumericT & left_time, NumericT & right_time)
	{
		Vector<2, NumericT> lhs_start, lhs_end, rhs_start, rhs_end;
		lhs_start = lhs.start();
		lhs_end = lhs.end();

		rhs_start = rhs.start();
		rhs_end = rhs.end();

		LineSegment<2, NumericT> lhs2d(lhs_start, lhs_end), rhs2d(rhs_start, rhs_end);
		if (line_intersection_test(lhs2d, rhs2d, left_time, right_time)) {
			// Collision occurred in 2-space, check in n-space
			Vector<D, NumericT> lhs_pt, rhs_pt;

			lhs_pt = lhs.point_at_time(left_time);
			rhs_pt = rhs.point_at_time(right_time);

			return lhs_pt.equivalent(rhs_pt);
		} else   {
			// No collision occurred.
			return false;
		}
	}

	template <std::size_t D, typename NumericT>
	bool LineSegment<D, NumericT>::intersects_with (const LineSegment<D, NumericT> & other, NumericT & left_time, NumericT & right_time) const
	{
		return line_intersection_test(*this, other, left_time, right_time);
	}

	typedef LineSegment<2> LineSegment2;
	typedef LineSegment<3> LineSegment3;

	extern template class LineSegment<2, RealT>;
	extern template class LineSegment<3, RealT>;
}