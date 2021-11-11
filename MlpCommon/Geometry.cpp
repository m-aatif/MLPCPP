/**
	Geometry.cpp
	Purpose: Provide many functions involving rectangles geometry.

	@author G. Rao
	@version 1.0
*/
#include "Geometry.h"

using namespace std;

namespace geon
{
	const aRectangle aRectangle::nullRect(0, 0, 0, 0);

	/**
	aRectangle::aRectangle()
	Summary: basic aRectangle constructor. It creates a rectangle positioned at (0,0) whose dimensions are (10,10).
	Params: none
	Return: none
	*/
	aRectangle::aRectangle()
	{
		aPoint pts[] = {
			bpn::construct<aPoint>(0, 0),
			bpn::construct<aPoint>(10, 0),
			bpn::construct<aPoint>(10, 10),
			bpn::construct<aPoint>(0, 10)
		};
		bpn::set_points(m_poly, pts, pts + 4);
		remapDimensions();
		m_tag = 0;
	}

	/**
	aRectangle::aRectangle()
	Summary: aRectangle constructor. It creates a rectangle positioned at (x,y) whose dimensions are (width,height).
	Params: x		double		The value of left corner.
			y		double		The value of bottom corner.
			width	double		The width of the rectangle.
			height	double		The height of the rectangle.
	Return: none
	*/
	aRectangle::aRectangle(double x, double y, double width, double height)
	{
		aPoint pts[] = {
			bpn::construct<aPoint>(x, y),
			bpn::construct<aPoint>(x + width, y),
			bpn::construct<aPoint>(x + width, y + height),
			bpn::construct<aPoint>(x, y + height)
		};
		bpn::set_points(m_poly, pts, pts + 4);
		remapDimensions();
		m_tag = 0;
	}

	/**
	aRectangle::(aPoint ptStart, aPoint ptStop)
	Summary: aRectangle constructor. It creates a rectangle with bottom-left corner point <ptStart> and
			top-right corner <ptStop>.
	Params: ptStart		aPoint		The value of bottom-left corner.
			ptStop		aPoint		The value of top-right corner.
	Return: none
	*/
	aRectangle::aRectangle(aPoint ptStart, aPoint ptStop)
	{
		aPoint pts[] = {
			bpn::construct<aPoint>(ptStart.x(), ptStart.y()),
			bpn::construct<aPoint>(ptStop.x(), ptStart.y()),
			bpn::construct<aPoint>(ptStop.x(), ptStop.y()),
			bpn::construct<aPoint>(ptStart.x(), ptStop.y())
		};
		bpn::set_points(m_poly, pts, pts + 4);
		remapDimensions();
		m_tag = 0;
	}

	/**
	void aRectangle::inflate(double x, double y)
	Summary: It inflates a rectangle by x,y leaving the origin unchanged.
	Params: x		double		The increment of width value.
			y		double		The increment of heigh value.
	Return: none
	*/
	void aRectangle::inflate(double x, double y)
	{
		aPoint pts[] = {
			bpn::construct<aPoint>(left(), bottom()),
			bpn::construct<aPoint>(right() + x, bottom()),
			bpn::construct<aPoint>(right() + x, top() + y),
			bpn::construct<aPoint>(left(), top() + y)
		};
		bpn::set_points(m_poly, pts, pts + 4);
		remapDimensions();
	}

	/**
	void aRectangle::inflate(double l, double t, double r, double b)
	Summary: It inflates a rectangle.
	Params: l		double		The decrement of left value.
			t		double		The increment of top value.
			r		double		The increment of right value.
			b		double		The decrement of bottom value.
	Return: none
	*/
	void aRectangle::inflate(double l, double t, double r, double b)
	{
		aPoint pts[] = {
			bpn::construct<aPoint>(left() - l, bottom() - b),
			bpn::construct<aPoint>(right() + r, bottom() - b),
			bpn::construct<aPoint>(right() + r, top() + t),
			bpn::construct<aPoint>(left() - l, top() + t)
		};
		bpn::set_points(m_poly, pts, pts + 4);
		remapDimensions();
	}

	/**
	long double aRectangle::area()
	Summary: Gets the area of the rectangle.
	Params: none.
	Return: rectangle area.
	*/
	long double aRectangle::area()
	{
		return bpn::area(m_poly);
	}

	/**
	aRectangle aRectangle::operator + (aPoint pt)
	Summary: Offsets a rectangle by point <pt>.
	Params: pt		aPoint		The point to use as offset.
	Return: The offsetted rectangle.
	*/
	aRectangle aRectangle::operator + (aPoint pt)
	{
		bpn::convolve(m_poly, pt);
		remapDimensions();
		return *this;
	}

	/**
	aRectangle aRectangle::operator += (aPoint pt)
	Summary: Offsets this rectangle by point <pt>.
	Params: pt		aPoint		The point to use as offset.
	Return: The offsetted rectangle.
	*/
	aRectangle aRectangle::operator += (aPoint pt)
	{
		bpn::convolve(m_poly, pt);
		remapDimensions();
		return *this;
	}

	/**
	bool aRectangle::operator == (aRectangle other)
	Summary: Compares this rectangle with <other> rectangle.
	Params: other	aRectangle		The rectangle to compare.
	Return: TRUE	The rectangles match.
			TRUE	The rectangles mismatch.
	*/
	bool aRectangle::operator == (aRectangle other)
	{
		return (
			(left() == other.left()) &&
			(bottom() == other.bottom()) &&
			(right() == other.right()) &&
			(top() == other.top())
			);
	}

	void aRectangle::scaleUp(double amount)
	{
		bpn::scale_up(m_poly, amount);
		remapDimensions();
	}

	void aRectangle::scaleDown(double amount)
	{
		bpn::scale_down(m_poly, amount);
		remapDimensions();
	}

	bool aRectangle::intersect(aRectangle other, bool considerTouch)
	{
		bpn::rectangle_data<double> rect;
		bpn::extents(rect, other.m_poly);
		return bpn::intersect(m_rectData, rect, considerTouch);
	}
	bool aRectangle::intersectMod(aRectangle other, bool considerTouch)
	{
		/*
		bpn::rectangle_data<double> rect;
		bpn::extents(rect, other.m_poly);
		return bpn::intersect(m_rectData, rect, considerTouch);
		*/
		return (
			intersect(other.bottomLeft(), other.bottomRight()) ||
			intersect(other.topLeft(), other.topRight()) ||
			intersect(other.bottomLeft(), other.topLeft()) ||
			intersect(other.bottomRight(), other.topRight())
			);
	}

	bool aRectangle::intersect2(aPoint pt1, aPoint pt2, bool considerTouch)
	{
		return
			GeometryTools::intersectSegment(pt1, pt2, bottomLeft(), topLeft()) ||
			GeometryTools::intersectSegment(pt1, pt2, topLeft(), topRight()) ||
			GeometryTools::intersectSegment(pt1, pt2, topRight(), bottomRight()) ||
			GeometryTools::intersectSegment(pt1, pt2, bottomRight(), bottomLeft());
	}

	bool aRectangle::intersect(aPoint pt1, aPoint pt2, bool considerTouch)
	{
		double dx = pt2.x() - pt1.x();
		double dy = pt2.y() - pt1.y();
		double sbl = dy * (left() - pt1.x()) - dx * (bottom() - pt1.y());
		double sbr = dy * (right() - pt1.x()) - dx * (bottom() - pt1.y());
		double stl = dy * (left() - pt1.x()) - dx * (top() - pt1.y());
		double str = dy * (right() - pt1.x()) - dx * (top() - pt1.y());
		bool bCheck = isPositive(sbl);
		if (bCheck == isPositive(sbr))
			if (bCheck == isPositive(stl))
				if (bCheck == isPositive(str))
					return false;
		double xMin = min(pt1.x(), pt2.x());
		double xMax = max(pt1.x(), pt2.x());
		double yMin = min(pt1.y(), pt2.y());
		double yMax = max(pt1.y(), pt2.y());
		double xrMin = min(left(), right());
		double xrMax = max(left(), right());
		double yrMin = min(bottom(), top());
		double yrMax = max(bottom(), top());
		bool bCheck2(
			((xrMin <= xMin) && (xrMax >= xMin)) ||
			((xrMin <= xMax) && (xrMax >= xMax)) ||
			((xMin <= xrMin) && (xMax >= xrMin)) ||
			((xMin <= xrMax) && (xMax >= xrMax))
		);
		bool bCheck3(
			((yrMin <= yMin) && (yrMax >= yMin)) ||
			((yrMin <= yMax) && (yrMax >= yMax)) ||
			((yMin <= yrMin) && (yMax >= yrMin)) ||
			((yMin <= yrMax) && (yMax >= yrMax))
		);
		return (bCheck2 | bCheck3);
	}

	bool aRectangle::contains(aPoint pt, bool considerTouch)
	{
		//return bpn::contains(m_rectData, pt, considerTouch);
		if (considerTouch)
		{
			if (
				(pt.x() >= left()) && (pt.x() <= right()) &&
				(pt.y() >= bottom()) && (pt.y() <= top())
				)
				return true;
		}
		else
		{
			if (
				(pt.x() > left()) && (pt.x() < right()) &&
				(pt.y() > bottom()) && (pt.y() < top())
				)
				return true;
		}
		return false;
	}

	bool aRectangle::contains(double x, double y, bool considerTouch)
	{
		return bpn::contains(m_rectData, aPoint(x, y), considerTouch);
	}

	bool aRectangle::contains(aRectangle other, bool considerTouch)
	{
		//return bpn::contains(m_rectData, other.GetRectData(), considerTouch);
		if (considerTouch)
		{
			if (
				(other.left() >= left()) && (other.right() <= right()) &&
				(other.bottom() >= bottom()) && (other.top() <= top())
				)
				return true;
		}
		else
		{
			if (
				(other.left() > left()) && (other.right() < right()) &&
				(other.bottom() > bottom()) && (other.top() < top())
				)
				return true;
		}
		return false;
	}

	void aRectangle::remapDimensions()
	{
		bpn::extents(m_rectData, m_poly);
		bpn::orientation_2d ho, vo;
		ho = bpn::HORIZONTAL;
		vo = bpn::VERTICAL;
		m_horzIntervalData = m_rectData.get(ho);
		m_vertIntervalData = m_rectData.get(vo);
		m_bChanged = false;
	}

	double aRectangle::left()
	{
		if (m_bChanged)
			remapDimensions();
		return m_horzIntervalData.low();
	}

	double aRectangle::bottom()
	{
		if (m_bChanged)
			remapDimensions();
		return m_vertIntervalData.low();
	}

	double aRectangle::right()
	{
		if (m_bChanged)
			remapDimensions();
		return m_horzIntervalData.high();
	}

	double aRectangle::top()
	{
		if (m_bChanged)
			remapDimensions();
		return m_vertIntervalData.high();
	}

	double aRectangle::width()
	{
		if (m_bChanged)
			remapDimensions();
		return m_horzIntervalData.high() - m_horzIntervalData.low();
	}

	double aRectangle::height()
	{
		if (m_bChanged)
			remapDimensions();
		return m_vertIntervalData.high() - m_vertIntervalData.low();
	}

	aPoint aRectangle::bottomLeft()
	{
		if (m_bChanged)
			remapDimensions();
		return aPoint(m_horzIntervalData.low(), m_vertIntervalData.low());
	}

	aPoint aRectangle::bottomRight()
	{
		if (m_bChanged)
			remapDimensions();
		return aPoint(m_horzIntervalData.high(), m_vertIntervalData.low());
	}

	aPoint aRectangle::topLeft()
	{
		if (m_bChanged)
			remapDimensions();
		return aPoint(m_horzIntervalData.low(), m_vertIntervalData.high());
	}

	aPoint aRectangle::topRight()
	{
		if (m_bChanged)
			remapDimensions();
		return aPoint(m_horzIntervalData.high(), m_vertIntervalData.high());
	}

	aPoint aRectangle::center()
	{
		if (m_bChanged)
			remapDimensions();
		return aPoint(
			(m_horzIntervalData.high() + m_horzIntervalData.low()) / 2.0,
			(m_vertIntervalData.high() + m_vertIntervalData.low()) / 2.0
		);
	}

	bool aRectangle::joinableLeft(aRectangle other)
	{
		return (
			(left() == other.right()) &&
			(bottom() == other.bottom()) &&
			(top() == other.top())
			);
	}

	bool aRectangle::alignedLeft(aRectangle other, bool considerTouch)
	{
		bool bRet = (left() == other.left());
		if (considerTouch)
		{
			bRet &= (
				(bottom() == other.top()) ||
				(top() == other.bottom())
				);
		}
		return bRet;
	}

	bool aRectangle::joinLeft(aRectangle other)
	{
		if (joinableLeft(other))
		{
			aPoint pts[] = {
				bpn::construct<aPoint>(other.left(), other.bottom()),
				bpn::construct<aPoint>(other.left(), other.top()),
				bpn::construct<aPoint>(right(), top()),
				bpn::construct<aPoint>(right(), bottom())
			};
			bpn::set_points(m_poly, pts, pts + 4);
			remapDimensions();
			return true;
		}
		return false;
	}

	bool aRectangle::joinableTopLeft(aRectangle other)
	{
		return (
			(left() == other.right()) &&
			(top() == other.bottom())
			);
	}

	bool aRectangle::joinableTop(aRectangle other)
	{
		return (
			(left() == other.left()) &&
			(right() == other.right()) &&
			(top() == other.bottom())
			);
	}

	bool aRectangle::alignedTop(aRectangle other, bool considerTouch)
	{
		bool bRet = (top() == other.top());
		if (considerTouch)
		{
			bRet &= (
				(left() == other.right()) ||
				(right() == other.left())
				);
		}
		return bRet;
	}

	bool aRectangle::joinTop(aRectangle other)
	{
		if (joinableTop(other))
		{
			aPoint pts[] = {
				bpn::construct<aPoint>(left(), bottom()),
				bpn::construct<aPoint>(other.left(), other.top()),
				bpn::construct<aPoint>(other.right(), other.top()),
				bpn::construct<aPoint>(right(), bottom())
			};
			bpn::set_points(m_poly, pts, pts + 4);
			remapDimensions();
			return true;
		}
		return false;
	}

	bool aRectangle::joinableTopRight(aRectangle other)
	{
		return (
			(right() == other.left()) &&
			(top() == other.bottom())
			);
	}

	bool aRectangle::joinableRight(aRectangle other)
	{
		return (
			(bottom() == other.bottom()) &&
			(right() == other.left()) &&
			(top() == other.top())
			);
	}

	bool aRectangle::alignedRight(aRectangle other, bool considerTouch)
	{
		bool bRet = (right() == other.right());
		if (considerTouch)
		{
			bRet &= (
				(bottom() == other.top()) ||
				(top() == other.bottom())
				);
		}
		return bRet;
	}

	bool aRectangle::joinRight(aRectangle other)
	{
		if (joinableRight(other))
		{
			aPoint pts[] = {
				bpn::construct<aPoint>(left(), bottom()),
				bpn::construct<aPoint>(left(), top()),
				bpn::construct<aPoint>(other.right(), other.top()),
				bpn::construct<aPoint>(other.right(), other.bottom())
			};
			bpn::set_points(m_poly, pts, pts + 4);
			remapDimensions();
			return true;
		}
		return false;
	}

	bool aRectangle::joinableBottomRight(aRectangle other)
	{
		return (
			(bottom() == other.top()) &&
			(right() == other.left())
			);
	}

	bool aRectangle::joinableBottom(aRectangle other)
	{
		return (
			(left() == other.left()) &&
			(bottom() == other.top()) &&
			(right() == other.right())
			);
	}

	bool aRectangle::alignedBottom(aRectangle other, bool considerTouch)
	{
		bool bRet = (bottom() == other.bottom());
		if (considerTouch)
		{
			bRet &= (
				(left() == other.right()) ||
				(right() == other.left())
				);
		}
		return bRet;
	}

	bool aRectangle::joinBottom(aRectangle other)
	{
		if (joinableBottom(other))
		{
			aPoint pts[] = {
				bpn::construct<aPoint>(other.left(), other.bottom()),
				bpn::construct<aPoint>(left(), top()),
				bpn::construct<aPoint>(right(), top()),
				bpn::construct<aPoint>(other.right(), other.bottom())
			};
			bpn::set_points(m_poly, pts, pts + 4);
			remapDimensions();
			return true;
		}
		return false;
	}

	bool aRectangle::joinableBottomLeft(aRectangle other)
	{
		return (
			(left() == other.right()) &&
			(bottom() == other.top())
			);
	}

	bool aRectangle::joinable(aRectangle other)
	{
		if (joinableLeft(other))
			return true;
		if (joinableTop(other))
			return true;
		if (joinableRight(other))
			return true;
		if (joinableBottom(other))
			return true;
		return false;
	}

	bool aRectangle::join(aRectangle other)
	{
		if (joinLeft(other))
			return true;
		if (joinTop(other))
			return true;
		if (joinRight(other))
			return true;
		if (joinBottom(other))
			return true;
		return false;
	}

	bool aRectangle::dockableAtRight(aRectangle other)
	{
		return (right() == other.left());
	}

	bool aRectangle::dockableAtLeft(aRectangle other)
	{
		return (left() == other.right());
	}

	bool aRectangle::dockableAtBottom(aRectangle other)
	{
		return (bottom() == other.top());
	}

	bool aRectangle::dockableAtTop(aRectangle other)
	{
		return (top() == other.bottom());
	}

	double aRectangle::centerDistance(aRectangle other)
	{
		aPoint pt0 = center();
		aPoint pt1 = other.center();
		return hypot(pt0.x() - pt1.x(), pt0.y() - pt1.y());
	}

	void aRectangle::dump(string s)
	{
		//cout << s << ": (" << left() << "," << bottom() << "," << width() << "," << height() << ")" << endl;
		cout << left() << "," << bottom() << "," << width() << "," << height() << endl;
	}

	int GeometryTools::orientation(aPoint p, aPoint q, aPoint r)
	{
		double val = (q.y() - p.y()) * (r.x() - q.x()) - (q.x() - p.x()) * (r.y() - q.y());
		if (val > 0)
			// Clockwise orientation
			return 1;
		else if (val < 0)
			// Counterclockwise orientation
			return 2;
		else
			// Colinear orientation
			return 0;
	}

	int GeometryTools::onSegment(aPoint p, aPoint q, aPoint r)
	{
		return ((q.x() <= max(p.x(), r.x())) && (q.x() >= min(p.x(), r.x())) &&
			(q.y() <= max(p.y(), r.y())) && (q.y() >= min(p.y(), r.y())));
	}

	bool GeometryTools::intersectSegment(aPoint p1, aPoint q1, aPoint p2, aPoint q2)
	{
		// Find the 4 orientations required for
		// the general and special cases
		int o1 = orientation(p1, q1, p2);
		int o2 = orientation(p1, q1, q2);
		int o3 = orientation(p2, q2, p1);
		int o4 = orientation(p2, q2, q1);

		// General case
		if ((o1 != o2) && (o3 != o4))
			return true;

		// Special Cases

		// p1, q1 and p2 are colinear and p2 lies on segment p1q1
		if ((o1 == 0) && onSegment(p1, p2, q1))
			return true;

		// p1, q1 and q2 are colinear and q2 lies on segment p1q1
		if ((o2 == 0) && onSegment(p1, q2, q1))
			return true;

		// p2, q2 and p1 are colinear and p1 lies on segment p2q2
		if ((o3 == 0) && onSegment(p2, p1, q2))
			return true;

		// p2, q2 and q1 are colinear and q1 lies on segment p2q2
		if ((o4 == 0) && onSegment(p2, q1, q2))
			return true;

		// If none of the cases
		return false;
	}

	bool GeometryTools::belongsToLine(aPoint pt0, aPoint pt, aPoint pt1)
	{
		double x0, y0, m, y, dx;
		if (pt0.x() == pt1.x())
		{
			return (pt.x() == pt0.x());
		}
		else
		{
			x0 = pt0.x();
			y0 = pt0.y();
			dx = pt1.x() - x0;
			m = (pt1.y() - y0) / dx;

			y = y0 + (pt.x() - x0)*m;
			return (y == pt.y());
		}

		return false;
	}

	aPoint GeometryTools::scalePoint(aPoint pt, double minX, double minY, double res)
	{
		return aPoint(
			(pt.x() - minX) / res,
			(pt.y() - minY) / res
		);
	}

	aPoint GeometryTools::descalePoint(aPoint pt, double minX, double minY, double res)
	{
		return aPoint(
			res * pt.x() + minX,
			res * pt.y() + minY
		);
	}

	aRectangle GeometryTools::scaleRect(aRectangle rc, double minX, double minY, double res)
	{
		return aRectangle(
			aPoint(
			(rc.left() - minX) / res,
				(rc.bottom() - minY) / res
			),
			aPoint(
			(rc.right() - minX) / res,
				(rc.top() - minY) / res
			)
		);
	}
};

