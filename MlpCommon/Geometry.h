#pragma once

#include <stdint.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/adapted/boost_polygon.hpp>

namespace bpn = boost::polygon;
namespace bgn = boost::geometry;
using namespace boost::polygon::operators;

typedef bpn::polygon_data<double> aPolygon;
typedef bpn::polygon_traits<aPolygon>::point_type aPoint;

/**
geon: A name space which stands for geo[metry]n[amespace].
*/
namespace geon
{
	/**
	aRectangle: A class that implements many useful features about rectangles.
	*/
	class aRectangle
	{
	public:
		static const aRectangle nullRect;

		aRectangle();
		aRectangle(double x, double y, double width, double height);
		aRectangle(aPoint ptStart, aPoint ptStop);

		long double area();
		bool contains(aPoint pt, bool considerTouch = true);
		bool contains(double x, double y, bool considerTouch = true);
		bool contains(aRectangle other, bool considerTouch = true);
		aRectangle operator + (aPoint pt);
		aRectangle operator += (aPoint pt);
		bool operator == (aRectangle other);
		void scaleUp(double amount);
		void scaleDown(double amount);
		bool intersect(aRectangle other, bool considerTouch = true);
		bool intersectMod(aRectangle other, bool considerTouch = true);
		bool intersect(aPoint pt1, aPoint pt2, bool considerTouch = true);
		bool intersect2(aPoint pt1, aPoint pt2, bool considerTouch = true);
		inline bool isPositive(double num) { return num >= 0.0; }

		void inflate(double x, double y);
		void inflate(double l, double t, double r, double b);

		double centerDistance(aRectangle other);
		bool joinableLeft(aRectangle other);
		bool joinableTopLeft(aRectangle other);
		bool joinableTop(aRectangle other);
		bool joinableTopRight(aRectangle other);
		bool joinableRight(aRectangle other);
		bool joinableBottomRight(aRectangle other);
		bool joinableBottom(aRectangle other);
		bool joinableBottomLeft(aRectangle other);
		
		bool joinable(aRectangle other);

		bool joinLeft(aRectangle other);
		bool joinTop(aRectangle other);
		bool joinRight(aRectangle other);
		bool joinBottom(aRectangle other);
		
		bool join(aRectangle other);

		bool alignedLeft(aRectangle other, bool considerTouch = true);
		bool alignedRight(aRectangle other, bool considerTouch = true);
		bool alignedTop(aRectangle other, bool considerTouch = true);
		bool alignedBottom(aRectangle other, bool considerTouch = true);

		bool dockableAtRight(aRectangle other);
		bool dockableAtLeft(aRectangle other);
		bool dockableAtBottom(aRectangle other);
		bool dockableAtTop(aRectangle other);

		double left();
		double bottom();
		double right();
		double top();
		double width();
		double height();
		aPoint bottomLeft();
		aPoint bottomRight();
		aPoint topLeft();
		aPoint topRight();
		aPoint center();

		inline bpn::rectangle_data<double> GetRectData() const { return m_rectData; }
		void dump(std::string s);

		inline void setTag(int tag) { m_tag = tag; }
		inline int getTag() const { return m_tag; }
	private:
		aPolygon m_poly;
		bpn::rectangle_data<double> m_rectData;
		bpn::interval_data<double> m_horzIntervalData;
		bpn::interval_data<double> m_vertIntervalData;
		bool m_bChanged;

		void remapDimensions();

		int m_tag;	///	Useful to identify the rectangle inside a vector or a map.
	};

	typedef std::vector<aPoint> pointVec;
	typedef std::vector<aRectangle> rectangleVec;

	/**
	Geometrytools: A static class that implements some useful features.
	*/
	class GeometryTools
	{
	public:
		static int orientation(aPoint p, aPoint q, aPoint r);
		static int onSegment(aPoint p, aPoint q, aPoint r);
		static bool intersectSegment(aPoint p1, aPoint q1, aPoint p2, aPoint q2);
		static bool belongsToLine(aPoint pt0, aPoint pt, aPoint pt1);
		static aPoint scalePoint(aPoint pt, double minX, double minY, double res);
		static aPoint descalePoint(aPoint pt, double minX, double minY, double res);
		static aRectangle scaleRect(aRectangle rc, double minX, double minY, double res);
	};
};

