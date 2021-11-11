#include "CoveragePlanner.h"
#include <boost/numeric/ublas/matrix.hpp>

using namespace std;

namespace afarcloud
{
	void CoveragePlanner::findAndJoinRectanglesLeft(vector<aRectangle> inRects, vector<aRectangle> &outRects)
	{
		aRectangle aBase, ar;
		size_t ix = 0;
		int newTag = 0;

		aBase = inRects[ix++];
		while (ix < inRects.size())
		{
			ar = inRects[ix++];
			if (!aBase.joinLeft(ar))
			{
				aBase.setTag(newTag++);
				outRects.push_back(aBase);
				aBase = ar;
			}
		}
		aBase.setTag(newTag);
		outRects.push_back(aBase);
	}

	void CoveragePlanner::findAndJoinRectanglesRight(vector<aRectangle> inRects, vector<aRectangle> &outRects)
	{
		aRectangle aBase, ar;
		size_t ix = 0;
		int newTag = 0;

		aBase = inRects[ix++];
		while (ix < inRects.size())
		{
			ar = inRects[ix++];
			if (!aBase.joinRight(ar))
			{
				aBase.setTag(newTag++);
				outRects.push_back(aBase);
				aBase = ar;
			}
		}
		aBase.setTag(newTag);
		outRects.push_back(aBase);
	}

	void CoveragePlanner::findAndJoinRectanglesBottom(rectangleMap inRectsMap, size_t xSize, size_t ySize, vector<aRectangle> &outRects)
	{
		aRectangle aBase, ar;
		bool set = false;
		rectangleMap::iterator it;
		int tag, newTag = 0;;

		for (size_t x = 0; x < xSize; x++)
			for (size_t y = 0; y < ySize; y++)
			{
				tag = (y << 16) + x;
				if (!set)
				{
					aBase = inRectsMap[tag];
					set = true;
				}
				else
				{
					it = inRectsMap.find(tag);
					if (it != inRectsMap.end())
					{
						ar = it->second;
						if (!aBase.joinBottom(ar))
						{
							aBase.setTag(newTag++);
							outRects.push_back(aBase);
							aBase = ar;
						}
					}
				}
			}

		aBase.setTag(newTag);
		outRects.push_back(aBase);
	}

	void CoveragePlanner::findAndJoinRectanglesTop(rectangleMap inRectsMap, size_t xSize, size_t ySize, vector<aRectangle> &outRects)
	{
		aRectangle aBase, ar;
		bool set = false;
		rectangleMap::iterator it;
		int tag, newTag = 0;

		for (size_t x = 0; x < xSize; x++)
			for (size_t y = 0; y < ySize; y++)
			{
				tag = (y << 16) + x;
				if (!set)
				{
					aBase = inRectsMap[tag];
					set = true;
				}
				else
				{
					it = inRectsMap.find(tag);
					if (it != inRectsMap.end())
					{
						ar = it->second;
						if (!aBase.joinTop(ar))
						{
							aBase.setTag(newTag++);
							outRects.push_back(aBase);
							aBase = ar;
						}
					}
				}
			}

		aBase.setTag(newTag);
		outRects.push_back(aBase);
	}

	void CoveragePlanner::adjustRectanglesLeft(vector<aRectangle> &outRects)
	{
		aRectangle aBase, ar;
		size_t ixBase, ixRun;
		bool joint;

		ixBase = 0;
		do
		{
			aBase = outRects[ixBase];
			joint = false;
			for (ixRun = ixBase + 1; ixRun < outRects.size(); ixRun++)
			{
				ar = outRects[ixRun];
				if (aBase.joinLeft(ar))
				{
					outRects[ixBase] = aBase;
					joint = true;
					break;
				}
			}
			if (joint)
			{
				outRects.erase(outRects.begin() + ixRun);
			}
			else
				ixBase++;
		} while (ixBase < outRects.size());
	}

	void CoveragePlanner::adjustRectanglesRight(vector<aRectangle> &outRects)
	{
		aRectangle aBase, ar;
		size_t ixBase, ixRun;
		bool joint;

		ixBase = 0;
		do
		{
			aBase = outRects[ixBase];
			joint = false;
			for (ixRun = ixBase + 1; ixRun < outRects.size(); ixRun++)
			{
				ar = outRects[ixRun];
				if (aBase.joinRight(ar))
				{
					outRects[ixBase] = aBase;
					joint = true;
					break;
				}
			}
			if (joint)
			{
				outRects.erase(outRects.begin() + ixRun);
			}
			else
				ixBase++;
		} while (ixBase < outRects.size());
	}

	void CoveragePlanner::adjustRectanglesBottom(vector<aRectangle> &outRects)
	{
		aRectangle aBase, ar;
		size_t ixBase, ixRun;
		bool joint;

		ixBase = 0;
		do
		{
			aBase = outRects[ixBase];
			joint = false;
			for (ixRun = ixBase + 1; ixRun < outRects.size(); ixRun++)
			{
				ar = outRects[ixRun];
				if (aBase.joinBottom(ar))
				{
					outRects[ixBase] = aBase;
					joint = true;
					break;
				}
			}
			if (joint)
			{
				outRects.erase(outRects.begin() + ixRun);
			}
			else
				ixBase++;
		} while (ixBase < outRects.size());
	}

	void CoveragePlanner::adjustRectanglesTop(vector<aRectangle> &outRects)
	{
		aRectangle aBase, ar;
		size_t ixBase, ixRun;
		bool joint;

		ixBase = 0;
		do
		{
			aBase = outRects[ixBase];
			joint = false;
			for (ixRun = ixBase + 1; ixRun < outRects.size(); ixRun++)
			{
				ar = outRects[ixRun];
				if (aBase.joinTop(ar))
				{
					outRects[ixBase] = aBase;
					joint = true;
					break;
				}
			}
			if (joint)
			{
				outRects.erase(outRects.begin() + ixRun);
			}
			else
				ixBase++;
		} while (ixBase < outRects.size());
	}

	/**
	void CoveragePlanner::aggregateRectanglesA()
	Summary: Aggregates the allowed cell horizontally and vertically, in macrocells.
	Then choose the aggreagation that generate as less macrocells as possible.
	Params: none.
	Return:	path		pointVec		The path obtained.
	*/
	void CoveragePlanner::aggregateRectanglesA()
	{
		/**
		Aggregate rectangles
		*/

		switch (m_direction)
		{
		case NE_DIRECTION:
			findAndJoinRectanglesRight(m_basicCells, m_compoundCellsH);
			findAndJoinRectanglesTop(m_basicCellRectangleMap, m_xCoords.size(), m_yCoords.size(), m_compoundCellsV);
			adjustRectanglesTop(m_compoundCellsH);
			adjustRectanglesRight(m_compoundCellsV);
			break;
		case SE_DIRECTION:
			findAndJoinRectanglesRight(m_basicCells, m_compoundCellsH);
			findAndJoinRectanglesBottom(m_basicCellRectangleMap, m_xCoords.size(), m_yCoords.size(), m_compoundCellsV);
			adjustRectanglesBottom(m_compoundCellsH);
			adjustRectanglesRight(m_compoundCellsV);
			break;
		case NW_DIRECTION:
			findAndJoinRectanglesLeft(m_basicCells, m_compoundCellsH);
			findAndJoinRectanglesTop(m_basicCellRectangleMap, m_xCoords.size(), m_yCoords.size(), m_compoundCellsV);
			adjustRectanglesTop(m_compoundCellsH);
			adjustRectanglesLeft(m_compoundCellsV);
			break;
		case SW_DIRECTION:
			findAndJoinRectanglesLeft(m_basicCells, m_compoundCellsH);
			findAndJoinRectanglesBottom(m_basicCellRectangleMap, m_xCoords.size(), m_yCoords.size(), m_compoundCellsV);
			adjustRectanglesBottom(m_compoundCellsH);
			adjustRectanglesLeft(m_compoundCellsV);
			break;
		}

		/**
		Choose the better representation
		*/
		aRectangle rc;
		vector<aRectangle>::iterator ito;

		int tag = 0;
		if (m_compoundCellsH.size() < m_compoundCellsV.size())
		{
			m_expansion = HORZ_EXPANSION;
			ito = m_compoundCellsH.begin();
			while (ito != m_compoundCellsH.end())
			{
				rc = *ito;
				rc.setTag(tag++);
				m_compoundCells.push_back(rc);
				ito++;
			}
		}
		else
		{
			m_expansion = VERT_EXPANSION;
			ito = m_compoundCellsV.begin();
			while (ito != m_compoundCellsV.end())
			{
				rc = *ito;
				rc.setTag(tag++);
				m_compoundCells.push_back(rc);
				ito++;
			}
		}
		for (size_t i = 0; i < m_compoundCells.size(); i++)
		{
			m_compoundCells[i].setTag(i);
		}

		for (size_t i = 0; i < m_compoundCellsV.size(); i++)
		{
			m_compoundCellsV[i].setTag(i);
		}

		for (size_t i = 0; i < m_compoundCellsH.size(); i++)
		{
			m_compoundCellsH[i].setTag(i);
		}
	}

}
