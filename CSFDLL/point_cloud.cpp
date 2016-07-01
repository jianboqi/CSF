#include "point_cloud.h"


void csf::PointCloud::computeBoundingBox(Point& bbMin, Point& bbMax)
{
	if (empty())
	{
		bbMin = bbMax = Point();
		return;
	}

	bbMin = bbMax = at(0);
	for (size_t i = 1; i < size(); i++)
	{
		const csf::Point& P = at(i);
		for (int d = 0; d < 3; ++d)
		{
			if (P.u[d] < bbMin.u[d])
			{
				bbMin.u[d] = P.u[d];
			}
			else if (P.u[d] > bbMax.u[d])
			{
				bbMax.u[d] = P.u[d];
			}
		}
	}
}