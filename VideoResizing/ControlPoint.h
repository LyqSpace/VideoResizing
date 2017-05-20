#ifndef CONTROLPOINT_H
#define CONTROLPOINT_H

#include <utility>
#include "common.h"

typedef pair<double, int > BaryCoord;

class ControlPoint {
private:

public:

	enum {
		ANCHOR_NONE = 0,
		ANCHOR_STATIC = 1,
		ANCHOR_TOP_LEFT = 2,
		ANCHOR_TOP_RIGHT = 3,
		ANCHOR_BOTTOM_LEFT = 4,
		ANCHOR_BOTTOM_RIGHT = 5
	} ANCHOR_TYPE;

	int frameId;
	Point2f originPos, pos;
	int anchorType;
	int superpixelIndex;
	double saliency;

	vector<int> spatialNeighbors;
	vector< BaryCoord > temporalNeighbors;

	ControlPoint();
	ControlPoint( int, const Point2f &, int, int, double );
	void AddSpatialNeighbor( int );
	void AddTemporalNeighbor( const vector<BaryCoord> & );

	void PrintSpatialNeighbors();
	void PrintTemporalNeighbors();

};

#endif