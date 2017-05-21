#ifndef CONTROLPOINT_H
#define CONTROLPOINT_H

#include <utility>
#include "common.h"

typedef pair<double, int > BaryCoord;

class ControlPoint {
private:

public:

	enum {
		ANCHOR_CENTER = 0x0,
		ANCHOR_BOUND = 0x1,
		ANCHOR_TOP_LEFT = 0x20,
		ANCHOR_TOP_RIGHT = 0x21,
		ANCHOR_BOTTOM_LEFT = 0x22,
		ANCHOR_BOTTOM_RIGHT = 0x23
	} ANCHOR_TYPE;

	int frameId;
	Point2f originPos, pos;
	int anchorType;
	int superpixelIndex;
	double saliency;
	Point2f flow;

	vector<int> spatialBound;
	vector< BaryCoord > temporalNeighbors;

	ControlPoint();
	ControlPoint( int, const Point2f &, int, int, double );
	void AddSpatialBound( int );
	void AddTemporalNeighbor( const vector<BaryCoord> & );

	void PrintSpatialBound();
	void PrintTemporalNeighbors();

};

#endif