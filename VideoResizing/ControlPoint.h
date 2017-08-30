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
		ANCHOR_STATIC = 0x2,
		ANCHOR_STATIC_LEFT = 0x3,
		ANCHOR_STATIC_TOP = 0x4,
		ANCHOR_STATIC_RIGHT = 0x5,
		ANCHOR_STATIC_BOTTOM = 0x6,
	} ANCHOR_TYPE;

	int frameId;
	Point2f originPos, pos;
	int anchorType;
	int superpixelIndex;
	double saliency;
	Point2f flow;

	vector<int> boundNeighbors;
	vector<int> superpixelNeighbors;
	vector< BaryCoord > temporalNeighbors;

	ControlPoint();
	ControlPoint( int, const Point2f &, int, int, double );

	void AddBoundNeighbor( int neighborIndex );
	void AddSuperpixelNeighbor( int neighborIndex );
	void AddSpatialNeighbor( int neighborIndex );
	void AddTemporalNeighbor( const vector<BaryCoord> &baryCoordinate );

	void PrintSpatialBound();
	void PrintTemporalNeighbors();

};

#endif