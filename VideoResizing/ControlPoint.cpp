#include "ControlPoint.h"


ControlPoint::ControlPoint() {
	frameId = -1;
	spatialNeighbors.clear();
	temporalNeighbors.clear();
}

ControlPoint::ControlPoint( int _frameId, const Point2f &_pos, int _anchorType ) {

	frameId = _frameId;
	originPos = _pos;
	pos = _pos;
	anchorType = _anchorType;

	spatialNeighbors.clear();
	temporalNeighbors.clear();

}

void ControlPoint::AddSpatialNeighbor( int neighborIndex ) {
	spatialNeighbors.push_back( neighborIndex );
}

void ControlPoint::AddTemporalNeighbor( const vector<BaryCoord> &baryCoordinate ) {
	temporalNeighbors = baryCoordinate;
}

void ControlPoint::PrintSpatialNeighbors() {
	for ( const auto &index : spatialNeighbors ) printf( "%d ", index );
	printf( "\n" );
}

void ControlPoint::PrintTemporalNeighbors() {
	for ( const auto &index : temporalNeighbors ) printf( "%d ", index );
	printf( "\n" );
}