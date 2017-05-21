#include "ControlPoint.h"


ControlPoint::ControlPoint() {
	frameId = -1;
	spatialBound.clear();
	temporalNeighbors.clear();
}

ControlPoint::ControlPoint( int _frameId, const Point2f &_pos, int _anchorType, int _superpixelIndex, double _saliency ) {

	frameId = _frameId;
	originPos = _pos;
	pos = _pos;
	anchorType = _anchorType;
	superpixelIndex = _superpixelIndex;
	saliency = _saliency;

	spatialBound.clear();
	temporalNeighbors.clear();

}

void ControlPoint::AddSpatialBound( int neighborIndex ) {
	spatialBound.push_back( neighborIndex );
}

void ControlPoint::AddTemporalNeighbor( const vector<BaryCoord> &baryCoordinate ) {
	temporalNeighbors = baryCoordinate;
}

void ControlPoint::PrintSpatialBound() {
	for ( const auto &index : spatialBound ) printf( "%d ", index );
	printf( "\n" );
}

void ControlPoint::PrintTemporalNeighbors() {
	for ( const auto &index : temporalNeighbors ) printf( "%d ", index );
	printf( "\n" );
}