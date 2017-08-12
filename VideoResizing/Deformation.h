#ifndef DEFORMATION_H
#define DEFORMATION_H

#include <map>
#include <ctime>
#include "common.h"
#include "KeyFrame.h"
#include "ControlPoint.h"
#include "io.h"

typedef pair<int, int> Edge;

class Deformation {

private:
	string videoName;

	int frameNum;
	Size frameSize;
	vector<KeyFrame> &frames;
	vector<Point2f> staticPoints;

	int controlPointsNum;
	vector<ControlPoint> controlPoints;
	vector< vector<int> > frameControlPointIndex;
	vector< pair<int, int> > structureEdges;

	double deformedScaleX, deformedScaleY;
	Size deformedFrameSize;

	vector<Mat> deformedFrames;

	void DrawSubdiv( const Mat &, const Subdiv2D & );
	void DrawEdge( int, int, Mat &edgeImg );
	void DrawLocate( const Point2f &, const vector<BaryCoord> & );

	void CalcBaryCoordLambda( const Point2f &, const vector<Point2f> &, vector<double> & );
	void CalcBaryCoord1( map<string, int> &posToPointIndexMap, const Point2f &p, vector<BaryCoord> &baryCoord );
	void CalcBaryCoord2( Subdiv2D &subdiv, map<string, int> &posToPointIndexMap, int e0, const Point2f &p, vector<BaryCoord> &baryCoord );
	void CalcBaryCoord3( Subdiv2D &subdiv, map<string, int> &posToPointIndexMap, int e0, const Point2f &p, vector<BaryCoord> &baryCoord );
	int LocatePoint( Subdiv2D &subdiv, map<string, int> &posToPointIndexMap, const Point2f &p, vector<BaryCoord> &baryCoord );
	Point2f CalcPointByBaryCoord( const vector<BaryCoord> &, int );

	Point2f GetBoundPoint( int, int );
	void DelaunayDivide();
	void AddTemporalNeighbors();

	double CalcEnergyStructureL();
	double CalcEnergyStructureD();
	double CalcEnergyShapeL();
	double CalcEnergyShapeD();
	double CalcEnergyTemporal();

	void MinEnergyStructureL( vector<Point2f> &newControlPoints, double lambda );
	void MinEnergyStructureD( vector<Point2f> &newControlPoints, double lambda );
	void MinEnergyShapeL( vector<Point2f> &newControlPoints, double lambda );
	void MinEnergyShapeD( vector<Point2f> &newControlPoints, double lambda );
	void MinEnergyTemporal( vector<Point2f> &newControlPoints, double lambda );

	void CollinearConstraint( vector<Point2f> &newControlPoints );
	void UpdateControlPoints( const vector<Point2f> &newControlPoints );

public:

	enum {
		ORIGIN_POS = 0,
		DEFORMED_POS = 1,
		ORIGIN_POS_WITH_FRAME = 2,
		DEFORMED_POS_WITH_FRAME = 3
	} POS_TYPE;

	vector<Mat> deformedMap;

	Deformation( vector<KeyFrame> &, const string &_videoName );
	void BuildControlPoints();
	
	void InitDeformation( double, double );
	double CalcEnergy();
	void MinimizeEnergy();

	void CalcDeformedMap();
	void RenderFrame( const Mat &, const Mat &, Mat & );
	void RenderKeyFrames();
	void RenderFrames( const vector<Mat> &inputFrames, int shotSt, int shotEd );

};

#endif