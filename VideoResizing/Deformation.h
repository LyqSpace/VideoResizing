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

	int controlPointsNum, freeEleNum;
	vector<int> freeEleMap;
	vector<ControlPoint> controlPoints;
	vector<Mat> controlPointsMap;
	vector< vector<int> > frameControlPointIndex;
	vector<int> centerControlPointIndex;
	vector<int> temporalControlPointIndex;
	vector< vector<int> > spatialEdges;

	double deformedScaleX, deformedScaleY;
	Size deformedFrameSize;

	vector<Mat> deformedFrames;

	void BuildControlPoints();
	void AddTemporalNeighbors();
	void AddSpatialNeighbors();

	void DrawSubdiv( const Mat &, const Subdiv2D & );
	void DrawEdge( int, int, Mat &edgeImg );
	void DrawLocate( const Point2f &, const vector<BaryCoord> & );

	void CalcBaryCoordLambda( const Point2f &, vector<Point2f> &, vector<double> & );
	void CalcBaryCoord1( const Mat &cpMap, const Point2f &p, vector<BaryCoord> &baryCoord );
	void CalcBaryCoord2( Subdiv2D &subdiv, const Mat &cpMap, int e0, const Point2f &p, vector<BaryCoord> &baryCoord );
	void CalcBaryCoord3( Subdiv2D &subdiv, const Mat &cpMap, int e0, const Point2f &p, vector<BaryCoord> &baryCoord );
	int LocatePoint( Subdiv2D &subdiv, const Mat &cpMap, const Point2f &p, vector<BaryCoord> &baryCoord );
	Point2f CalcPointByBaryCoord( const vector<BaryCoord> &, int );

	Point2f GetBoundPoint( int, int );

	double CalcSaliencyEnergy();
	double CalcObjectEnergy();
	double CalcStructureEnergy();
	double CalcTemporalEnergy();

	void AddSaliencyConstraints( Mat &coefMat, Mat &constVec );
	void AddObjectConstraints( Mat &coefMat, Mat &constVec );
	void AddStructureConstraints( Mat &coefMat, Mat &constVec );
	void AddTemporalConstraints( Mat &coefMat, Mat &constVec );
	void OptimizeEnergyFunction();

	void CollinearConstraints();
	void UpdateControlPoints();

public:

	enum {
		ORIGIN_POS = 0,
		DEFORMED_POS = 1,
		ORIGIN_POS_WITH_FRAME = 2,
		DEFORMED_POS_WITH_FRAME = 3
	} POS_TYPE;

	vector<Mat> deformedMap;

	Deformation( vector<KeyFrame> &, const string &_videoName );
	
	void InitDeformation( double, double );
	double CalcEnergy();
	void MinimizeEnergy();

	void CalcDeformedMap();
	void RenderFrame( const Mat &, const Mat &, Mat & );
	void RenderKeyFrames();
	void RenderFrames( const vector<Mat> &inputFrames, int shotSt, int shotEd );

};

#endif