#ifndef KEYFRAME_H
#define KEYFRAME_H

#include <cmath>
#include <queue>
#include "common.h"
#include "slic.h"

class KeyFrame {

private:
	Mat imgWithContours, spatialContrastMap, temporalContrastMap;
	Mat paletteMap, paletteDist;

	vector<Vec3f> palette;
	vector< vector<int> > superpixelColorHist;
	vector<int> superpixelCard;
	
	vector<double> superpixelSpatialContrast, superpixelTemporalContrast;

	double CalcColorHistDiff( int, int );
	double CalcSpatialDiff( int, int );
	
public:

	enum {
		BOUND_NONE = 0,
		BOUND_LEFT = 1,
		BOUND_TOP = 2,
		BOUND_RIGHT = 3,
		BOUND_BOTTOM = 4
	} BOUND_LABEL;

	Mat img, CIELabImg, grayImg;
	Mat pixelLabel;
	Mat forwardFlowMap, backwardFlowMap, forwardLocalMotionMap, backwardLocalMotionMap;

	vector<int> superpixelBoundLabel;

	int verticalEdgesNum, horizontalEdgesNum;
	Mat verticalEdgesLabel, horizontalEdgesLabel;

	Mat saliencyMap;
	vector<double> superpixelSaliency;

	int frameId, cols, rows, superpixelNum;
	Size size;
	Point2f forwardGlobalMotion, backwardGlobalMotion;
	bool opFlag, edFlag;

	vector<Point> superpixelCenter;

	KeyFrame( const Mat &, int );
	void DrawImgWithContours();
	void SegSuperpixel();
	void MarkBoundLabel();
	void QuantizeColorSpace( const vector<Vec3f> &, const Mat & );
	void CalcSuperpixelColorHist();

	void SegVerticalEdges();
	void SegHorizontalEdges();

	void CalcSpatialContrast();
	void CalcTemporalContrast();
	void CalcSaliencyMap();
	void SumSuperpixelSaliency();

	void FreeMemory();

};

#endif