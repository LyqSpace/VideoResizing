#ifndef KEYFRAME_H
#define KEYFRAME_H

#include <cmath>
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
	Mat img, CIELabImg, grayImg;
	Mat pixelLabel;
	Mat forwardFlowMap, backwardFlowMap, forwardLocalMotionMap, backwardLocalMotionMap;

	Mat saliencyMap;
	vector<double> superpixelSaliency;

	int frameId, cols, rows, superpixelNum;
	Size size;
	Point2f forwardGlobalMotion, backwardGlobalMotion;
	bool opFlag, edFlag;

	vector<Point> superpixelCenter;

	KeyFrame( const Mat &, int );
	void SegSuperpixel();
	void QuantizeColorSpace( const vector<Vec3f> &, const Mat & );
	void CalcSuperpixelColorHist();
	void CalcSpatialContrast();
	void CalcTemporalContrast();
	void CalcSaliencyMap();
	void SumSuperpixelSaliency();

	void FreeMemory();

};

#endif