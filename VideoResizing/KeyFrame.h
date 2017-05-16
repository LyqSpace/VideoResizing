#ifndef KEYFRAME_H
#define KEYFRAME_H

#include <cmath>
#include "common.h"
#include "slic.h"

class KeyFrame {
private:
	Mat imgWithContours, spatialContrastMap, temporalContrastMap;
	Mat pixelLabel, paletteMap, paletteDist;

	vector<Vec3f> palette;
	vector< vector<int> > superpixelColorHist;
	vector<int> superpixelCard;
	vector<Point> superpixelCenter;
	vector<double> superpixelSpatialContrast, superpixelTemporalContrast, superpixelSaliency;

	double CalcColorHistDiff( int, int );
	double CalcSpatialDiff( int, int );
	
public:
	Mat img, CIELabImg, grayImg;
	Mat forwardFlowMap, backwardFlowMap, forwardLocalMotionMap, backwardLocalMotionMap;
	Mat saliencyMap;
	int frameId, cols, rows, superPixelNum;
	Size size;
	Point2f forwardGlobalMotion, backwardGlobalMotion;
	bool opFlag, edFlag;

	KeyFrame( const Mat &, int );
	void SegSuperpixel();
	void QuantizeColorSpace( const vector<Vec3f> &, const Mat & );
	void CalcSuperpixelColorHist();
	void CalcSpatialContrast();
	void CalcTemporalContrast();
	void CalcSaliencyMap();
};

#endif