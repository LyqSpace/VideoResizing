#ifndef KEYFRAME_H
#define KEYFRAME_H

#include <cmath>
#include "common.h"
#include "slic.h"

class KeyFrame {
private:
	Mat imgWithContours, spatialContrastMap;
	Mat pixelLabel, paletteMap, paletteDist;

	vector<Vec3f> palette;
	vector< vector<int> > superpixelColorHist;
	vector<int> superpixelCard;
	vector<Point> superpixelCenter;
	vector<double> superpixelSpatialContrast;

	double CalcColorHistDiff( int, int );
	double CalcSpatialDiff( int, int );
	
public:
	Mat img, CIELabImg, grayImg, flowMap;
	int cols, rows, superPixelNum;
	Size size;

	KeyFrame( const Mat & );
	void SegSuperpixel();
	void QuantizeColorSpace( const vector<Vec3f> &, const Mat & );
	void CalcSuperpixelColorHist();
	void CalcSpatialContrast();
	void CalcTemporalContrast();
};

#endif