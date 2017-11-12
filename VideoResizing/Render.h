#ifndef RENDER_H
#define RENDER_H

#include "common.h"
#include "KeyFrame.h"
#include "ControlPoint.h"
#include "Deformation.h"

struct TypeA {
	Mat a, b, c, d;
};

class Render {

private:
	vector<ControlPoint> &controlPoints;
	vector<KeyFrame> &keyFrames;

	Mat cpOriginMat, cpDeformedMat, pixelMat;
	Size frameSize, deformedFrameSize;
	int frameNum;

	vector<TypeA> A;
	Mat w, normPixelCpCentroid;

	vector<Mat> deformedMaps;

	void CalcW();
	Mat CalcCentroid( const Mat &cpMat );
	Mat CalcProduct( const Mat &lMat, const Mat &rMat );
	void CalcA( const Mat &cpOriginCentroidMat, const vector<Mat> &cpHatArr );
	void CalcRenderParams();
	Mat CalcDeformedMap();

public:
	Render( const Deformation &deformation, vector<ControlPoint> &controlPoints, vector<KeyFrame> &keyFrames );
	void CalcDeformedMaps();
	void RenderFrame( const Mat &img, const Mat &deformedMap, Mat &deformedImg );
	void RenderKeyFrames();

};

#endif