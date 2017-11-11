#ifndef RENDER_H
#define RENDER_H

#include "common.h"
#include "KeyFrame.h"
#include "ControlPoint.h"

struct TypeA {
	Mat a, b, c, d;
};

class Render {

private:
	vector<ControlPoint> controlPoints;
	Mat cpOriginMat, cpDeformedMat, pixelMat;
	Size frameSize, deformedFrameSize;

	vector<TypeA> A;
	Mat w, normPixelCpCentroid;

	void CalcW();
	Mat CalcCentroid( const Mat &cpMat );
	Mat CalcProduct( const Mat &lMat, const Mat &rMat );
	void CalcA( const Mat &cpOriginCentroidMat, const vector<Mat> &cpHatArr );

public:
	Render( const vector<ControlPoint> &_controlPoints, const Size &_frameSize, const Size &_deformedFrameSize );
	void CalcRenderParams();
	void render();
};

#endif