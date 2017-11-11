#include "Render.h"

Render::Render( const vector<ControlPoint> &_controlPoints, const Size &_frameSize, const Size &_deformedFrameSize ) {
	controlPoints = _controlPoints;
	frameSize = _frameSize;
	deformedFrameSize = _deformedFrameSize;

	cpOriginMat = Mat( 2, controlPoints.size(), CV_32FC1 );
	pixelMat = Mat( 2, frameSize.width * frameSize.height, CV_32FC1 );

	for ( size_t controlPointIndex = 0; controlPointIndex < controlPoints.size(); controlPointIndex++ ) {
		cpOriginMat.at<float>( 0, controlPointIndex ) = controlPoints[controlPointIndex].originPos.x;
		cpOriginMat.at<float>( 1, controlPointIndex ) = controlPoints[controlPointIndex].originPos.y;
		cpDeformedMat.at<float>( 0, controlPointIndex ) = controlPoints[controlPointIndex].pos.x;
		cpDeformedMat.at<float>( 1, controlPointIndex ) = controlPoints[controlPointIndex].pos.y;
	}

	for ( int y = 0; y < frameSize.height; y++ ) {
		for ( int x = 0; x < frameSize.width; x++ ) {
			pixelMat.at<float>( 0, y * frameSize.width + x ) = x;
			pixelMat.at<float>( 1, y * frameSize.width + x ) = y;
		}
	}

	CalcW();

}

void Render::CalcW() {

	w = Mat::zeros( cpOriginMat.cols, pixelMat.cols, CV_32FC1 );

	for ( int i = 0; i < cpOriginMat.cols; i++ ) {
		Mat tmpMat = repeat( cpOriginMat.col( i ), 1, pixelMat.cols );
		tmpMat = tmpMat - pixelMat;
		pow( tmpMat, 2, tmpMat );
		w.row( i ) = 1.0f / (tmpMat.row( 0 ) + tmpMat.row( 1 ));
	}

}

Mat Render::CalcCentroid(const Mat &cpMat) {

	Mat sumMat = Mat::zeros( 1, w.cols, CV_32FC1 );

	for ( int i = 0; i < w.rows; i++ ) {
		sumMat += w.row( i );
	}

	Mat tmpMat = repeat( sumMat, cpMat.rows, 1 );
	Mat cpCentroidMat = cpMat * w / tmpMat;

	return cpCentroidMat;

}

Mat Render::CalcProduct( const Mat &lMat, const Mat &rMat ) {

	Mat multiRes = lMat.mul( rMat );
	Mat sumRes = Mat::zeros( 1, multiRes.cols, CV_32FC1 );
	for ( int i = 0; i < multiRes.rows; i++ ) {
		sumRes += multiRes.row( i );
	}

	return sumRes;

}

void Render::CalcA( const Mat &cpOriginCentroidMat, const vector<Mat> &cpHatArr) {

	Mat r1 = pixelMat - cpOriginCentroidMat;
	Mat r2;
	vconcat( r1.row( 1 ), -r1.row( 0 ), r2 );

	A.clear();
	for ( size_t i = 0; i < cpHatArr.size(); i++ ) {
		Mat l1 = cpHatArr.at(i);
		Mat l2;
		vconcat( l1.row( 1 ), (l1.row( 0 )).mul( -1 ), l2 );

		Mat sumRes;
		TypeA AOne;

		sumRes = CalcProduct( l1, r1 );
		AOne.a = (w.row( i )).mul( sumRes );

		sumRes = CalcProduct( l1, r2 );
		AOne.b = (w.row( i )).mul( sumRes );

		sumRes = CalcProduct( l2, r1 );
		AOne.c = (w.row( i )).mul( sumRes );

		sumRes = CalcProduct( l2, r2 );
		AOne.d = (w.row( i )).mul( sumRes );

		A.push_back( AOne );

	}

}

void Render::CalcRenderParams() {

	Mat cpOriginCentroidMat = CalcCentroid(cpOriginMat);
	vector<Mat> cpHatArr;

	for ( int i = 0; i < cpOriginMat.cols; i++ ) {
		Mat tmpMat = repeat( cpOriginMat.col( i ), 1, cpOriginCentroidMat.cols ) - cpOriginCentroidMat;
		cpHatArr.push_back( tmpMat );
	}

	CalcA( cpOriginCentroidMat, cpHatArr );

	Mat tmpMat = pixelMat - cpOriginCentroidMat;
	pow( tmpMat, 2, tmpMat );

	Mat sumMat = Mat::zeros( 1, tmpMat.cols, CV_32FC1 );
	for ( int i = 0; i < tmpMat.rows; i++ ) {
		sumMat += tmpMat.row( i );
	}

	sqrt( sumMat, normPixelCpCentroid );

}

void Render::render() {

	Mat cpDeformedCentroid = CalcCentroid( cpDeformedMat );

}
