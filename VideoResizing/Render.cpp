#include "Render.h"

Render::Render( const Deformation &deformation, 
				vector<ControlPoint> &_controlPoints, 
				vector<KeyFrame> &_keyFrames ) :controlPoints( _controlPoints ), keyFrames(_keyFrames) {
	frameSize = deformation.frameSize;
	deformedFrameSize = deformation.deformedFrameSize;
	frameNum = deformation.frameNum;
}

void Render::CalcW() {

	w = Mat::zeros( cpDeformedMat.cols, pixelMat.cols, CV_32FC1 );

	for ( int i = 0; i < cpDeformedMat.cols; i++ ) {

		Mat tmpMat = repeat( cpDeformedMat.col( i ), 1, pixelMat.cols );
		tmpMat = tmpMat - pixelMat;
		pow( tmpMat, 2, tmpMat );
		w.row( i ) = 1.0f / (tmpMat.row( 0 ) + tmpMat.row( 1 ) + VERY_SMALL);

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

void Render::CalcA( const Mat &cpDeformedCentroidMat, const vector<Mat> &cpDeformedHatArr) {

	Mat r1 = pixelMat - cpDeformedCentroidMat;
	Mat r2;
	vconcat( r1.row( 1 ), -r1.row( 0 ), r2 );

	A.clear();
	for ( size_t i = 0; i < cpDeformedHatArr.size(); i++ ) {
		Mat l1 = cpDeformedHatArr[i];
		Mat l2;
		vconcat( l1.row( 1 ), -l1.row( 0 ), l2 );

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

	Mat cpDeformedCentroidMat = CalcCentroid(cpDeformedMat);
	vector<Mat> cpDeformedHatArr;

	for ( int i = 0; i < cpDeformedMat.cols; i++ ) {
		Mat cpDeformedHat = repeat( cpDeformedMat.col( i ), 1, cpDeformedCentroidMat.cols ) - cpDeformedCentroidMat;
		cpDeformedHatArr.push_back( cpDeformedHat );
	}

	CalcA( cpDeformedCentroidMat, cpDeformedHatArr );

	Mat tmpMat = pixelMat - cpDeformedCentroidMat;
	pow( tmpMat, 2, tmpMat );
	sqrt( tmpMat.row( 0 ) + tmpMat.row( 1 ), normPixelCpCentroid );

}

Mat Render::CalcDeformedMap() {

	CalcRenderParams();

	Mat cpOriginCentroid = CalcCentroid( cpOriginMat );

	Mat fv2 = Mat::zeros( cpOriginCentroid.size(), CV_32FC1 );

	for ( int i = 0; i < cpOriginMat.cols; i++ ) {

		Mat cpOriginHat = repeat( cpOriginMat.col( i ), 1, cpOriginCentroid.cols ) - cpOriginCentroid;
		
		Mat con1;
		vconcat( A[i].a, A[i].c, con1 );
		con1 = cpOriginHat.mul( con1 );

		Mat con2;
		vconcat( A[i].b, A[i].d, con2 );
		con2 = cpOriginHat.mul( con2 );

		Mat tmpMat;
		vconcat( con1.row( 0 ) + con1.row( 1 ), con2.row( 0 ) + con2.row( 1 ), tmpMat );
		fv2 += tmpMat;

	}

	Mat tmpMat = fv2.mul( fv2 );
	Mat normFv2 = Mat::zeros( 1, fv2.cols, CV_32FC1 );
	sqrt( tmpMat.row( 0 ) + tmpMat.row( 1 ), normFv2 );

	tmpMat = normPixelCpCentroid.mul( 1 / normFv2 );
	tmpMat = repeat( tmpMat, fv2.rows, 1 );

	Mat fv = fv2.mul( tmpMat ) + cpOriginCentroid;

	Mat deformedMap = Mat( deformedFrameSize, CV_32SC2, Scalar(-1, -1) );
	for ( int y = 0; y < deformedFrameSize.height; y++ ) {
		for ( int x = 0; x < deformedFrameSize.width; x++ ) {

			int pointIndex = y * deformedFrameSize.width + x;
			Point originPoint( fv.at<float>( 0, pointIndex ), fv.at<float>( 1, pointIndex ) );

			if ( x < 2 ) {
				cout << Point( x, y ) << " " << originPoint << endl;
			}

			// RestrictInside( originPoint, frameSize );
			if ( !CheckOutside( originPoint, frameSize) ) 
			deformedMap.at<Point>( y, x ) = originPoint;

		}
	}

	return deformedMap;

}

void Render::CalcDeformedMaps() {

	int controlPointSt, controlPointEd;
	controlPointEd = 0;

	pixelMat = Mat( 2, deformedFrameSize.width * deformedFrameSize.height, CV_32FC1 );
	for ( int y = 0; y < deformedFrameSize.height; y++ ) {
		for ( int x = 0; x < deformedFrameSize.width; x++ ) {
			pixelMat.at<float>( 0, y * deformedFrameSize.width + x ) = x;
			pixelMat.at<float>( 1, y * deformedFrameSize.width + x ) = y;
		}
	}

	deformedMaps.clear();

	for ( int frameId = 0; frameId < frameNum; frameId++ ) {

		controlPointSt = controlPointEd;
		for ( ; controlPointEd < (int)controlPoints.size(); controlPointEd++ ) {
			if ( controlPoints[controlPointEd].frameId != frameId ) break;
		}

		vector<Point> originPos, pos;
		for ( int controlPointIndex = controlPointSt; controlPointIndex < controlPointEd; controlPointIndex++ ) {
			ControlPoint &controlPoint = controlPoints[controlPointIndex];
			if ( controlPoint.anchorType == ControlPoint::ANCHOR_BOUND || controlPoint.anchorType == ControlPoint::ANCHOR_CENTER ) {
				originPos.push_back( controlPoint.originPos );
				pos.push_back( controlPoint.pos );
			}
		}

		for ( int y = 0; y < deformedFrameSize.height; y += 5 ) {
			double deformedScale = (double)deformedFrameSize.height / frameSize.height;
			originPos.push_back( Point( 0, y / deformedScale ) );
			pos.push_back( Point( 0, y ) );
			originPos.push_back( Point( frameSize.width - 1, y / deformedScale ) );
			pos.push_back( Point( deformedFrameSize.width - 1, y ) );
		}

		for ( int x = 0; x < deformedFrameSize.width; x += 5 ) {
			double deformedScale = (double)deformedFrameSize.width / frameSize.width;
			originPos.push_back( Point( x / deformedScale, 0 ) );
			pos.push_back( Point( x, 0 ) );
			originPos.push_back( Point( x / deformedScale, frameSize.height - 1 ) );
			pos.push_back( Point( x, deformedFrameSize.height - 1 ) );
		}

		int cpNum = pos.size();
		cpOriginMat = Mat( 2, cpNum, CV_32FC1 );
		cpDeformedMat = Mat( 2, cpNum, CV_32FC1 );

		for ( int i = 0; i < cpNum; i++ ) {
			cpOriginMat.at<float>( 0, i ) = originPos[i].x;
			cpOriginMat.at<float>( 1, i ) = originPos[i].y;
			cpDeformedMat.at<float>( 0, i ) = pos[i].x;
			cpDeformedMat.at<float>( 1, i ) = pos[i].y;
		}

		CalcW();
		Mat deformedMap = CalcDeformedMap();
		deformedMaps.push_back( deformedMap );

	}

}


void Render::RenderFrame( const Mat &img, const Mat &deformedMap, Mat &deformedImg ) {

	Size frameSize = img.size();

	deformedImg = Mat::zeros( deformedFrameSize, CV_8UC3 );

	for ( int y = 0; y < deformedFrameSize.height; y++ ) {
		for ( int x = 0; x < deformedFrameSize.width; x++ ) {

			Point originPoint, deformedPoint;
			deformedPoint = Point( x, y );
			originPoint = deformedMap.at<Point>( deformedPoint );
			if ( !CheckOutside(originPoint, frameSize) )
			deformedImg.at<Vec3b>( deformedPoint ) = img.at<Vec3b>( originPoint );

		}
	}

}


void Render::RenderKeyFrames() {

	printf( "Render key frames.\n" );

	for ( int i = 0; i < frameNum; i++ ) {

		Mat deformedFrame, edgeImg;

		RenderFrame( keyFrames[i].img, deformedMaps[i], deformedFrame );

		imshow( "Saliency Map", keyFrames[i].saliencyMap );
		imshow( "Origin Frame", keyFrames[i].img );
		imshow( "Deformed Frame", deformedFrame );

		Mat stretchImg;
		resize( keyFrames[i].img, stretchImg, deformedFrameSize );
		imshow( "Stretch Img", stretchImg );
		// DrawEdge( i, DEFORMED_POS_WITH_FRAME, edgeImg );
		// WriteKeyFrameEdgeImg( frames[i].frameId, edgeImg, videoName );

		waitKey();

	}

}