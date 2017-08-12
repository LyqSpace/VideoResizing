#include "saliency.h"


void DrawOpticalFlow( const Mat &flowMat, const Mat &frame, string windowName ) {

	Mat img = frame.clone();
	int step = 10;

	for ( int y = 0; y < flowMat.rows; y += step ) {
		for ( int x = 0; x < flowMat.cols; x += step ) {
			Point2f flow = flowMat.at<Point2f>( y, x );
			line( img, Point( x, y ), Point( cvRound( x + flow.x ), cvRound( y + flow.y ) ), Scalar( 0, 255, 0 ) );
			circle( img, Point( x, y ), 2, Scalar( 0, 0, 255 ) );
		}
	}
	imshow( windowName, img );
	waitKey( 1 );

}

void CalcMotion( const Mat &flowMap, Mat &localMotionMap, Point2f &globalMotion ) {

	globalMotion = Point2f( 0, 0 );
	for ( int y = 0; y < flowMap.rows; y++ ) {
		for ( int x = 0; x < flowMap.cols; x++ ) {
			Point2f flow = flowMap.at<Point2f>( y, x );
			globalMotion.x += flow.x;
			globalMotion.y += flow.y;
		}
	}

	globalMotion.x /= (flowMap.rows * flowMap.cols);
	globalMotion.y /= (flowMap.rows * flowMap.cols);

	localMotionMap = Mat::zeros( flowMap.size(), CV_32FC2 );
	for ( int y = 0; y < localMotionMap.rows; y++ ) {
		for ( int x = 0; x < localMotionMap.cols; x++ ) {
			Point2f flow = flowMap.at<Point2f>( y, x );
			Point2f localMotion = Point2f( flow.x - globalMotion.x, flow.y - globalMotion.y );
			localMotionMap.at<Point2f>( y, x ) = localMotion;
		}
	}
}

void CalcSaliencyMap( vector<KeyFrame> &frames ) {

	printf( "Calculate key frames saliency map.\n" );

	printf( "\tCalculate optical flow.\n" );

	for ( size_t i = 1; i < frames.size(); i++ ) {

		Mat localMotionMap;
		Point2f globalMotion;

		calcOpticalFlowFarneback( frames[i - 1].grayImg, frames[i].grayImg, frames[i - 1].forwardFlowMap, 0.5, 3, 15, 3, 5, 1.2, 0 );
		CalcMotion( frames[i - 1].forwardFlowMap, localMotionMap, globalMotion );

		frames[i - 1].forwardLocalMotionMap = localMotionMap.clone();
		frames[i - 1].forwardGlobalMotion = globalMotion;

		calcOpticalFlowFarneback( frames[i].grayImg, frames[i - 1].grayImg, frames[i].backwardFlowMap, 0.5, 3, 15, 3, 5, 1.2, 0 );
		CalcMotion( frames[i].backwardFlowMap, localMotionMap, globalMotion );

		frames[i].backwardLocalMotionMap = localMotionMap;
		frames[i].backwardGlobalMotion = globalMotion;

		// DrawOpticalFlow( frames[i - 1].forwardFlowMap, frames[i - 1].img, "Flow Ma/*p" );
		// DrawOpticalFlow( frames[i - 1].forwardLocalMotionMap, frames[i - 1].img, "Local Motion M*/ap" );

	}

	printf( "\tCalculate temporal contrast.\n" );

	for ( auto &frame : frames ) {
		frame.CalcTemporalContrast();
	}

	printf( "\tCalculate spatial contrast.\n" );

	for ( auto &frame : frames ) {
		frame.CalcSpatialContrast();
	}

	for ( auto &frame : frames ) {
		frame.CalcSaliencyMap();
	}

}

void SmoothSaliencyMap( vector<KeyFrame> &frames ) {

	printf( "Smooth key frames saliency map.\n" );

	int frameSpan = SALIENCY_SMOOTH_SPAN >> 1;
	Size size = frames[0].saliencyMap.size();
	vector<double> smoothWeight;
	vector<Mat> smoothedSaliencyMapVec;

	for ( int i = frameSpan; i < SALIENCY_SMOOTH_SPAN; i++ ) {
		smoothWeight.push_back( exp( -(float)sqr( i - frameSpan ) / sqr( frameSpan ) ) );
	}

#ifdef DEBUG
	//cout << frameSpan << endl;
	//for ( auto x : smoothWeight ) cout << x << endl;
#endif

	for ( int i = 0; i < (int)frames.size(); i++ ) {

		Mat smoothedSaliencyMap( frames[i].saliencyMap.size(), CV_32FC1 );

		for ( int y = 0; y < size.height; y++ ) {
			for ( int x = 0; x < size.width; x++ ) {

				Point2f p( x, y );
				double sumSaliency = smoothWeight[0] * frames[i].saliencyMap.at<float>( y, x );
				int frameCount = 1;

				for ( int j = 1; j <= frameSpan; j++ ) {
					if ( i - j < 0 ) break;
					Point2f flow = frames[i - j + 1].backwardFlowMap.at<Point2f>( FloorToInt( p.y ), FloorToInt( p.x ) );
					p.x += flow.x;
					p.y += flow.y;
					if ( CheckOutside( p, size ) ) break;
					sumSaliency += smoothWeight[j] * frames[i - j].saliencyMap.at<float>( FloorToInt( p.y ), FloorToInt( p.x ) );
					frameCount++;
				}

				p = Point2f( x, y );

				for ( int j = 1; j <= frameSpan; j++ ) {
					if ( i + j >= frames.size() ) break;
					Point2f flow = frames[i + j - 1].forwardFlowMap.at<Point2f>( FloorToInt( p.y ), FloorToInt( p.x ) );
					p.x += flow.x;
					p.y += flow.y;
					if ( CheckOutside( p, size ) ) break;
					sumSaliency += smoothWeight[j] * frames[i + j].saliencyMap.at<float>( FloorToInt( p.y ), FloorToInt( p.x ) );
					frameCount++;
				}

				sumSaliency /= frameCount;
				smoothedSaliencyMap.at<float>( y, x ) = sumSaliency;

			}
		}

		normalize( smoothedSaliencyMap, smoothedSaliencyMap, 0, 1, NORM_MINMAX );
		smoothedSaliencyMapVec.push_back( smoothedSaliencyMap );

	}

	for ( size_t i = 0; i < frames.size(); i++ ) {
#ifdef DEBUG
		//imshow( "Saliency Map Before", frames[i].saliencyMap );
	/*	cout << frames[i].frameId << endl;
		imshow( "Smoothed Saliency Map", smoothedSaliencyMapVec[i] );
		waitKey( 0 );*/
#endif
		frames[i].saliencyMap = smoothedSaliencyMapVec[i].clone();
		frames[i].SumSuperpixelSaliency();

	}

}