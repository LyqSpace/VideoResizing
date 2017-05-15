#include "pretreat.h"

void SegFramesToShotCutKeyFrames() {

	const int winSize = 7;
	const double shotThres = 10;
	const double keyThres = 2;

	Mat inputFrame, oldFrame, diffFrame;
	int frameCount = 0;
	string frameName;
	vector<double> diffArr;
	vector<int> shotArr, keyArr;

	while ( true ) {

		frameName = "frames/" + to_string( frameCount ) + ".png";
		inputFrame = imread( frameName );
		if ( inputFrame.empty() ) break;

		printf( "Segment shotcuts and detect key frames. Scan frame %d.\r", frameCount );

		if ( frameCount > 0 ) {
			absdiff( inputFrame, oldFrame, diffFrame );
			Scalar m = mean( diffFrame );

			double temp = 0;
			for ( int k = 0; k < 3; k++ ) temp += m.val[k];
			
			diffArr.push_back( temp );

			if ( temp > 20 ) {
				keyArr.push_back( frameCount );
			}

		} else {
			shotArr.push_back( 0 );
			keyArr.push_back( 0 );
		}

		oldFrame = inputFrame.clone();

		if ( frameCount >= winSize ) {
			vector< pair<double, int>> sortArr;
			pair<double, int> x;
			for ( int i = frameCount - winSize; i < frameCount; i++ ) {
				x.first = diffArr[i];
				x.second = i;
				sortArr.push_back( x );
			}

			sort( sortArr.begin(), sortArr.end() );

			if ( sortArr[6].first > sortArr[5].first * shotThres && abs( sortArr[5].first - eps ) > 0 ) {
				if ( shotArr.back() != sortArr[6].second + 1 ) {
					shotArr.push_back( sortArr[6].second + 1 );
				}
			}

		}

		frameCount++;

	}

	printf( "\n" );

	shotArr.push_back( frameCount );
	keyArr.push_back( frameCount );

	FILE *file = fopen( "frames/ShotCut.txt", "w" );
	fprintf( file, "%d\n", shotArr.size() );
	for ( auto item : shotArr ) {
		fprintf( file, "%d\n", item );
	}
	fclose( file );

	file = fopen( "frames/KeyFrames.txt", "w" );
	fprintf( file, "%d\n", keyArr.size() );
	for ( auto item : keyArr ) {
		fprintf( file, "%d\n", item );
	}
	fclose( file );

}

void DrawOpticalFlow( const Mat &flowMat, const Mat &frame ) {
	
	Mat img = frame.clone();
	int step = 10;

	for ( int y = 0; y < flowMat.rows; y += step ) {
		for ( int x = 0; x < flowMat.cols; x += step ) {
			Point2f flow = flowMat.at<Point2f>( y, x );
			line( img, Point( x, y ), Point( cvRound( x + flow.x ), cvRound( y + flow.y ) ), Scalar( 0, 255, 0 ) ); 
			circle( img, Point( x, y ), 2, Scalar( 0, 0, 255 ) );
		}
	}
	imshow( "Optical Flow", img );

}

void CalcSalientMap( const vector<KeyFrame> &keyFrames ) {
/*
	vector<Mat> dxMatArr;
	Mat preGrayMat;
	int count = 0;
	int canny_thres = 70;

	for ( auto frame : keyFrames ) {

		Mat grayMat, edgeMat, flowMat;

		cvtColor( frame, grayMat, CV_BGR2GRAY );
		blur( grayMat, edgeMat, Size( 3, 3 ) );
		Canny( edgeMat, edgeMat, canny_thres, canny_thres * 3 );

		imshow( "Canny", edgeMat );

		if ( count > 0 ) {
			calcOpticalFlowFarneback( preGrayMat, grayMat, flowMat, 0.5, 3, 30, 3, 5, 1.2, OPTFLOW_FARNEBACK_GAUSSIAN );
			DrawOpticalFlow( flowMat, frame );
		}

		preGrayMat = grayMat.clone();

		count++;

		imshow( "input", frame );

		waitKey( 1 );
	}*/
}

void CalcSuperpixel( const vector<KeyFrame> &keyFrames ) {
	for ( auto keyFrame : keyFrames ) {
		keyFrame.SegSuperpixel();
	}
}