#include "io.h"

string GetRootFolderPath( const string &videoName ) {
	
	int splitPos = videoName.find( '.' );
	string path = TEST_PATH + videoName.substr( 0, splitPos ) + "/";
	return path;

}

string GetFramesFolderPath( const string &videoName ) {

	string rootPath = GetRootFolderPath( videoName );
	return rootPath + "frames/";

}

string GetResultsFolderPath( const string &videoName ) {

	string rootPath = GetRootFolderPath( videoName );
	return rootPath + "results/";

}

string GetKeyFramesFolderPath( const string &videoName ) {

	string rootPath = GetRootFolderPath( videoName );
	return rootPath + "keyframes/";

}

string GetOutputVideoPath( const string &videoName, int type ) {

	int splitPos = videoName.find( '.' );
	string name = videoName.substr( 0, splitPos );
	
	string path;
	if ( type == RESIZED_VIDEO ) {
		path = TEST_PATH + name + "/" + name + "_resize.avi";
	} else if ( type == MIXED_VIDEO ) {
		path = TEST_PATH + name + "/" + name + "_mixed.avi";
	}
	return path;

}

void ConvertVideoToFrames( const string &videoName ) {
	
	_mkdir( TEST_PATH.c_str() );

	string path = GetRootFolderPath( videoName );
	_mkdir( path.c_str() );
	path = GetFramesFolderPath( videoName );
	_mkdir( path.c_str() );
	path = GetResultsFolderPath( videoName );
	_mkdir( path.c_str() );
	path = GetKeyFramesFolderPath( videoName );
	_mkdir( path.c_str() );

	string fileName = INPUT_PATH + videoName;
	VideoCapture cap;

	if ( !cap.open( fileName ) ) {
		cerr << "Could not open the input video.";
	}

	Mat frame;
	int count = 0;
	string frameName;
	int totalCount = (int)cap.get( CV_CAP_PROP_FRAME_COUNT );
	string framesFolderPath = GetFramesFolderPath( videoName );

	while ( cap.read( frame ) ) {
		printf( "Reading video. Progress rate %d/%d.\r", (int)cap.get( CV_CAP_PROP_POS_FRAMES ), totalCount );

		frameName = framesFolderPath + to_string( count ) + ".png";
		frame = frame( Rect( 0, 0, frame.cols - 5, frame.rows ) );
		imwrite( frameName, frame );
		count++;
	}

	printf( "\n" );
	
}

void ReadShotCut( vector<int> &shotArr, const string &videoName ) {

	shotArr.clear();
	string filePath = GetRootFolderPath( videoName ) + "ShotCut.txt";

	FILE *file = fopen( filePath.c_str(), "r" );
	int n;
	fscanf( file, "%d", &n );
	int x;
	while ( n-- ) {
		fscanf( file, "%d", &x );
		shotArr.push_back( x );
	}
	fclose( file );
}

void ReadKeyArr( vector<int> &keyArr, const string &videoName ) {

	keyArr.clear();
	string filePath = GetRootFolderPath( videoName ) + "KeyFrames.txt";

	FILE *file = fopen( (filePath).c_str(), "r" );
	int n;
	fscanf( file, "%d", &n );
	int x;
	while ( n-- ) {
		fscanf( file, "%d", &x );
		keyArr.push_back( x );
	}
	fclose( file );
}

void ReadKeyFrames( int shotSt, int shotEd, const vector<int> &keyArr, vector<KeyFrame> &keyFrames, const string &videoName ) {

#define DEBUG_SHORT_FRAMES

	printf( "Read key frames. Shotcut range: %d to %d. ", shotSt, shotEd );
	
	string framesFolderPath = GetFramesFolderPath( videoName );

	for ( auto keyId : keyArr ) {
		
		if ( keyId < shotSt ) continue;
		if ( keyId >= shotEd ) break;

		string frameName( framesFolderPath + to_string( keyId ) + ".png" );
		Mat frame = imread( frameName );
		KeyFrame keyFrame( frame, keyId );
		keyFrames.push_back( keyFrame );

#ifdef DEBUG_SHORT_FRAMES
		if ( keyFrames.size() > 0 ) break;
#endif
	}

	string frameName( framesFolderPath + to_string( shotEd - 1 ) + ".png" );
	Mat frame = imread( frameName );
	KeyFrame keyFrame( frame, shotEd - 1 );
	keyFrames.push_back( keyFrame ); 

	keyFrames.front().opFlag = true;
	keyFrames.back().edFlag = true;

	printf( "%d key frames.\n", keyFrames.size() );

}

void ReadFrames( int shotSt, int shotEd, vector<Mat> &frames, const string &videoName ) {

	string framesFolderPath = GetFramesFolderPath( videoName );

	for ( int i = shotSt; i < shotEd; i++ ) {
		
		printf( "Reading frames. Progress rate %d/%d.\r", i - shotSt, shotEd - shotSt - 1 );

		string frameName( framesFolderPath + to_string( i ) + ".png" );
		Mat frame = imread( frameName );
		frames.push_back( frame.clone() );
	}

	printf( "\n" );

}

void WriteKeyFrameEdgeImg( int frameId, const Mat &edgeImg, const string &videoName ) {

	string keyFramesFolderPath = GetKeyFramesFolderPath( videoName );
	string frameName( keyFramesFolderPath + to_string( frameId ) + ".png" );
	imwrite( frameName, edgeImg );

}

void WriteDeformedImg( int frameId, const Mat &img, const string &videoName ) {

	string resultsFolderPath = GetResultsFolderPath( videoName );
	string frameName( resultsFolderPath + to_string( frameId ) + ".png" );
	imwrite( frameName, img );

}

void WriteResizedVideo( const string &videoName ) {
	
	printf( "Write output video.\n" );

	string videoPath = GetOutputVideoPath( videoName, RESIZED_VIDEO );
	string resultsFolderPath = GetResultsFolderPath( videoName );
	
	string frameName( resultsFolderPath + to_string( 0 ) + ".png" );
	Mat img = imread( frameName );

	VideoWriter video;
	video.open( videoPath, CV_FOURCC( 'M', 'J', 'P', 'G' ), 15, img.size() );
	video << img;

	int frameIndex = 1;
	while ( true ) {
		frameName = resultsFolderPath + to_string( frameIndex ) + ".png" ;
		img = imread( frameName );
		if ( img.empty() ) break;
		video << img;
		frameIndex++;
	}

}

void WriteMixedVideo( const string &videoName, double deformedScaleX, double deformedScaleY ) {

	printf( "Write mixed input & output video.\n" );

	const int gap = 10;

	string videoPath = GetOutputVideoPath( videoName, MIXED_VIDEO );
	string resultsFolderPath = GetResultsFolderPath( videoName );
	string framesFolderPath = GetFramesFolderPath( videoName );

	string frameName = framesFolderPath + to_string( 0 ) + ".png";
	Mat inputImg = imread( frameName );
	frameName = resultsFolderPath + to_string( 0 ) + ".png" ;
	Mat deformedImg = imread( frameName );
	Mat uniformedImg;
	resize( inputImg, uniformedImg, Size(), deformedScaleX, deformedScaleY );

	Size size = Size(inputImg.cols + deformedImg.cols + deformedImg.cols + gap * 2, inputImg.rows);
	Rect rect0( 0, 0, inputImg.cols, inputImg.rows );
	Rect rect1( inputImg.cols + gap, 0, deformedImg.cols, deformedImg.rows );
	Rect rect2( inputImg.cols + deformedImg.cols + 2 * gap, 0, deformedImg.cols, deformedImg.rows );
	Mat mixedImg( size, CV_8UC3, Scalar( 255, 255, 255 ) );

	inputImg.copyTo( mixedImg( rect0 ) );
	uniformedImg.copyTo(mixedImg(rect1));
	deformedImg.copyTo( mixedImg( rect2 ) );

#ifdef DEBUG_WRITE_MIXED_VIDEO
	imshow( "mixed", mixedImg );
	imshow( "input", inputImg );
	imshow( "deformed", deformedImg );
	waitKey();
#endif

	VideoWriter video;
	video.open( videoPath, CV_FOURCC( 'M', 'J', 'P', 'G' ), 15, size );
	video << mixedImg;

	int frameIndex = 1;
	while ( true ) {
		frameName = framesFolderPath + to_string( frameIndex ) + ".png";
		inputImg = imread( frameName );
		frameName = resultsFolderPath + to_string( frameIndex ) + ".png";
		deformedImg = imread( frameName );

		if ( inputImg.empty() ) break;
		if ( deformedImg.empty() ) break;
		
		resize( inputImg, uniformedImg, Size(), deformedScaleX, deformedScaleY );

		inputImg.copyTo( mixedImg( rect0 ) );
		uniformedImg.copyTo(mixedImg(rect1));
		deformedImg.copyTo( mixedImg( rect2 ) );

#ifdef DEBUG_WRITE_MIXED_VIDEO
		imshow( "input", inputImg );
		imshow( "deformed", deformedImg );
		waitKey();
#endif

		video << mixedImg;
		frameIndex++;
	}

}