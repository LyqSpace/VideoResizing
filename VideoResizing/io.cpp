#include "io.h"

void ConvertVideoToFrames( const string &videoName ) {
	
	string fileName = "input/" + videoName;

	VideoCapture cap;
	if ( !cap.open( fileName ) ) {
		cerr << "Could not open the input video.";
	}

	Mat frame;
	int count = 0;
	string frameName;
	int totalCount = (int)cap.get( CV_CAP_PROP_FRAME_COUNT );

	while ( cap.read( frame ) ) {
		printf( "Reading video. Progress rate %d/%d.\r", (int)cap.get( CV_CAP_PROP_POS_FRAMES ), totalCount );

		frameName = "frames/" + to_string( count ) + ".png";
		imwrite( frameName, frame );
		count++;
	}

	printf( "\n" );
	
}

void ReadShotCut( vector<int> &shotArr ) {

	shotArr.clear();
	FILE *file = fopen( "frames/ShotCut.txt", "r" );
	int n;
	fscanf( file, "%d", &n );
	int x;
	while ( n-- ) {
		fscanf( file, "%d", &x );
		shotArr.push_back( x );
	}
	fclose( file );
}

void ReadKeyArr( vector<int> &keyArr ) {

	keyArr.clear();
	FILE *file = fopen( "frames/KeyFrames.txt", "r" );
	int n;
	fscanf( file, "%d", &n );
	int x;
	while ( n-- ) {
		fscanf( file, "%d", &x );
		keyArr.push_back( x );
	}
	fclose( file );
}

void ReadKeyFrames( int shotSt, int shotEd, const vector<int> &keyArr, vector<KeyFrame> &keyFrames ) {

	printf( "Read key frames. Shotcut range: %d to %d.\n", shotSt, shotEd );

	for ( auto keyId : keyArr ) {
		
		if ( keyId < shotSt ) continue;
		if ( keyId >= shotEd ) break;

		string frameName( "frames/" + to_string( keyId ) + ".png" );
		Mat frame = imread( frameName );
		KeyFrame keyFrame( frame );
		keyFrames.push_back( keyFrame );

	}

	string frameName( "frames/" + to_string( shotEd - 1 ) + ".png" );
	Mat frame = imread( frameName );
	KeyFrame keyFrame( frame );
	keyFrames.push_back( keyFrame );

}

void ReadFrames( int shotSt, int shotEd, vector<Mat> &frames ) {

	for ( int i = shotSt; i < shotEd; i++ ) {
		
		printf( "Reading frames. Progress rate %d/%d\r", i - shotSt, shotEd - shotSt - 1 );

		string frameName( "frames/" + to_string( i ) + ".png" );
		Mat frame = imread( frameName );
		frames.push_back( frame.clone() );
	}

	printf( "\n" );

}