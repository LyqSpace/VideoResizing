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

void ConvertVideoToFrames( const string &videoName ) {
	
	mkdir( TEST_PATH.c_str() );

	string path = GetRootFolderPath( videoName );
	mkdir( path.c_str() );
	path = GetFramesFolderPath( videoName );
	mkdir( path.c_str() );
	path = GetResultsFolderPath( videoName );
	mkdir( path.c_str() );

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

	printf( "Read key frames. Shotcut range: %d to %d. ", shotSt, shotEd );
	
	string framesFolderPath = GetFramesFolderPath( videoName );

	for ( auto keyId : keyArr ) {
		
		if ( keyId < shotSt ) continue;
		if ( keyId >= shotEd ) break;

		string frameName( framesFolderPath + to_string( keyId ) + ".png" );
		Mat frame = imread( frameName );
		KeyFrame keyFrame( frame, keyId );
		keyFrames.push_back( keyFrame );

#ifdef DEBUG
		if ( keyFrames.size() > 5 ) break;
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