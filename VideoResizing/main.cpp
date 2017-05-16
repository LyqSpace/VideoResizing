/*
*	Copyright (C)   Lyq root#lyq.me
*	File Name     : main.cpp
*	Creation Time : 2v  017-3-25
*	Environment   : Windows8.1-64bit VS2013 OpenCV2.4.9
*	Homepage      : http://www.lyq.me
*/


#include "common.h"
#include "io.h"
#include "pretreat.h"
#include "KeyFrame.h"

int main( int argc, char *argv[] ) {

	if ( argc < 2 ) {
		cerr << "Missing arguments.";
		return -2;
	}

	string videoName = argv[1];
	
	const string runTypeArray[3] = { "all", "resize", "render" };
	string runType(runTypeArray[0]);

	// video_name run_type
	if ( argc == 3 ) {
		if (!checkEleExist(runTypeArray, argv[2])) {
			cerr << "Wrong runType argument.";
		}
		runType = argv[2];
	}

	/*
	1. Convert video to frames.
	2. Segment frames to shotcut and keyframes.
	*/
	if ( runType == "all" ) {
		ConvertVideoToFrames( videoName );
		SegFramesToShotCutKeyFrames();
	}

	/*
	1. For each shotcut:
	2.     Read keyframes.
	3.     For each keyframe:
	4.         Segment keyframe to superpixels.
	5.         Calculate keyframe salient map.
	6.         Build deformation spatial constraints.
	7.     Build deformation temporal constraints.
	8.     Solve deformation energy functions.
	9.     Calculate pixel-wise deformation map.
	10.    Apply keyframe deformation map to other frames.
	*/
	if ( runType == "all" || runType == "resize" ) {
		
		vector<int> shotArr, keyArr;
		ReadShotCut( shotArr );
		ReadKeyArr( keyArr );

		for ( size_t i = 1; i < shotArr.size(); i++ ) {

			vector<KeyFrame> keyFrames;
			ReadKeyFrames( shotArr[i - 1], shotArr[i], keyArr, keyFrames );

			QuantizeFrames( keyFrames );
			CalcSuperpixel( keyFrames );
			CalcSaliencyMap( keyFrames );

			vector<Mat> frames;
			ReadFrames( shotArr[i - 1], shotArr[i], frames );

			
		}

	}

	if ( runType == "all" || runType == "resize" || runType == "render" ) {

	}



	system( "pause" );

	return 0;
}