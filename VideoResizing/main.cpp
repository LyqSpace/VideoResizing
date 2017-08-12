/*
*	Copyright (C)   Lyq root#lyq.me
*	File Name     : main.cpp
*	Creation Time : 2017-3-25
*	Environment   : Windows8.1-64bit VS2013 OpenCV2.4.9
*	Homepage      : http://www.lyq.me
*/


#include "common.h"
#include "io.h"
#include "pretreat.h"
#include "saliency.h"
#include "KeyFrame.h"
#include "Deformation.h"

int main( int argc, char *argv[] ) {

	if ( argc < 5 ) {
		cerr << "Missing arguments.";
		return -2;
	}

	// Get video name
	string videoName = argv[1];
	
	// Get run_type
	const string runTypeArray[4] = { "all", "import", "resize", "export" };
	string runType;
	
	if (!CheckEleExist(runTypeArray, argv[2])) {
		cerr << "Wrong runType argument.";
	}
	runType = argv[2];

	// Get deformed scale.
	double deformedScaleX, deformedScaleY;
	deformedScaleX = atof( argv[3] );
	deformedScaleY = atof( argv[4] );

	/*
	1. Convert video to frames.
	2. Segment frames to shotcut and keyframes.
	*/
	if ( runType == "all" || runType == "import" ) {
		ConvertVideoToFrames( videoName );
		SegFramesToShotCutKeyFrames( videoName );
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
		ReadShotCut( shotArr, videoName );
		ReadKeyArr( keyArr, videoName );

		for ( size_t i = 1; i < shotArr.size(); i++ ) {

			vector<KeyFrame> keyFrames;
			ReadKeyFrames( shotArr[i - 1], shotArr[i], keyArr, keyFrames, videoName );

			QuantizeFrames( keyFrames );
			CalcSuperpixel( keyFrames );
			SegEdges( keyFrames );

			CalcSaliencyMap( keyFrames );
			SmoothSaliencyMap( keyFrames );

			Deformation deformation( keyFrames, videoName );
			deformation.BuildControlPoints();
			deformation.InitDeformation( deformedScaleX, deformedScaleY );
			deformation.MinimizeEnergy();
			deformation.CalcDeformedMap();
			deformation.RenderKeyFrames();

			vector<Mat> frames;
			ReadFrames( shotArr[i - 1], shotArr[i], frames, videoName );
			deformation.RenderFrames( frames, shotArr[i - 1], shotArr[i] );


			cout << endl;

		}

	}

	if ( runType == "all" || runType == "export" ) {

		WriteResizedVideo( videoName );
		WriteMixedVideo( videoName, deformedScaleX, deformedScaleY );

	}



	system( "pause" );

	return 0;
}