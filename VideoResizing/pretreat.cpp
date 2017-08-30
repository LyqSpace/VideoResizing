#include "pretreat.h"

void SegFramesToShotCutKeyFrames( const string &videoName ) {

	const int winSize = 7;
	const double shotThres = THRES_SHOTCUT;
	const double keyThres = THRES_KEYFRAME;

	Mat inputFrame, oldFrame, diffFrame;
	int frameCount = 0;
	string frameName;
	vector<double> diffArr;
	vector<int> shotArr, keyArr;
	string framesFolderPath = GetFramesFolderPath( videoName );

	while ( true ) {

		frameName = framesFolderPath + to_string( frameCount ) + ".png";
		inputFrame = imread( frameName );
		if ( inputFrame.empty() ) break;

		printf( "Segment shotcuts and detect key frames. Scan frame %d.\r", frameCount + 1 );

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

	string filePath = GetRootFolderPath(videoName) + "ShotCut.txt";
	FILE *file = fopen( filePath.c_str(), "w" );

	fprintf( file, "%d\n", shotArr.size() );
	for ( auto item : shotArr ) {
		fprintf( file, "%d\n", item );
	}
	fclose( file );

	filePath = GetRootFolderPath( videoName ) + "KeyFrames.txt";
	file = fopen( filePath.c_str(), "w" );
	fprintf( file, "%d\n", keyArr.size() );
	for ( auto item : keyArr ) {
		fprintf( file, "%d\n", item );
	}
	fclose( file );

}

bool CmpVec3f0( const Vec3f &c0, const Vec3f &c1 ) {
	return c0.val[0] < c1.val[0];
}

bool CmpVec3f1( const Vec3f &c0, const Vec3f &c1 ) {
	return c0.val[0] < c1.val[0];
}

bool CmpVec3f2( const Vec3f &c0, const Vec3f &c1 ) {
	return c0.val[0] < c1.val[0];
}

void CalcPalette( const vector<KeyFrame> &frames, vector< Vec3f> &palette ) {

	vector<Vec3f> colorSet;

	for ( auto const &frame : frames ) {
		for ( int y = 0; y < frame.rows; y += 10 ) {
			for ( int x = 0; x < frame.cols; x += 10 ) {
				colorSet.push_back( frame.CIELabImg.at<Vec3f>( y, x ) );
			}
		}
	}
	
	vector< vector<Vec3f> > medianCutQue;
	medianCutQue.push_back( colorSet );

	for ( int level = 0; level < QUANTIZE_LEVEL; level++ ) {

		vector< vector<Vec3f> > tmpQue;

		for ( size_t i = 0; i < medianCutQue.size(); i++ ) {

			Vec3f minColor( 255, 255, 255 );
			Vec3f maxColor( 0, 0, 0 );
			for ( size_t j = 0; j < medianCutQue[i].size(); j++ ) {
				for ( int k = 0; k < 3; k++ ) {
					minColor.val[k] = min( minColor.val[k], medianCutQue[i][j].val[k] );
					maxColor.val[k] = max( maxColor.val[k], medianCutQue[i][j].val[k] );
				}
			}

			int cut_dimension = 0;
			double max_range = 0;
			for ( int k = 0; k < 3; k++ ) {
				if ( maxColor.val[k] - minColor.val[k] > max_range ) {
					max_range = maxColor.val[k] - minColor.val[k];
					cut_dimension = k;
				}
			}

			switch ( cut_dimension ) {
				case 0:
					sort( medianCutQue[i].begin(), medianCutQue[i].end(), CmpVec3f0 );
					break;
				case 1:
					sort( medianCutQue[i].begin(), medianCutQue[i].end(), CmpVec3f1 );
					break;
				case 2:
					sort( medianCutQue[i].begin(), medianCutQue[i].end(), CmpVec3f2 );
					break;
				default:
					cout << "error in cut" << endl;
					exit( 0 );
			}

			int mid_pos = medianCutQue[i].size() / 2;
			vector<Vec3f> part0( medianCutQue[i].begin(), medianCutQue[i].begin() + mid_pos );
			vector<Vec3f> part1( medianCutQue[i].begin() + mid_pos, medianCutQue[i].end() );

			tmpQue.push_back( part0 );
			tmpQue.push_back( part1 );
		}

		medianCutQue = tmpQue;
	}

	palette.clear();
	for ( size_t i = 0; i < medianCutQue.size(); i++ ) {

		Vec3f meanColor = medianCutQue[i][medianCutQue[i].size() >> 1];
		palette.push_back( meanColor );
	}
}

void QuantizeFrames( vector<KeyFrame> &frames ) {

	printf( "Quantize key frames color space.\n" );

	printf( "\tCalculate palette.\n" );

	vector< Vec3f> palette;
	CalcPalette( frames, palette );

	int paletteSize = palette.size();
	Mat paletteDist = Mat::zeros( paletteSize, paletteSize, CV_32FC1 );
	for ( size_t c1 = 0; c1 < palette.size(); c1++ ) {
		for ( size_t c2 = c1 + 1; c2 < palette.size(); c2++ ) {
			paletteDist.at<float>( c1, c2 ) = (float)CalcVec3fDiff( palette[c1], palette[c2] );
			paletteDist.at<float>( c2, c1 ) = paletteDist.at<float>( c1, c2 );
		}
	}

	printf( "\tQuantize each frames.\n" );

	for ( auto &frame : frames ) {
		frame.QuantizeColorSpace(palette, paletteDist);
	}
}

void CalcSuperpixel( vector<KeyFrame> &frames ) {

	printf( "Calculate key frames superpixels.\n" );

	for ( auto &frame : frames ) {
		frame.SegSuperpixel();
		frame.MarkBoundLabel();
		frame.CalcSuperpixelColorHist();
	}
}

void DetectSpatialStructure( vector<KeyFrame> &frames ) {

	printf( "Segment key frames edges.\n" );

	for ( auto &frame : frames ) {
	}

}