#include "KeyFrame.h"


KeyFrame::KeyFrame( const Mat &_img, int _frameId ) {
	
	img = _img.clone();
	cols = img.cols;
	rows = img.rows;
	size = img.size();
	frameId = _frameId;

	pixelLabel = Mat( rows, cols, CV_32SC1 );
	cvtColor( img, grayImg, COLOR_BGR2GRAY );
	img.convertTo( CIELabImg, CV_32FC3, 1.0 / 255 );
	cvtColor( CIELabImg, CIELabImg, COLOR_BGR2Lab );

	superpixelNum = 100;
	opFlag = false;
	edFlag = false;

}

void KeyFrame::FreeMemory() {

	imgWithContours.release();
	spatialContrastMap.release();
	temporalContrastMap.release();
	paletteMap.release();
	paletteDist.release();

	palette.clear();
	superpixelColorHist.clear();
	superpixelSpatialContrast.clear();
	superpixelTemporalContrast.clear();

	forwardLocalMotionMap.release();
	backwardLocalMotionMap.release();

}

double KeyFrame::CalcColorHistDiff( int spId0, int spId1 ) {

	vector<int> sp0 = superpixelColorHist[spId0];
	vector<int> sp1 = superpixelColorHist[spId1];

	double colorDiff = 0;
	for ( size_t c1 = 0; c1 < sp0.size(); c1++ ) {

		if ( sp0[c1] == 0 ) continue;

		double tmpDiff = 0;
		for ( size_t c2 = 0; c2 < sp1.size(); c2++ ) {

			if ( sp1[c2] == 0 ) continue;
			tmpDiff += paletteDist.at<float>( c1, c2 ) * sp1[c2];
		}
		colorDiff += sp0[c1] * tmpDiff / superpixelCard[spId1];

	}

	colorDiff /= superpixelCard[spId0];

	return colorDiff;
}

double KeyFrame::CalcSpatialDiff( int spId0, int spId1 ) {

	Point p0 = superpixelCenter[spId0];
	Point p1 = superpixelCenter[spId1];
	return NormL2( p0, p1 );
}

void KeyFrame::QuantizeColorSpace(const vector<Vec3f> &_palette, const Mat &_paletteDist) {

	palette = _palette;
	paletteDist = _paletteDist.clone();
	paletteMap = Mat( size, CV_32SC1 );

	for ( int y = 0; y < rows; y++ ) {
		for ( int x = 0; x < cols; x++ ) {
			
			int bestFit = 0;
			double minDiff = INF;
			for ( size_t i = 0; i < palette.size(); i++ ) {
				double tmpDiff = CalcVec3fDiff( CIELabImg.at<Vec3f>( y, x ), palette[i] );
				if ( tmpDiff < minDiff ) {
					minDiff = tmpDiff;
					bestFit = i;
				}
			}
			paletteMap.at<int>( y, x ) = bestFit;
		}
	}

#ifdef DEBUG
	/*Mat quantizeMap( size, CV_32FC3 );
	for ( int y = 0; y < rows; y++ ) {
		for ( int x = 0; x < cols; x++ ) {
			quantizeMap.ptr<Vec3f>( y )[x] = palette[paletteMap.ptr<int>( y )[x]];
		}
	}

	cvtColor( quantizeMap, quantizeMap, COLOR_Lab2BGR );
	quantizeMap.convertTo( quantizeMap, CV_8UC3, 255 );
	imshow( "Quantize Map", quantizeMap );
	waitKey( 1 );*/
#endif
	
}

void KeyFrame::SegSuperpixel() {

	SLIC slic;
	int *label;
	
	slic.GenerateSuperpixels(img, superpixelNum );
	
	superpixelCard = vector<int>( superpixelNum, 0 );
	superpixelCenter = vector<Point>( superpixelNum, Point( 0, 0 ) );
	label = slic.GetLabel();
	superpixelNum = 0;
	for ( int y = 0; y < rows; y++ ) {
		for ( int x = 0; x < cols; x++ ) {
			int pointIndex = y * cols + x;
			int labelIndex = label[pointIndex];
			pixelLabel.at<int>( Point( x, y ) ) = labelIndex;
			superpixelCard[labelIndex]++;
			superpixelCenter[labelIndex] += Point( x, y );
			superpixelNum = max( superpixelNum, labelIndex );
		}
	}

	superpixelNum++;

	for ( int i = 0; i < superpixelNum; i++ ) {
		superpixelCenter[i].x /= superpixelCard[i];
		superpixelCenter[i].y /= superpixelCard[i];
	}
	
#ifdef DEBUG
	//cout << "Superpixel Num: " << superpixelNum << endl;
	/*imgWithContours = slic.GetImgWithContours( cv::Scalar( 0, 0, 255 ) );
	imshow( "Img with Contours", imgWithContours );
	waitKey( 1 );*/
#endif

}

void KeyFrame::MarkBoundLabel() {

	superpixelBoundLabel = vector<int>( superpixelNum, BOUND_NONE );

	for ( int y = 0; y < rows; y++ ) {

		int label;

		label = pixelLabel.at<int>( y, 0 );
		superpixelBoundLabel[label] = BOUND_LEFT;

		label = pixelLabel.at<int>( y, cols - 1 );
		superpixelBoundLabel[label] = BOUND_RIGHT;

	}

	for ( int x = 0; x < cols; x++ ) {

		int label;

		label = pixelLabel.at<int>( 0, x );
		superpixelBoundLabel[label] = BOUND_TOP;

		label = pixelLabel.at<int>( rows - 1, x );
		superpixelBoundLabel[label] = BOUND_BOTTOM;

	}

}

void KeyFrame::CalcSuperpixelColorHist() {

	superpixelColorHist = vector < vector<int> >( superpixelNum, vector<int>( palette.size(), 0 ) );
	for ( int y = 0; y < rows; y++ ) {
		for ( int x = 0; x < cols; x++ ) {
			int labelIndex = pixelLabel.at<int>( y, x );
			int paletteIndex = paletteMap.at<int>( y, x );
			superpixelColorHist[labelIndex][paletteIndex]++;
		}
	}
}

void KeyFrame::CalcSpatialContrast() {

	superpixelSpatialContrast = vector<double>( superpixelNum, 0 );

	for ( int i = 0; i < superpixelNum; i++ ) {
		for ( int j = i + 1; j < superpixelNum; j++ ) {
			double spatialDiff = CalcSpatialDiff( i, j );
			double colorDiff = CalcColorHistDiff( i, j );

			double spatialWeight = exp( -spatialDiff / SIGMA_DIST );
			double colorContrast = 1 - exp( -colorDiff / SIGMA_COLOR );

			superpixelSpatialContrast[i] += spatialWeight * colorContrast * superpixelCard[j];
			superpixelSpatialContrast[j] += spatialWeight * colorContrast * superpixelCard[i];
#ifdef DEBUG
			//printf( "i: %d, j: %d, SpatialDiff: %.3lf, ColorDiff: %.3lf, spatialWeight: %.3lf, colorContrast: %.3lf\n", i, j, spatialDiff, colorDiff, spatialWeight, colorContrast );
#endif
		}
	}

	for ( int i = 0; i < superpixelNum; i++ ) {
		superpixelSpatialContrast[i] *= superpixelCard[i];
	}
	
#ifdef DEBUG
	////cout << "Before:" << endl;
	////for ( auto ele : superpixelSpatialContrast ) cout << ele << endl;
	//NormalizeVec( superpixelSpatialContrast );
	////cout << "After:" << endl;
	////for ( auto ele : superpixelSpatialContrast ) cout << ele << endl;

	//spatialContrastMap = Mat::zeros( size, CV_32FC1 );

	//for ( int y = 0; y < rows; y++ ) {
	//	for ( int x = 0; x < cols; x++ ) {
	//		spatialContrastMap.at<float>( y, x ) = superpixelSpatialContrast[pixelLabel.at<int>( y, x )];
	//	}
	//}
	//imshow( "Spatial Contrast Map", spatialContrastMap );
	//waitKey( 1 );
#endif
}

void KeyFrame::CalcTemporalContrast() {

	superpixelTemporalContrast = vector<double>( superpixelNum, 0 );

	if ( !opFlag ) {
		for ( int y = 0; y < rows; y++ ) {
			for ( int x = 0; x < cols; x++ ) {
				int label = pixelLabel.at<int>( y, x );
				superpixelTemporalContrast[label] += NormL2( backwardLocalMotionMap.at<Point2f>( y, x ) );
			}
		}
	}

	if ( !edFlag ) {
		for ( int y = 0; y < rows; y++ ) {
			for ( int x = 0; x < cols; x++ ) {
				int label = pixelLabel.at<int>( y, x );
				superpixelTemporalContrast[label] += NormL2( forwardLocalMotionMap.at<Point2f>( y, x ) );
			}
		}
	}

	if ( !(opFlag || edFlag) ) {
		for ( auto &item : superpixelTemporalContrast ) {
			item /= 2;
		}
	}
	
#ifdef DEBUG
	////cout << "Before:" << endl;
	////for ( auto ele : superpixelTemporalContrast ) cout << ele << endl;
	//NormalizeVec( superpixelTemporalContrast );
	////cout << "After:" << endl;
	////for ( auto ele : superpixelTemporalContrast ) cout << ele << endl;

	//temporalContrastMap = Mat::zeros( size, CV_32FC1 );

	//for ( int y = 0; y < rows; y++ ) {
	//	for ( int x = 0; x < cols; x++ ) {
	//		int label = pixelLabel.at<int>( y, x );
	//		temporalContrastMap.at<float>( y, x ) = superpixelTemporalContrast[label];
	//	}
	//}
	//imshow( "Motion Contrast Map", temporalContrastMap );
	//waitKey( 1 );
#endif

}

void KeyFrame::CalcSaliencyMap() {

	superpixelSaliency = vector<double>( superpixelNum, 0 );

	for ( int i = 0; i < superpixelNum; i++ ) {
		superpixelSaliency[i] = superpixelSpatialContrast[i] * superpixelTemporalContrast[i];
#ifdef DEBUG
		/*double tmp1 = superpixelSpatialContrast[i] + superpixelTemporalContrast[i];
		double tmp2 = superpixelSpatialContrast[i] * superpixelTemporalContrast[i];
		printf( "Spatial: %.3lf, Temporal: %.3lf, Plus: %.3lf, Multi: %.3lf\n", superpixelSpatialContrast[i], superpixelTemporalContrast[i], tmp1, tmp2 );*/
#endif DEBUG
	}

	NormalizeVec( superpixelSaliency );

	saliencyMap = Mat::zeros( size, CV_32FC1 );

	for ( int y = 0; y < rows; y++ ) {
		for ( int x = 0; x < cols; x++ ) {
			int label = pixelLabel.at<int>( y, x );
			saliencyMap.at<float>( y, x ) = superpixelSaliency[label];
		}
	}
#ifdef DEBUG
	//imshow( "Saliency Map", saliencyMap );
	//waitKey( 1 );
#endif
}

void KeyFrame::SumSuperpixelSaliency() {

	superpixelSaliency = vector<double>( superpixelNum, 0 );

	for ( int y = 0; y < rows; y++ ) {
		for ( int x = 0; x < cols; x++ ) {
			int label = pixelLabel.at<int>( y, x );
			superpixelSaliency[label] += saliencyMap.at<float>( y, x );
		}
	}

	for ( int i = 0; i < superpixelNum; i++ ) {
		superpixelSaliency[i] /= superpixelCard[i];
	}

	NormalizeVec( superpixelSaliency );

}