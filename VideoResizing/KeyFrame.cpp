#include "KeyFrame.h"


KeyFrame::KeyFrame( const Mat &_img ) {
	
	img = _img.clone();
	cols = img.cols;
	rows = img.rows;
	size = img.size();

	pixelLabel = Mat( rows, cols, CV_32SC1 );
	cvtColor( img, grayImg, COLOR_BGR2GRAY );
	img.convertTo( CIELabImg, CV_32FC3, 1.0 / 255 );
	cvtColor( CIELabImg, CIELabImg, COLOR_BGR2Lab );

	superPixelNum = 50;
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

	int tmpX = sqr( superpixelCenter[spId0].x - superpixelCenter[spId1].x );
	int tmpY = sqr( superpixelCenter[spId0].y - superpixelCenter[spId1].y );
	return sqrt( tmpX + tmpY );
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
	
	slic.GenerateSuperpixels(img, superPixelNum );
	
	superpixelCard = vector<int>( superPixelNum, 0 );
	superpixelCenter = vector<Point>( superPixelNum, Point( 0, 0 ) );
	label = slic.GetLabel();
	superPixelNum = 0;
	for ( int y = 0; y < rows; y++ ) {
		for ( int x = 0; x < cols; x++ ) {
			int pointIndex = y * cols + x;
			int labelIndex = label[pointIndex];
			pixelLabel.at<int>( Point( x, y ) ) = labelIndex;
			superpixelCard[labelIndex]++;
			superpixelCenter[labelIndex] += Point( x, y );
			superPixelNum = max( superPixelNum, labelIndex );
		}
	}

	superPixelNum++;

	for ( int i = 0; i < superPixelNum; i++ ) {
		superpixelCenter[i].x /= superpixelCard[i];
		superpixelCenter[i].y /= superpixelCard[i];
	}
	
#ifdef DEBUG
	//cout << "Superpixel Num: " << superPixelNum << endl;
	/*imgWithContours = slic.GetImgWithContours( cv::Scalar( 0, 0, 255 ) );
	imshow( "Img with Contours", imgWithContours );
	waitKey( 1 );*/
#endif

}

void KeyFrame::CalcSuperpixelColorHist() {

	superpixelColorHist = vector < vector<int> >( superPixelNum, vector<int>( palette.size(), 0 ) );
	for ( int y = 0; y < rows; y++ ) {
		for ( int x = 0; x < cols; x++ ) {
			int labelIndex = pixelLabel.at<int>( y, x );
			int paletteIndex = paletteMap.at<int>( y, x );
			superpixelColorHist[labelIndex][paletteIndex]++;
		}
	}
}

void KeyFrame::CalcSpatialContrast() {

	spatialContrastMap = Mat::zeros( size, CV_32FC1 );
	superpixelSpatialContrast = vector<double>( superPixelNum, 0 );

	for ( int i = 0; i < superPixelNum; i++ ) {
		for ( int j = i + 1; j < superPixelNum; j++ ) {
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

#ifdef DEBUG
	//cout << "Before:" << endl;
	//for ( auto ele : superpixelSpatialContrast ) cout << ele << endl;
	normalizeVec( superpixelSpatialContrast );
	//cout << "After:" << endl;
	//for ( auto ele : superpixelSpatialContrast ) cout << ele << endl;

	for ( int y = 0; y < rows; y++ ) {
		for ( int x = 0; x < cols; x++ ) {
			spatialContrastMap.at<float>( y, x ) = superpixelSpatialContrast[pixelLabel.at<int>( y, x )];
		}
	}
	imshow( "Spatial Contrast Map", spatialContrastMap );
	waitKey( 1 );
#endif
}

void KeyFrame::CalcTemporalContrast() {

}