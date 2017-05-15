#include "KeyFrame.h"

KeyFrame::KeyFrame( const Mat &_img ) {
	
	img = _img.clone();
	cols = img.cols;
	rows = img.rows;
	cvtColor( img, grayImg, COLOR_BGR2GRAY );
	pixelLabel = Mat( rows, cols, CV_32SC1 );

	superPixelNum = 100;
}

void KeyFrame::SegSuperpixel() {

	SLIC slic;
	int *label;
	
	slic.GenerateSuperpixels(img, superPixelNum );
	
	label = slic.GetLabel();
	superPixelNum = 0;
	for ( int y = 0; y < rows; y++ ) {
		for ( int x = 0; x < cols; x++ ) {
			int labelIndex = y * cols + x;
			pixelLabel.at<int>( Point( x, y ) ) = label[labelIndex];
			superPixelNum = max( superPixelNum, label[labelIndex] );
		}
	}

	superPixelNum++;
	
#ifdef DEBUG
	cout << "Superpixel Num: " << superPixelNum << endl;
	imgWithContours = slic.GetImgWithContours( cv::Scalar( 0, 0, 255 ) );
	imshow( "Img with Contours", imgWithContours );
	waitKey( 0 );
#endif

}