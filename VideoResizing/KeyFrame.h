#ifndef KEYFRAME_H
#define KEYFRAME_H

#include <ctime>
#include "common.h"
#include "slic.h"

class KeyFrame {
private:
	Mat img, grayImg, imgWithContours;
	Mat pixelLabel;
	int cols, rows;
public:
	int superPixelNum;

	KeyFrame( const Mat & );
	void SegSuperpixel();

};

#endif