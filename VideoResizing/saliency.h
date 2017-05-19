#ifndef SALIENCY_H
#define SALIENCY_H

#include "common.h"
#include "KeyFrame.h"

void DrawOpticalFlow( const Mat &, const Mat & );

void CalcMotion( const Mat &, Mat &, Point2f & );

void CalcSaliencyMap( vector<KeyFrame> & );

void SmoothSaliencyMap( vector<KeyFrame> & );

#endif