#ifndef IO_H
#define IO_H

#include "common.h"
#include "KeyFrame.h"

void ConvertVideoToFrames( const string & );

void ReadShotCut( vector<int> & );

void ReadKeyArr( vector<int> & );

void ReadKeyFrames( int, int, const vector<int> &, vector<KeyFrame> & );

void ReadFrames( int, int, vector<Mat> & );

#endif