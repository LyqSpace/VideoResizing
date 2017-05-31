#ifndef IO_H
#define IO_H

#include <direct.h>
#include "common.h"
#include "KeyFrame.h"

string GetRootFolderPath( const string &videoName );

string GetFramesFolderPath( const string &videoName );

string GetResultsFolderPath( const string &videoName );

void ConvertVideoToFrames( const string &videoName );

void ReadShotCut( vector<int> &, const string &videoName );

void ReadKeyArr( vector<int> &, const string &videoName );

void ReadKeyFrames( int, int, const vector<int> &, vector<KeyFrame> &, const string &videoName );

void ReadFrames( int, int, vector<Mat> &, const string &videoName );

#endif