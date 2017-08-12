#ifndef IO_H
#define IO_H

#include <direct.h>
#include "common.h"
#include "KeyFrame.h"

string GetRootFolderPath( const string &videoName );

string GetFramesFolderPath( const string &videoName );

string GetResultsFolderPath( const string &videoName );

string GetKeyFramesFolderPath( const string &videoName );

string GetOutputVideoPath( const string &videoName, int type );

void ConvertVideoToFrames( const string &videoName );

void ReadShotCut( vector<int> &, const string &videoName );

void ReadKeyArr( vector<int> &, const string &videoName );

void ReadKeyFrames( int, int, const vector<int> &, vector<KeyFrame> &, const string &videoName );

void ReadFrames( int, int, vector<Mat> &, const string &videoName  );

void WriteKeyFrameEdgeImg( int frameId, const Mat &edgeImg, const string &videoName );

void WriteDeformedImg( int frameId, const Mat &img, const string &videoName );

#define RESIZED_VIDEO 0
#define MIXED_VIDEO 1

void WriteResizedVideo( const string &videoName );

void WriteMixedVideo( const string &videoName, double deformedScaleX, double deformedScaleY );

#endif