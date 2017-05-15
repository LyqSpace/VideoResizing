#ifndef PRETREAT_H
#define PRETREAT_H

#include "common.h"
#include "KeyFrame.h"

void SegFramesToShotCutKeyFrames();

void CalcSalientMap( const vector<KeyFrame> & );

void CalcSuperpixel( const vector<KeyFrame> & );

#endif