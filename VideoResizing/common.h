#ifndef COMMON_H
#define COMMON_H

#define _CRT_SECURE_NO_WARNINGS
#define DEBUG

#include <string>
#include <cstdlib>
#include <algorithm>
#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

const double INF = 1e10;
const double eps = 1e-8;
const double VERY_SMALL = 1e-2;
const double ITER_TERMINATE = 0.1;
const int MIN_ENERGY_ITERS = 300;
const int NEIGHBORS_NUM = 8;
const Point neighbors[NEIGHBORS_NUM] = {
	Point( 0, 1 ), Point( 1, 0 ), Point( -1, 0 ), Point( 0, -1 ), 
	Point( 1, 1 ), Point( 1, -1 ), Point( -1, 1 ), Point( -1, -1 ) 
};
const int LARGE_NEIGHBORS_NUM = 24;
const Point largeNeighbors[LARGE_NEIGHBORS_NUM] = {
	Point( -2, -2 ), Point( -1, -2 ), Point( 0, -2 ), Point( 1, -2 ), Point( 2, -2 ),
	Point( -2, -1 ), Point( -1, -1 ), Point( 0, -1 ), Point( 1, -1 ), Point( 2, -1 ),
	Point( -2, 0 ), Point( -1, 0 ), Point( 1, 0 ), Point( 2, 0 ),
	Point( -2, 1 ), Point( -1, 1 ), Point( 0, 1 ), Point( 1, 1 ), Point( 2, 1 ),
	Point( -2, 2 ), Point( -1, 2 ), Point( 0, 2 ), Point( 1, 2 ), Point( 2, 2 )
};
const string TEST_PATH = "./test/";
const string INPUT_PATH = "./input/";

#define sqr(_x) ((_x) * (_x))

const int THRES_SHOTCUT = 10;
const int THRES_KEYFRAME = 2;
const int QUANTIZE_LEVEL = 5;
const int MAX_SUPERPIXEL_NUM = 50;
const double SIGMA_COLOR = 40;
const double SIGMA_DIST = 200;
const int SALIENCY_SMOOTH_SPAN = 11;

const double ALPHA_SALIENCY = 1;
const double ALPHA_SPATIAL = 1;
const double ALPHA_TEMPORAL = 1;


template<class T, size_t N>
bool CheckEleExist( const T( &eleArray )[N], const string &eleVal ) {
	if ( find( begin( eleArray ), end( eleArray ), eleVal ) != end( eleArray ) ) {
		return true;
	} else {
		return false;
	}
}

template<class T>
void NormalizeVec( vector<T> &vec ) {
	T eleMin = INF;
	T eleMax = -INF;
	for ( auto ele : vec ) {
		eleMin = min( eleMin, ele );
		eleMax = max( eleMax, ele );
	}
	eleMin -= VERY_SMALL;
	eleMax += VERY_SMALL;
	T eleSpan = eleMax - eleMin;
	for ( auto &ele : vec ) {
		ele = (ele - eleMin) / eleSpan;
	}
}

template<class T>
double SqrNormL2( const T &p ) {
	return sqr( p.x ) + sqr( p.y );
}

template<class T>
double NormL2( const T &p ) {
	return sqrt( SqrNormL2( p ) );
}

template<class T>
bool CheckOutside( const T &p, const Size &size ) {
	if ( p.x < 0 || p.x >= size.width || p.y < 0 || p.y >= size.height ) {
		return true;
	} else {
		return false;
	}
}

template<class T>
int RoundToInt( const T &d ) {
	return int( round( d ) );
}

template<class T>
int CeilToInt( const T &d ) {
	return int( ceil( d ) );
}

template<class T>
int FloorToInt( const T &d ) {
	return int( floor( d ) );
}

template<class T>
int SignNumber( const T d ) {
	if ( abs( d ) < eps ) {
		return 0;
	} else if ( d > eps ) {
		return 1;
	} else {
		return -1;
	}
}

template<class T>
double CrossProduct( const T &p1, const T &p2 ) {
	return p1.x * p2.y - p1.y * p2.x;
}

template<class T>
double DotProduct( const T &p1, const T &p2 ) {
	return p1.x * p2.x + p1.y * p2.y;
}

void  RestrictInside( Point &p, const Size &size );

void  RestrictInside( Point2f &p, const Size &size ); 

double CalcVec3fDiff( const Vec3f &, const Vec3f & );


#endif