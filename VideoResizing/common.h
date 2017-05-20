#ifndef COMMON_H
#define COMMON_H

#define _CRT_SECURE_NO_WARNINGS
#define DEBUG

#include <string>
#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

const double INF = 1e10;
const double eps = 1e-8;
const double resize_rate = 0.6;
const int DIRECTIONS_NUM = 8;
const Point directions[DIRECTIONS_NUM] = {
	Point( 0, 1 ), Point( 1, 0 ), Point( -1, 0 ), Point( 0, -1 ), 
	Point( 1, 1 ), Point( 1, -1 ), Point( -1, 1 ), Point( -1, -1 ) 
};

#define sqr(_x) ((_x) * (_x))

const int QUANTIZE_LEVEL = 5;
const double SIGMA_COLOR = 40;
const double SIGMA_DIST = 200;
const int SALIENCY_SMOOTH_SPAN = 5;
const int MIN_ENERGY_ITERS = 200;

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
	T eleSpan = eleMax - eleMin;
	for ( auto &ele : vec ) {
		ele = (ele - eleMin) / eleSpan;
	}
}

template<class T>
double NormL2( const T &p0, const T &p1 ) {
	return sqrt( sqr( p0.x - p1.x ) + sqr( p0.y - p1.y ) );
}

template<class T>
double NormL2( const T &p ) {
	return sqrt( sqr( p.x ) + sqr( p.y ) );
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

double CalcVec3fDiff( const Vec3f &, const Vec3f & );

string Point2fToString( const Point2f & );

Point2f StringToPoint2f( const string & );

Point Point2fToPoint( const Point2f & );

#endif