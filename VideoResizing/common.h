#ifndef COMMON_H
#define COMMON_H

#define _CRT_SECURE_NO_WARNINGS
#define DEBUG

#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

const double INF = 1e10;
const double eps = 1e-8;
const double resize_rate = 0.6;

#define sqr(_x) ((_x) * (_x))

const int QUANTIZE_LEVEL = 5;
const double SIGMA_COLOR = 40;
const double SIGMA_DIST = 200;

struct TypeColorSpace {
	Point pos;
	float color[3];
	TypeColorSpace() {
		pos = Point( 0, 0 );
		color[0] = color[1] = color[2] = 0;
	}
	TypeColorSpace( Point _pos, Vec3f _color ) {
		pos = _pos;
		for ( int i = 0; i < 3; i++ ) color[i] = _color.val[i];
	}
};

template<class T, size_t N>
bool checkEleExist( const T( &eleArray )[N], const string &eleVal ) {
	if ( find( begin( eleArray ), end( eleArray ), eleVal ) != end( eleArray ) ) {
		return true;
	} else {
		return false;
	}
}

template<class T>
void normalizeVec( vector<T> &vec ) {
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

double CalcVec3fDiff( const Vec3f &, const Vec3f & );

#endif