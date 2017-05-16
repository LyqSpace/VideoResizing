#include "common.h"

double CalcVec3fDiff( const Vec3f &p0, const Vec3f &p1 ) {
	double diff = 0;
	for ( int i = 0; i < 3; i++ ) {
		diff += sqr( p0.val[i] - p1.val[i] );
	}
	return sqrt( diff );
}