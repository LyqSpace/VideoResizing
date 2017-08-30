#include "common.h"

void  RestrictInside( Point &p, const Size &size ) {
	p.x = max( 0, min( size.width - 1, p.x ) );
	p.y = max( 0, min( size.height - 1, p.y ) );
}

void  RestrictInside( Point2f &p, const Size &size ) {
	p.x = max( 0.0f, min( size.width - 1.0f, p.x ) );
	p.y = max( 0.0f, min( size.height - 1.0f, p.y ) );
}

double CalcVec3fDiff( const Vec3f &p0, const Vec3f &p1 ) {
	double diff = 0;
	for ( int i = 0; i < 3; i++ ) {
		diff += sqr( p0.val[i] - p1.val[i] );
	}
	return sqrt( diff );
}
