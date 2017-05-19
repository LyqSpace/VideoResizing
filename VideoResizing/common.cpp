#include "common.h"

double CalcVec3fDiff( const Vec3f &p0, const Vec3f &p1 ) {
	double diff = 0;
	for ( int i = 0; i < 3; i++ ) {
		diff += sqr( p0.val[i] - p1.val[i] );
	}
	return sqrt( diff );
}

string Point2fToString( const Point2f &p ) {
	return to_string( int( p.x * 1000 ) ) + " " + to_string( int( p.y * 1000 ) );
}

Point2f StringToPoint2f( const string &str ) {
	int split = str.find( " " );
	float x = atof( str.substr( 0, split ).c_str() );
	float y = atof( str.substr( split ).c_str() );
	return Point2f( x / 1000.0, y / 1000.0 );
}

Point Point2fToPoint( const Point2f &p ) {
	return Point( RoundToInt( p.x ), RoundToInt( p.y ) );
}