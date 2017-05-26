#include "Deformation.h"

Deformation::Deformation( vector<KeyFrame> &_frames ) :frames( _frames ) {
	frameNum = _frames.size();
	frameSize = _frames[0].img.size();

	staticPoints.push_back( Point2f( 0, 0 ) );
	staticPoints.push_back( Point2f( frameSize.width - 1, 0 ) );
	staticPoints.push_back( Point2f( 0, frameSize.height - 1 ) );
	staticPoints.push_back( Point2f( frameSize.width - 1, frameSize.height - 1 ) );

	controlPointsNum = 0;
	controlPoints.clear();
	frameControlPointIndex = vector< vector<int> >( frameNum );

	for ( auto &frame : frames ) {
		frame.FreeMemory();
	}

}

void Deformation::DrawSubdiv( const Mat &_img, const Subdiv2D &subdiv ) {

	Scalar delaunay_color( 255, 255, 255 );
	Mat img = _img.clone();

	vector<Vec4f> edgeList;
	subdiv.getEdgeList( edgeList );
	for ( size_t i = 0; i < edgeList.size(); i++ ) {
		Vec4f e = edgeList[i];
		Point pt0 = Point( cvRound( e[0] ), cvRound( e[1] ) );
		Point pt1 = Point( cvRound( e[2] ), cvRound( e[3] ) );
		line( img, pt0, pt1, delaunay_color, 1, CV_AA, 0 );
	}

	imshow( "SubDiv", img );
	waitKey( 1 );

}

void Deformation::DrawEdge( int frameId, int posType ) {

	Mat img;
	Scalar lineColor( 255, 255, 255 );
	Scalar centerColor( 0, 0, 255 );
	Scalar boundColor( 255, 0, 0 );

	switch ( posType ) {
		case ORIGIN_POS:
			img = Mat::zeros( frameSize, CV_8UC3 );
			break;
		case DEFORMED_POS:
			img = Mat::zeros( deformedFrameSize, CV_8UC3 );
			break;
		case ORIGIN_POS_WITH_FRAME:
			img = frames[frameId].img.clone();
			break;
		case DEFORMED_POS_WITH_FRAME:
			img = deformedFrames[frameId].clone();
			break;
		default:
			break;
	}

	for ( const auto &controlPointIndex : frameControlPointIndex[frameId] ) {

		const ControlPoint &controlPoint = controlPoints[controlPointIndex];

		if ( controlPoint.anchorType != ControlPoint::ANCHOR_CENTER ) continue;

		if ( posType == ORIGIN_POS || posType == ORIGIN_POS_WITH_FRAME ) {
			circle( img, controlPoint.originPos, 3, centerColor, 2, CV_AA );
		} else {
			circle( img, controlPoint.pos, 3, centerColor, 2, CV_AA );
		}

		for ( const auto &boundIndex : controlPoint.spatialBound ) {

			const ControlPoint &boundPoint = controlPoints[boundIndex];

			if ( boundPoint.anchorType == ControlPoint::ANCHOR_CENTER ) {
				cout << controlPoint.originPos << " " << boundPoint.originPos << endl;
			}

			if ( posType == ORIGIN_POS || posType == ORIGIN_POS_WITH_FRAME ) {
				circle( img, boundPoint.originPos, 3, boundColor, 2, CV_AA );
				line( img, controlPoint.originPos, boundPoint.originPos, lineColor, 1, CV_AA );
			} else {
				circle( img, boundPoint.pos, 3, boundColor, 2, CV_AA );
				line( img, controlPoint.pos, boundPoint.pos, lineColor, 1, CV_AA );
			}

		}

	}

	imshow( "Edge", img );
	waitKey( 1 );


}

void Deformation::DrawLocate( const Point2f &deformedPoint, const vector<BaryCoord> &baryCoord ) {

	Mat img = Mat::zeros( frameSize, CV_8UC3 );

	circle( img, deformedPoint, 5, Scalar( 255, 0, 0 ), 2, CV_AA );

	for ( const auto &vertex : baryCoord ) {
		ControlPoint controlPoint = controlPoints[vertex.second];
		circle( img, controlPoint.originPos, 5, Scalar( 0, 0, 128 ), 2, CV_AA );
		circle( img, controlPoint.pos, 5, Scalar( 128, 0, 0 ), 2, CV_AA );
	}

	Point2f originPoint = CalcPointByBaryCoord( baryCoord, ORIGIN_POS );
	circle( img, originPoint, 5, Scalar( 0, 0, 255 ), 2, CV_AA );

	imshow( "Locate", img );
	waitKey( 1 );

}

Point2f Deformation::GetBoundPoint( int index0, int index1 ) {

	ControlPoint p0 = controlPoints[index0];
	ControlPoint p1 = controlPoints[index1];

	if ( p0.saliency < p1.saliency ) {
		swap( p0, p1 );
	}

	int superpixelIndex = p0.superpixelIndex;
	int frameId = p0.frameId;
	Point2f boundPoint( -1, -1 );

	if ( SignNumber( p0.originPos.x - p1.originPos.x ) == 0 ) {

		int x = p0.originPos.x;
		if ( p0.originPos.y < p1.originPos.y ) {
			for ( int y = p0.originPos.y; y < p1.originPos.y; y++ ) {
				if ( frames[frameId].pixelLabel.at<int>( y, x ) != superpixelIndex ) {
					boundPoint = Point( x, y );
					break;
				}
			}
		} else {
			for ( int y = p0.originPos.y; y > p1.originPos.y; y-- ) {
				if ( frames[frameId].pixelLabel.at<int>( y, x ) != superpixelIndex ) {
					boundPoint = Point( x, y );
					break;
				}
			}
		}

	} else {

		int dx = 1;
		double k = (p1.originPos.y - p0.originPos.y) / abs( p0.originPos.x - p1.originPos.x );
		if ( p0.originPos.x > p1.originPos.x ) dx = -1;

		if ( dx > 0 ) {

			for ( int x = p0.originPos.x; x < p1.originPos.x; x += dx ) {

				int y = RoundToInt( p0.originPos.y + k * abs( x - p0.originPos.x ) );
				int tmpY = RoundToInt( p0.originPos.y + k * abs( x + dx - p0.originPos.x ) );

				if ( k > 0 ) {
					while ( y <= tmpY ) {
						if ( frames[frameId].pixelLabel.at<int>( y, x ) != superpixelIndex ) {
							boundPoint = Point( x, y );
							break;
						}
						y++;
					}
					if ( boundPoint.x != -1 ) break;

				} else {
					
					while ( y >= tmpY ) {
						if ( frames[frameId].pixelLabel.at<int>( y, x ) != superpixelIndex ) {
							boundPoint = Point( x, y );
							break;
						}
						y--;
					}
					if ( boundPoint.x != -1 ) break;

				}

			}

		} else {

			for ( int x = p0.originPos.x; x > p1.originPos.x; x += dx ) {

				int y = RoundToInt( p0.originPos.y + k * abs( x - p0.originPos.x ) );
				int tmpY = RoundToInt( p0.originPos.y + k * abs( x + dx - p0.originPos.x ) );
				
				if ( k > 0 ) {
					while ( y <= tmpY ) {
						if ( frames[frameId].pixelLabel.at<int>( y, x ) != superpixelIndex ) {
							boundPoint = Point( x, y );
							break;
						}
						y++;
					}
					if ( boundPoint.x != -1 ) break;

				} else {
					while ( y >= tmpY ) {
						if ( frames[frameId].pixelLabel.at<int>( y, x ) != superpixelIndex ) {
							boundPoint = Point( x, y );
							break;
						}
						y--;
					}
					if ( boundPoint.x != -1 ) break;

				}

			}

		}

	}

	if ( boundPoint == Point2f( -1, -1 ) ) {
		boundPoint = 0.5 * (p0.originPos + p1.originPos);
	}

	return boundPoint;

}

void Deformation::DelaunayDivide() {

	printf( "\tDelaunay divide each key frames.\n" );

	for ( int i = 0; i < frameNum; i++ ) {

		Rect rect( 0, 0, frameSize.width, frameSize.height );
		Subdiv2D subdiv( rect );
		map<string, int> posToPointIndexMap;

		// Add superpixel center points.
		frameControlPointIndex[i] = vector<int>( frames[i].superpixelNum );
		for ( int j = 0; j < frames[i].superpixelNum; j++ ) {
			double saliency = frames[i].superpixelSaliency[j];
			controlPoints.push_back( ControlPoint( i, frames[i].superpixelCenter[j], ControlPoint::ANCHOR_CENTER, j, saliency ) );
			posToPointIndexMap[Point2fToString( frames[i].superpixelCenter[j] )] = controlPointsNum;
			frameControlPointIndex[i][j] = controlPointsNum;

			subdiv.insert( frames[i].superpixelCenter[j] );
			controlPointsNum++;

#ifdef DEBUG_DIVIDE
			DrawSubdiv( frames[i].img, subdiv );
#endif

		}

		// Add superpixel bound points.
		vector<Vec4f> edgeList;
		subdiv.getEdgeList( edgeList );
		map< string, int> edgeExist;

		for ( const auto &e : edgeList ) {

			Point2f p0( e.val[0], e.val[1] );
			Point2f p1( e.val[2], e.val[3] );

			if ( CheckOutside( p0, frameSize ) || CheckOutside( p1, frameSize ) ) {
				continue;
			}

			int index0 = posToPointIndexMap[Point2fToString( p0 )];
			int index1 = posToPointIndexMap[Point2fToString( p1 )];

			string edgeHash0 = to_string( index0 ) + " " + to_string( index1 );
			string edgeHash1 = to_string( index1 ) + " " + to_string( index0 );
			if ( edgeExist.count( edgeHash0 ) > 0 ) continue;
			if ( edgeExist.count( edgeHash1 ) > 0 ) continue;
			edgeExist[edgeHash0] = 1;
			edgeExist[edgeHash1] = 1;

			Point2f boundPoint = GetBoundPoint( index0, index1 );
			controlPoints.push_back( ControlPoint( i, boundPoint, ControlPoint::ANCHOR_BOUND, -1, -1 ) );

			controlPoints[index0].AddSpatialBound( controlPointsNum );
			controlPoints[index1].AddSpatialBound( controlPointsNum );

			controlPointsNum++;

		}

		// Add anchor points.
		for ( const auto &point : staticPoints ) {

			int label = frames[i].pixelLabel.at<int>( point );
			int controlPointIndex = frameControlPointIndex[i][label];

			controlPoints.push_back( ControlPoint( i, point, ControlPoint::ANCHOR_STATIC, label, -1 ) );
			controlPoints[controlPointIndex].AddSpatialBound( controlPointsNum );

			controlPointsNum++;

		}

		// Add image bound points.
		for ( int j = 0; j < frames[i].superpixelNum; j++ ) {

			Point2f center = frames[i].superpixelCenter[j];
			Point pointBound;
			int controlPointIndex = frameControlPointIndex[i][j];

			switch ( frames[i].superpixelBoundLabel[j] ) {

				case KeyFrame::BOUND_LEFT:
					pointBound = Point( 0, center.y );
					controlPoints.push_back( ControlPoint( i, pointBound, ControlPoint::ANCHOR_STATIC, j, -1 ) );
					controlPoints[controlPointIndex].AddSpatialBound( controlPointsNum );
					controlPointsNum++;
					break;

				case KeyFrame::BOUND_TOP:
					pointBound = Point( center.x, 0 );
					controlPoints.push_back( ControlPoint( i, pointBound, ControlPoint::ANCHOR_STATIC, j, -1 ) );
					controlPoints[controlPointIndex].AddSpatialBound( controlPointsNum );
					controlPointsNum++;
					break;

				case KeyFrame::BOUND_RIGHT:
					pointBound = Point( frameSize.width - 1, center.y );
					controlPoints.push_back( ControlPoint( i, pointBound, ControlPoint::ANCHOR_STATIC, j, -1 ) );
					controlPoints[controlPointIndex].AddSpatialBound( controlPointsNum );
					controlPointsNum++;
					break;

				case KeyFrame::BOUND_BOTTOM:
					pointBound = Point( center.x, frameSize.height - 1 );
					controlPoints.push_back( ControlPoint( i, pointBound, ControlPoint::ANCHOR_STATIC, j, -1 ) );
					controlPoints[controlPointIndex].AddSpatialBound( controlPointsNum );
					controlPointsNum++;
					break;

				case KeyFrame::BOUND_NONE:
					break;
				default:
					break;
			}
		}

#ifdef DEBUG_DIVIDE
		DrawEdge( i, ORIGIN_POS_WITH_FRAME );
		frames[i].DrawImgWithContours();
		waitKey( 0 );
#endif

	}

	printf( "\tControl point num: %d.\n", controlPoints.size() );

#ifdef DEBUG
	/*for ( auto &p : controlPoints ) {
		cout << p.frameId << " " << p.pos << endl;
		}*/
#endif

}

void Deformation::CalcBaryCoordLambda( const Point2f &p, const vector<Point2f> &vertices, vector<double> &lambda ) {

	if ( vertices.size() == 3 ) {

		double detT = (vertices[1].y - vertices[2].y) * (vertices[0].x - vertices[2].x) +
			(vertices[2].x - vertices[1].x) * (vertices[0].y - vertices[2].y);

		lambda[0] = ((vertices[1].y - vertices[2].y) * (p.x - vertices[2].x) +
					  (vertices[2].x - vertices[1].x) * (p.y - vertices[2].y)) / detT;
		lambda[1] = ((vertices[2].y - vertices[0].y) * (p.x - vertices[2].x) +
					  (vertices[0].x - vertices[2].x) * (p.y - vertices[2].y)) / detT;
		lambda[2] = 1 - lambda[0] - lambda[1];

	} else if ( vertices.size() == 2 ) {

		double detT = vertices[0].x * vertices[1].y - vertices[0].y * vertices[1].x;

		if ( SignNumber( detT ) == 0 ) {
			if ( SignNumber( vertices[0].x + vertices[1].x ) != 0 ) {
				lambda[0] = (p.x - vertices[1].x) / (vertices[0].x + vertices[1].x);
				lambda[1] = 1 - lambda[0];
			} else if ( SignNumber( vertices[0].y + vertices[1].y ) != 0 ) {
				lambda[0] = (p.y - vertices[1].y) / (vertices[0].y + vertices[1].y);
				lambda[1] = 1 - lambda[0];
			}
		} else {
			lambda[0] = (p.x * vertices[1].y - p.y * vertices[1].x) / detT;
			lambda[1] = (p.y * vertices[0].x - p.x * vertices[0].y) / detT;
		}
	}

}

void Deformation::CalcBaryCoord1( map<string, int> &posToPointIndexMap, const Point2f &p, vector<BaryCoord> &baryCoord ) {
	int controlPointIndex = posToPointIndexMap[Point2fToString( p )];
	baryCoord.push_back( make_pair( 1, controlPointIndex ) );
}

void Deformation::CalcBaryCoord2( Subdiv2D &subdiv, map<string, int> &posToPointIndexMap, int e0, const Point2f &p, vector<BaryCoord> &baryCoord ) {

	Point2f pointOrg, pointDst;
	vector<Point2f> biVertices;

	if ( subdiv.edgeOrg( e0, &pointOrg ) > 0 && subdiv.edgeDst( e0, &pointDst ) > 0 ) {
		biVertices.push_back( pointOrg );
		biVertices.push_back( pointDst );
	} else {
		cout << "[CalcBaryCoord2] Get points error: pointOrg " << subdiv.edgeOrg( e0, &pointOrg ) << ", pointDst " << subdiv.edgeDst( e0, &pointDst ) << endl;
	}

	vector<double> lambda( 2 );

	CalcBaryCoordLambda( p, biVertices, lambda );

	for ( int i = 0; i < 2; i++ ) {
		int vertex = posToPointIndexMap[Point2fToString( biVertices[i] )];
		baryCoord.push_back( make_pair( lambda[i], vertex ) );
	}

#ifdef DEBUG
	//printf( "Bi vertices X: %.3lf = %.3lf * %.3lf + %.3lf * %.3lf\n", nextFramePos.x, lambda[0], biVertices[0].x, lambda[1], biVertices[1].x );
	//printf( "Bi vertices Y: %.3lf = %.3lf * %.3lf + %.3lf * %.3lf\n", nextFramePos.y, lambda[0], biVertices[0].y, lambda[1], biVertices[1].y );
	//double tmpX = 0, tmpY = 0;
	//for ( int i = 0; i < 3; i++ ) {
	//	tmpX += lambda[i] * biVertices[i].x;
	//	tmpY += lambda[i] * biVertices[i].y;
	//}
	//cout << Point2f( tmpX, tmpY ) << endl;
#endif

}

void Deformation::CalcBaryCoord3( Subdiv2D &subdiv, map<string, int> &posToPointIndexMap, int e0, const Point2f &p, vector<BaryCoord> &baryCoord ) {

	vector<Point2f> triVertices;
	int e = e0;

	do {
		Point2f pointOrg, pointDst;
		if ( subdiv.edgeOrg( e, &pointOrg ) > 0 && subdiv.edgeDst( e, &pointDst ) > 0 ) {

			bool vertexExistFlag = false;
			for ( const auto &vertex : triVertices ) {
				if ( vertex == pointOrg ) {
					vertexExistFlag = true;
					break;
				}
			}
			if ( !vertexExistFlag ) {
				triVertices.push_back( pointOrg );
				if ( triVertices.size() >= 3 ) break;
			}

			vertexExistFlag = false;
			for ( const auto &vertex : triVertices ) {
				if ( vertex == pointDst ) {
					vertexExistFlag = true;
					break;
				}
			}
			if ( !vertexExistFlag ) {
				triVertices.push_back( pointDst );
				if ( triVertices.size() >= 3 ) break;
			}

		}

		e = subdiv.getEdge( e, Subdiv2D::NEXT_AROUND_LEFT );

	} while ( e != e0 );

	if ( triVertices.size() != 3 ) {
		printf( "[CalcBaryCoord3] Triangle vertices size is inequal 3.\n" );
		return;
	}

	vector<double> lambda( 3 );

	CalcBaryCoordLambda( p, triVertices, lambda );

#ifdef DEBUG
	/*cout << nextFramePos << endl;
	printf( "%.3lf %.3lf %.3lf\n", lambda[0], lambda[1], lambda[2] );
	cout << triVertices[0] << " " << triVertices[1] << " " << triVertices[2] << endl;
	double tmpX = 0, tmpY = 0;
	for ( int i = 0; i < 3; i++ ) {
	tmpX += lambda[i] * triVertices[i].x;
	tmpY += lambda[i] * triVertices[i].y;
	}
	cout << Point2f( tmpX, tmpY ) << endl;*/
#endif

	for ( int i = 0; i < 3; i++ ) {
		int vertex = posToPointIndexMap[Point2fToString( triVertices[i] )];
		baryCoord.push_back( make_pair( lambda[i], vertex ) );
	}

}

int Deformation::LocatePoint( Subdiv2D &subdiv, map<string, int> &posToPointIndexMap, const Point2f &p, vector<BaryCoord> &baryCoord ) {

//#define DEBUG_LOCATE_POINT

	int e0, vertex, locateStatus;

	baryCoord.clear();
	locateStatus = subdiv.locate( p, e0, vertex );

	switch ( locateStatus ) {
		case CV_PTLOC_INSIDE:
			CalcBaryCoord3( subdiv, posToPointIndexMap, e0, p, baryCoord );
			break;
		case CV_PTLOC_ON_EDGE:
			CalcBaryCoord2( subdiv, posToPointIndexMap, e0, p, baryCoord );
			break;
		case CV_PTLOC_VERTEX:
			CalcBaryCoord1( posToPointIndexMap, p, baryCoord );
			break;
		default:
			break;
	}

#ifdef DEBUG_LOCATE_POINT
	Point2f tmpP0 = CalcPointByBaryCoord( baryCoord, ORIGIN_POS );
	Point2f tmpP1 = CalcPointByBaryCoord( baryCoord, DEFORMED_POS );
	cout << p << " " << tmpP0 << " " << tmpP1 << endl;
	for ( const auto &coord : baryCoord ) {
		cout << coord.first << " " << controlPoints[coord.second].originPos << " " << controlPoints[coord.second].pos << endl;
	}
	cout << endl;
	// DrawLocate( p, baryCoord );
#endif

	return locateStatus;

}

void Deformation::AddTemporalNeighbors() {

	printf( "\tAdd temporal neighbors to control points.\n" );

	int curFramePointIndex = 0;
	int nextFramePointIndex = 0;
	Rect rect( 0, 0, frameSize.width, frameSize.height );
	

	for ( ; nextFramePointIndex < controlPointsNum; nextFramePointIndex++ ) {
		if ( controlPoints[nextFramePointIndex].frameId != 0 ) break;
	}

	for ( int frameIndex = 0; frameIndex < frameNum - 1; frameIndex++ ) {

		Subdiv2D subdiv( rect );
		map<string, int> posToPointIndexMap;

		for ( ; nextFramePointIndex < controlPointsNum; nextFramePointIndex++ ) {
			
			ControlPoint &controlPoint = controlPoints[nextFramePointIndex];
			if ( controlPoint.frameId == frameIndex + 2 ) break;
			subdiv.insert( controlPoint.originPos );
			posToPointIndexMap[ Point2fToString( controlPoint.originPos ) ] = nextFramePointIndex;

		}

		for ( ; curFramePointIndex < controlPointsNum; curFramePointIndex++ ) {
			
			ControlPoint &controlPoint = controlPoints[curFramePointIndex];
			if ( controlPoint.anchorType == ControlPoint::ANCHOR_STATIC ) continue;
			if ( controlPoint.frameId != frameIndex ) break;

			Point2f flow = frames[controlPoint.frameId].forwardFlowMap.at<Point2f>( Point2fToPoint( controlPoint.originPos ) );
			Point2f nextFramePos = controlPoint.originPos + flow;
			controlPoint.flow = flow;

			if ( CheckOutside( nextFramePos, frameSize ) ) continue;

			vector<BaryCoord> baryCoord;
			int locateStatus = LocatePoint( subdiv, posToPointIndexMap, nextFramePos, baryCoord );
			if ( locateStatus == CV_PTLOC_INSIDE || locateStatus == CV_PTLOC_ON_EDGE || locateStatus == CV_PTLOC_VERTEX ) {
				controlPoint.AddTemporalNeighbor( baryCoord );
			}
			
		}

	}
	
}

void Deformation::BuildControlPoints() {

	printf( "Build key frames control points.\n" );

	DelaunayDivide();

	AddTemporalNeighbors();

}

void Deformation::InitDeformation( double _deformedScaleX, double _deformedScaleY ) {

	printf( "Initialize deformation.\n" );

	deformedScaleX = _deformedScaleX;
	deformedScaleY = _deformedScaleY;
	deformedFrameSize = Size( CeilToInt( frameSize.width * deformedScaleX ), CeilToInt( frameSize.height * deformedScaleY ) );

	for ( auto &controlPoint : controlPoints ) {
		controlPoint.pos.x *= deformedScaleX;
		controlPoint.pos.y *= deformedScaleY;
	}

#ifdef DEBUG
	//DrawEdge( ORIGIN_POS );
	//DrawEdge( DEFORMED_POS );
#endif

}

double Deformation::CalcEnergy() {

	double E = 0;

	// Calculate E_L energy term.
	double E_L = 0;

	for ( const auto &centerPoint : controlPoints ) {

		if ( centerPoint.anchorType != ControlPoint::ANCHOR_CENTER ) continue;

		double saliency = centerPoint.saliency;
		double sumDistortion = 0;

		for ( const auto &boundIndex : centerPoint.spatialBound ) {

			ControlPoint &boundPoint = controlPoints[boundIndex];

			double distortion = NormL2( centerPoint.pos - boundPoint.pos, centerPoint.originPos - boundPoint.originPos );
			sumDistortion += distortion;

		}

		E_L += saliency * sumDistortion;

	}

	// Calculate E_D energy term.
	double E_D = 0;

	for ( const auto &centerPoint : controlPoints ) {

		if ( centerPoint.anchorType != ControlPoint::ANCHOR_CENTER ) continue;

		double saliency = centerPoint.saliency;

		double tmpDistortion0 = 0;
		double tmpDistortion1 = 0;
		for ( const auto &boundIndex : centerPoint.spatialBound ) {

			ControlPoint &boundPoint = controlPoints[boundIndex];
			double distortionRate = NormL2( centerPoint.pos - boundPoint.pos, centerPoint.originPos - boundPoint.originPos ) /
				NormL2( centerPoint.originPos - boundPoint.originPos );

			tmpDistortion0 += sqr( distortionRate );
			tmpDistortion1 += distortionRate;

		}

		tmpDistortion0 = tmpDistortion0 / centerPoint.spatialBound.size();
		tmpDistortion1 = sqr( tmpDistortion1 ) / sqr( centerPoint.spatialBound.size() );

		E_D += saliency * (tmpDistortion0 - tmpDistortion1);

	}

	// Calculate E_T energy term.
	double E_T = 0;

	for ( const auto &point : controlPoints ) {

		int nextFrameId = point.frameId + 1;

		if ( nextFrameId == frameNum ) continue;

		Point2f nextFramePointPos = CalcPointByBaryCoord( point.temporalNeighbors, DEFORMED_POS );
		Point2f nextFramePointOriginPos = CalcPointByBaryCoord( point.temporalNeighbors, ORIGIN_POS );

		E_T += NormL2( point.pos - nextFramePointPos, point.originPos - nextFramePointOriginPos );

	}

	E = E_L + E_D + E_T;

	return E;

}

void Deformation::MinimizeEnergy() {

#define DEBUG_ENERGY
	
	printf( "Minimize resize energy.\n" );

	double lambda = 1;
	double curE = CalcEnergy();
	printf( "\tIter 0. Energy: %.3lf. Learning rate: %.3lf.\n", curE, lambda );

	for ( int iter = 0; iter < MIN_ENERGY_ITERS; iter++ ) {

		vector<Point2f> newControlPoints( controlPointsNum );

		for ( int i = 0; i < controlPointsNum; i++ ) {
			newControlPoints[i] = controlPoints[i].pos;
		}

		// Minimize E_L energy term.
		for ( size_t i = 0; i < controlPoints.size(); i++ ) {

			ControlPoint &centerPoint = controlPoints[i];
			if ( centerPoint.anchorType != ControlPoint::ANCHOR_CENTER ) continue;

			double saliency = centerPoint.saliency;
			Point2f sumDelta( 0, 0 );

			for ( const auto &boundIndex : centerPoint.spatialBound ) {

				ControlPoint &boundPoint = controlPoints[boundIndex];

				Point2f mlclr = (centerPoint.pos - boundPoint.pos) - (centerPoint.originPos - boundPoint.originPos);
				double dnmtr = NormL2( centerPoint.pos - boundPoint.pos, centerPoint.originPos - boundPoint.originPos );
				
				if ( SignNumber( dnmtr ) == 0 ) continue;

				Point2f tmp = 1 / dnmtr * mlclr;

#ifdef DEBUG_ENERGY
				//cout << "E_L " << newControlPoints[boundIndex] << " " << saliency * tmp << endl;
#endif

				newControlPoints[boundIndex] -= lambda * saliency * tmp;
				sumDelta += tmp;

			}

#ifdef DEBUG_ENERGY
			//cout << "E_L " << newControlPoints[i] << " " << saliency * sumDelta << endl;
#endif
			newControlPoints[i] -= lambda * saliency * sumDelta;

		}

		// Minimize E_D energy term.
		for ( size_t i = 0; i < controlPoints.size(); i++ ) {

			ControlPoint &centerPoint = controlPoints[i];
			if ( centerPoint.anchorType != ControlPoint::ANCHOR_CENTER ) continue;

			int spatialBoundSize = centerPoint.spatialBound.size();
			double avgDistortion = 0;
			Point2f avgPartialDistortion = 0;

			for ( const auto &boundIndex : centerPoint.spatialBound ) {

				ControlPoint &boundPoint = controlPoints[boundIndex];

				double tmp = NormL2( centerPoint.originPos - boundPoint.originPos );
				if ( SignNumber( tmp ) != 0 ) {
					avgDistortion += NormL2( centerPoint.pos - boundPoint.pos, centerPoint.originPos - boundPoint.originPos ) / tmp;
				}
				tmp = NormL2( centerPoint.originPos - boundPoint.originPos ) * NormL2( centerPoint.pos - boundPoint.pos, centerPoint.originPos - boundPoint.originPos );
				if ( SignNumber( tmp ) != 0 ) {
					avgPartialDistortion += 1 / tmp * ((centerPoint.pos - boundPoint.pos) - (centerPoint.originPos - boundPoint.originPos));
				}

			}

			avgDistortion = 1.0 / spatialBoundSize * avgDistortion;
			avgPartialDistortion = 1.0 / spatialBoundSize * avgPartialDistortion;

#ifdef DEBUG_ENERGY
			cout << "E_D " << avgDistortion << " " << avgPartialDistortion << endl;
#endif

			double saliency = centerPoint.saliency;
			Point2f PiDelta = 0;

			for ( const auto &boundIndex : centerPoint.spatialBound ) {

				ControlPoint &boundPoint = controlPoints[boundIndex];

				double distortion = 0;
				double tmp = NormL2( centerPoint.originPos - boundPoint.originPos );
				Point2f partialDistortion( 0, 0 );

				if ( SignNumber( tmp ) != 0 ) {
					 distortion = NormL2( centerPoint.pos - boundPoint.pos, centerPoint.originPos - boundPoint.originPos ) / tmp;
				}

				tmp = NormL2( centerPoint.originPos - boundPoint.originPos ) * NormL2( centerPoint.pos - boundPoint.pos, centerPoint.originPos - boundPoint.originPos );
				if ( SignNumber( tmp ) != 0 ) {
					partialDistortion = 1 / tmp * ((centerPoint.pos - boundPoint.pos) - (centerPoint.originPos - boundPoint.originPos));
				}

				PiDelta += 2 * distortion * partialDistortion - 2 * distortion * avgPartialDistortion;

				Point2f PjDelta = saliency / spatialBoundSize * (-2 * distortion * partialDistortion + 2 * partialDistortion * avgDistortion);

#ifdef DEBUG_ENERGY
				//cout << "E_D " << newControlPoints[boundIndex] << " " << PjDelta << endl;
#endif
				newControlPoints[boundIndex] -= lambda * PjDelta;

			}

			PiDelta = saliency / spatialBoundSize * PiDelta;

#ifdef DEBUG_ENERGY
			//cout << "E_D " << newControlPoints[i] << " " << PiDelta << endl;
#endif
			newControlPoints[i] -= lambda * PiDelta;

		}


		// Update control points.
		for ( int i = 0; i < controlPointsNum; i++ ) {
			if ( controlPoints[i].anchorType != ControlPoint::ANCHOR_STATIC ) {
#ifdef DEBUG_ENERGY
				cout << controlPoints[i].pos << " " << newControlPoints[i] << endl;
#endif
				controlPoints[i].pos = newControlPoints[i];
				RestrictInside( controlPoints[i].pos, deformedFrameSize );
			}
		}

		double preE = curE;
		curE = CalcEnergy();
		if ( curE >= preE ) {
			lambda /= 2;
		}

		if ( lambda < ITER_TERMINATE ) break;

		printf( "\tIter %d. Energy: %.3lf. Learning rate: %.3lf.\n", iter + 1, curE, lambda );


	}
}


Point2f Deformation::CalcPointByBaryCoord( const vector<BaryCoord> &baryCoord, int posType ) {

	Point2f deformedPoint( 0, 0 );

	for ( const auto &vertex : baryCoord ) {
		// cout << vertex.first << " " << vertex.second << endl;
		ControlPoint point = controlPoints[vertex.second];
		if ( posType == ORIGIN_POS ) {
#ifdef DEBUG
			// cout << vertex.first << " " << point.originPos << " ";
#endif
			deformedPoint += vertex.first * point.originPos;
		} else {
			deformedPoint += vertex.first * point.pos;
		}
	}

	// cout << endl;

	return deformedPoint;

}

void Deformation::CalcDeformedMap() {

//#define DEBUG_CALC_DEFORMED_MAP

	clock_t timeSt = clock();

	deformedMap.clear();
	
	int controlPointIndex = 0;
	Rect rect( 0, 0, deformedFrameSize.width, deformedFrameSize.height );

	for ( int frameIndex = 0; frameIndex < frameNum; frameIndex++ ) {

		deformedMap.push_back( Mat( deformedFrameSize, CV_32FC2 ) );

		printf( "Calculate key frames deformed map. Progress rate %d/%d.\r", frameIndex + 1, frameNum );

		Subdiv2D subdiv( rect );
		map<string, int> posToPointIndexMap;

		for ( ; controlPointIndex < controlPointsNum; controlPointIndex++ ) {
			
			ControlPoint &controlPoint = controlPoints[controlPointIndex];

			if ( controlPoint.frameId != frameIndex ) break;

			subdiv.insert( controlPoint.pos );
			posToPointIndexMap[Point2fToString( controlPoint.pos )] = controlPointIndex;

		}

		for ( int y = 0; y < deformedFrameSize.height; y++ ) {
			for ( int x = 0; x < deformedFrameSize.width; x++ ) {

				vector<BaryCoord> baryCoord;
				Point2f originPoint, deformedPoint;
				
				deformedPoint = Point2f( x, y );
				int locateStatus = LocatePoint( subdiv, posToPointIndexMap, deformedPoint, baryCoord );
				if ( locateStatus == CV_PTLOC_INSIDE || locateStatus == CV_PTLOC_ON_EDGE || locateStatus == CV_PTLOC_VERTEX ) {
					originPoint = CalcPointByBaryCoord( baryCoord, ORIGIN_POS );
					RestrictInside( originPoint, frameSize );
					deformedMap[frameIndex].at<Point2f>( deformedPoint ) = originPoint;
				} else {
					printf( "[CalcDeformedMap] Locate error. " );
					cout << deformedPoint << endl;
				}
				
#ifdef DEBUG_CALC_DEFORMED_MAP
				if ( frameIndex == 0 && y == 80 && x <= 30 ) {
					for ( size_t i = 0; i < baryCoord.size(); i++ ) {
						cout << baryCoord[i].first << " " << controlPoints[baryCoord[i].second].pos << " " << controlPoints[baryCoord[i].second].originPos << endl;
					}
					cout << x << " " << originPoint << " " << deformedPoint << endl << endl;
				}

#endif		

#ifdef DEBUG
				// DrawLocate( deformedPoint, baryCoord );
#endif
			}
		}

	}

	printf( "\n" );
	clock_t timeEd = clock();
	printf( "Calculate key frames deformed map. Time used %ds.\n", (timeEd - timeSt) / 1000 );

}

void Deformation::RenderFrame( const Mat &img, const Mat &deformedMap, Mat &deformedImg ) {

	Size frameSize = img.size();

	deformedImg = Mat::zeros( deformedFrameSize, CV_8UC3 );

	for ( int y = 0; y < deformedFrameSize.height; y++ ) {
		for ( int x = 0; x < deformedFrameSize.width; x++ ) {

			Point2f originPoint, deformedPoint;
			deformedPoint = Point2f( x, y );
			originPoint = deformedMap.at<Point2f>( deformedPoint );

			deformedImg.at<Vec3b>( deformedPoint ) = img.at<Vec3b>( originPoint );

		}
	}

	imshow( "Deformed Frame", deformedImg );
	waitKey( 0 );

}

void Deformation::RenderKeyFrames() {

	printf( "Render key frames.\n" );

	for ( int i = 0; i < frameNum; i++ ) {

		Mat deformedFrame;

		RenderFrame( frames[i].img, deformedMap[i], deformedFrame );

		deformedFrames.push_back( deformedFrame );

		DrawEdge( i, DEFORMED_POS_WITH_FRAME );

	}

}