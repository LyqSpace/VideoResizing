#include "Deformation.h"

Deformation::Deformation( vector<KeyFrame> &_frames, const string &_videoName ) :frames( _frames ) {

	videoName = _videoName;
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

void Deformation::DrawEdge( int frameId, int posType, Mat &_img ) {

	Mat img;
	Scalar lineColor( 255, 255, 255 );
	Scalar centerColor( 0, 0, 255 );
	Scalar boundColor( 255, 0, 0 );
	Scalar staticColor( 0, 255, 0 );

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

		for ( const auto &neighborIndex : controlPoint.shapeNeighbors ) {

			const ControlPoint &neighborPoint = controlPoints[neighborIndex];

			if ( neighborPoint.anchorType == ControlPoint::ANCHOR_CENTER ) {
				cout << controlPoint.originPos << " " << neighborPoint.originPos << endl;
			}

			if ( posType == ORIGIN_POS || posType == ORIGIN_POS_WITH_FRAME ) {

				line( img, controlPoint.originPos, neighborPoint.originPos, lineColor, 1, CV_AA );
				if ( neighborPoint.anchorType == ControlPoint::ANCHOR_BOUND ) {
					circle( img, neighborPoint.originPos, 3, boundColor, 2, CV_AA );
				} else {
					circle( img, neighborPoint.originPos, 3, staticColor, 2, CV_AA );
				}
				
			} else {

				line( img, controlPoint.pos, neighborPoint.pos, lineColor, 1, CV_AA );
				if ( neighborPoint.anchorType == ControlPoint::ANCHOR_BOUND ) {
					circle( img, neighborPoint.pos, 3, boundColor, 2, CV_AA );
				} else {
					circle( img, neighborPoint.pos, 3, staticColor, 2, CV_AA );
				}
				
			}

		}

	}

	_img = img.clone();

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
	Point2f neighborPoint( -1, -1 );

	if ( SignNumber( p0.originPos.x - p1.originPos.x ) == 0 ) {

		int x = p0.originPos.x;
		if ( p0.originPos.y < p1.originPos.y ) {
			for ( int y = p0.originPos.y; y < p1.originPos.y; y++ ) {
				if ( frames[frameId].pixelLabel.at<int>( y, x ) != superpixelIndex ) {
					neighborPoint = Point( x, y );
					break;
				}
			}
		} else {
			for ( int y = p0.originPos.y; y > p1.originPos.y; y-- ) {
				if ( frames[frameId].pixelLabel.at<int>( y, x ) != superpixelIndex ) {
					neighborPoint = Point( x, y );
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
							neighborPoint = Point( x, y );
							break;
						}
						y++;
					}
					if ( neighborPoint.x != -1 ) break;

				} else {
					
					while ( y >= tmpY ) {
						if ( frames[frameId].pixelLabel.at<int>( y, x ) != superpixelIndex ) {
							neighborPoint = Point( x, y );
							break;
						}
						y--;
					}
					if ( neighborPoint.x != -1 ) break;

				}

			}

		} else {

			for ( int x = p0.originPos.x; x > p1.originPos.x; x += dx ) {

				int y = RoundToInt( p0.originPos.y + k * abs( x - p0.originPos.x ) );
				int tmpY = RoundToInt( p0.originPos.y + k * abs( x + dx - p0.originPos.x ) );
				
				if ( k > 0 ) {
					while ( y <= tmpY ) {
						if ( frames[frameId].pixelLabel.at<int>( y, x ) != superpixelIndex ) {
							neighborPoint = Point( x, y );
							break;
						}
						y++;
					}
					if ( neighborPoint.x != -1 ) break;

				} else {
					while ( y >= tmpY ) {
						if ( frames[frameId].pixelLabel.at<int>( y, x ) != superpixelIndex ) {
							neighborPoint = Point( x, y );
							break;
						}
						y--;
					}
					if ( neighborPoint.x != -1 ) break;

				}

			}

		}

	}

	if ( neighborPoint == Point2f( -1, -1 ) ) {
		neighborPoint = 0.5 * (p0.originPos + p1.originPos);
	}

	return neighborPoint;

}

void Deformation::DelaunayDivide() {

#define DEBUG_DELAUNAY_DIVIDE

	printf( "\tDelaunay divide each key frames.\n" );

	for ( int i = 0; i < frameNum; i++ ) {

		Rect rect( 0, 0, frameSize.width, frameSize.height );
		Subdiv2D subdiv( rect );
		map<string, int> posToPointIndexMap;
		double superpixelMaxDist = 1.8 * sqrt( frameSize.width * frameSize.height / (double)frames[i].superpixelNum );

		// Add superpixel center points.
		frameControlPointIndex[i] = vector<int>( frames[i].superpixelNum );
		for ( int j = 0; j < frames[i].superpixelNum; j++ ) {
			double saliency = frames[i].superpixelSaliency[j];
			controlPoints.push_back( ControlPoint( i, frames[i].superpixelCenter[j], ControlPoint::ANCHOR_CENTER, j, saliency ) );
			posToPointIndexMap[Point2fToString( frames[i].superpixelCenter[j] )] = controlPointsNum;
			frameControlPointIndex[i][j] = controlPointsNum;

			subdiv.insert( frames[i].superpixelCenter[j] );
			controlPointsNum++;

#ifdef DEBUG_DELAUNAY_DIVIDE
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

			if ( frames[i].superpixelBoundLabel[controlPoints[index0].superpixelIndex] != KeyFrame::BOUND_NONE &&
				 frames[i].superpixelBoundLabel[controlPoints[index1].superpixelIndex] != KeyFrame::BOUND_NONE ) {
				double superpixelDist = NormL2( p0, p1 );
				if (superpixelDist > superpixelMaxDist ) continue;
			}

			Point2f neighborPoint = GetBoundPoint( index0, index1 );
			controlPoints.push_back( ControlPoint( i, neighborPoint, ControlPoint::ANCHOR_BOUND, -1, -1 ) );

			controlPoints[index0].AddShapeNeighbor( controlPointsNum );
			controlPoints[index1].AddShapeNeighbor( controlPointsNum );

			controlPoints[controlPointsNum].AddShapeNeighbor( index0 );
			controlPoints[controlPointsNum].AddShapeNeighbor( index1 );

			controlPoints[index0].AddStructureNeighbor( index1 );
			controlPoints[index1].AddStructureNeighbor( index0 );

			structureEdges.push_back( make_pair( index0, index1 ) );

			controlPointsNum++;

		}

		// Add anchor points.
		for ( const auto &point : staticPoints ) {

			int label = frames[i].pixelLabel.at<int>( point );
			int controlPointIndex = frameControlPointIndex[i][label];

			controlPoints.push_back( ControlPoint( i, point, ControlPoint::ANCHOR_STATIC, label, -1 ) );
			controlPoints[controlPointIndex].AddShapeNeighbor( controlPointsNum );
			controlPoints[controlPointsNum].AddShapeNeighbor( controlPointIndex );

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
					controlPoints.push_back( ControlPoint( i, pointBound, ControlPoint::ANCHOR_STATIC_LEFT, j, -1 ) );
					controlPoints[controlPointIndex].AddShapeNeighbor( controlPointsNum );
					controlPoints[controlPointsNum].AddShapeNeighbor( controlPointIndex );
					controlPointsNum++;
					break;

				case KeyFrame::BOUND_TOP:
					pointBound = Point( center.x, 0 );
					controlPoints.push_back( ControlPoint( i, pointBound, ControlPoint::ANCHOR_STATIC_TOP, j, -1 ) );
					controlPoints[controlPointIndex].AddShapeNeighbor( controlPointsNum );
					controlPoints[controlPointsNum].AddShapeNeighbor( controlPointIndex );
					controlPointsNum++;
					break;

				case KeyFrame::BOUND_RIGHT:
					pointBound = Point( frameSize.width - 1, center.y );
					controlPoints.push_back( ControlPoint( i, pointBound, ControlPoint::ANCHOR_STATIC_RIGHT, j, -1 ) );
					controlPoints[controlPointIndex].AddShapeNeighbor( controlPointsNum );
					controlPoints[controlPointsNum].AddShapeNeighbor( controlPointIndex );
					controlPointsNum++;
					break;

				case KeyFrame::BOUND_BOTTOM:
					pointBound = Point( center.x, frameSize.height - 1 );
					controlPoints.push_back( ControlPoint( i, pointBound, ControlPoint::ANCHOR_STATIC_BOTTOM, j, -1 ) );
					controlPoints[controlPointIndex].AddShapeNeighbor( controlPointsNum );
					controlPoints[controlPointsNum].AddShapeNeighbor( controlPointIndex );
					controlPointsNum++;
					break;

				case KeyFrame::BOUND_NONE:
					break;
				default:
					break;
			}
		}

#ifdef DEBUG_DELAUNAY_DIVIDE
		Mat img;
		DrawEdge( i, ORIGIN_POS_WITH_FRAME, img );
		imshow( "Edge", img );
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
			if ( controlPoint.anchorType != ControlPoint::ANCHOR_CENTER && controlPoint.anchorType != ControlPoint::ANCHOR_BOUND ) continue;
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

double Deformation::CalcEnergyStructureL() {

// #define DEBUG_CALC_ENERGY_STRUCTURE_L

	double energy = 0;

	for ( const auto &edge : structureEdges ) {

		ControlPoint &p0 = controlPoints[edge.first];
		ControlPoint &p1 = controlPoints[edge.second];

		double saliency = max( p0.saliency, p1.saliency );
		double distortion = NormL2( p0.pos - p1.pos, p0.originPos - p1.originPos );

		energy += saliency * distortion;

#ifdef DEBUG_CALC_ENERGY_STRUCTURE_L
		if ( p0.frameId == 0 ) {
			cout << "Calc Structure L " << p0.pos << " " << p1.pos << " " << saliency << " " << distortion << endl;
		}
#endif

	}

	return energy;

}

double Deformation::CalcEnergyStructureD() {

//#define DEBUG_CALC_ENERGY_STRUCTURE_D

	double energy = 0;

	for ( const auto &centerPoint : controlPoints ) {

		if ( centerPoint.anchorType != ControlPoint::ANCHOR_CENTER ) continue;

		double avgDistortion0 = 0;
		double avgDistortion1 = 0;
		for ( const auto &neighborIndex : centerPoint.structureNeighbors ) {

			ControlPoint &neighborPoint = controlPoints[neighborIndex];
			double distortionRate = NormL2( centerPoint.pos - neighborPoint.pos ) / NormL2( centerPoint.originPos - neighborPoint.originPos );
			
			if ( SignNumber( NormL2( centerPoint.originPos - neighborPoint.originPos ) ) == 0 ) continue;

			double saliency = max( centerPoint.saliency, neighborPoint.saliency );
			double weight = 1 - saliency;
			distortionRate *= weight;

			avgDistortion0 += sqr( distortionRate );
			avgDistortion1 += distortionRate;

#ifdef DEBUG_CALC_ENERGY_STRUCTURE_D
			if ( centerPoint.frameId == 0 ) {
				cout << "Calc Structure D\t" << centerPoint.pos << " " << neighborPoint.pos << " " << distortionRate << endl;
			}
#endif

		}

		avgDistortion0 = avgDistortion0 / centerPoint.structureNeighbors.size();
		avgDistortion1 = sqr( avgDistortion1 / centerPoint.structureNeighbors.size() );

#ifdef DEBUG_CALC_ENERGY_STRUCTURE_D
		if ( centerPoint.frameId == 0 ) {
			cout << "Calc Structure D " << centerPoint.pos << " " << avgDistortion0 - avgDistortion1 << endl;
		}
		
#endif

		energy += avgDistortion0 - avgDistortion1;

	}

	return energy;

}

double Deformation::CalcEnergyShapeL() {

//#define DEBUG_CALC_ENERGY_SHAPE_L

	double energy = 0;

	for ( const auto &centerPoint : controlPoints ) {

		if ( centerPoint.anchorType != ControlPoint::ANCHOR_CENTER ) continue;

		double saliency = centerPoint.saliency;
		double sumDistortion = 0;

		for ( const auto &neighborIndex : centerPoint.shapeNeighbors ) {

			ControlPoint &neighborPoint = controlPoints[neighborIndex];

			double distortion = NormL2( centerPoint.pos - neighborPoint.pos, centerPoint.originPos - neighborPoint.originPos );
			sumDistortion += distortion;

#ifdef DEBUG_CALC_ENERGY_SHAPE_L
			if (centerPoint.frameId == 0 )
			cout << "CALC E_L " << centerPoint.pos << " " << neighborPoint.pos << " " << saliency << " " << saliency * distortion << endl;
#endif

		}

		energy += saliency * sumDistortion;

	}

	return energy;

}

double Deformation::CalcEnergyShapeD() {

// #define DEBUG_CALC_ENERGY_SHAPE_D

	double energy = 0;

	for ( const auto &centerPoint : controlPoints ) {

		if ( centerPoint.anchorType != ControlPoint::ANCHOR_CENTER ) continue;

		double avgDistortion0 = 0;
		double avgDistortion1 = 0;
		for ( const auto &neighborIndex : centerPoint.shapeNeighbors ) {

			ControlPoint &neighborPoint = controlPoints[neighborIndex];
			double distortionRate = NormL2( centerPoint.pos - neighborPoint.pos ) / NormL2( centerPoint.originPos - neighborPoint.originPos );

			if ( SignNumber( NormL2( centerPoint.originPos - neighborPoint.originPos ) ) == 0 ) continue;

			avgDistortion0 += sqr( distortionRate );
			avgDistortion1 += distortionRate;

#ifdef DEBUG_CALC_ENERGY_SHAPE_D
			//if (centerPoint.frameId == 0 )
			//cout << "\t" << centerPoint.pos << " " << neighborPoint.pos << " " << distortionRate << endl;
#endif

		}

		avgDistortion0 = avgDistortion0 / centerPoint.shapeNeighbors.size();
		avgDistortion1 = sqr( avgDistortion1 / centerPoint.shapeNeighbors.size() );

#ifdef DEBUG_CALC_ENERGY_SHAPE_D
		//if (centerPoint.frameId == 0 )
		//cout << centerPoint.pos << " " << avgDistortion0 - avgDistortion1 << endl;
#endif

		double saliency = centerPoint.saliency;

		energy += saliency * (avgDistortion0 - avgDistortion1);

	}
	
	return energy;

}

double Deformation::CalcEnergyTemporal() {

//#define DEBUG_CALC_ENERGY_TEMPORAL

	double energy = 0;

	for ( const auto &point : controlPoints ) {

		if ( point.frameId + 1 == frameNum ) continue;
		if ( point.anchorType != ControlPoint::ANCHOR_CENTER && point.anchorType != ControlPoint::ANCHOR_BOUND ) continue;

		Point2f nextFramePos = CalcPointByBaryCoord( point.temporalNeighbors, DEFORMED_POS );

		energy += NormL2( nextFramePos - point.pos, point.flow );

#ifdef DEBUG_CALC_ENERGY_TEMPORAL
		if ( point.frameId == 0 ) {
			cout << "Calc Energy Temporal pos " << point.pos << " next pos " << nextFramePos << " flow " << nextFramePos - point.pos << " origin flow " << point.flow << " energy " << NormL2( nextFramePos - point.pos, point.flow) << endl;
		}
#endif

	}

	energy = 0;

	return energy;

}

double Deformation::CalcEnergy() {

#define DEBUG_CALC_ENERGY

	double energyStructureL = CalcEnergyStructureL();
	double energyStructureD = CalcEnergyStructureD();
	double energyShapeL = CalcEnergyShapeL();
	double energyShapeD = CalcEnergyShapeD();
	double energyTemporal = CalcEnergyTemporal();

#ifdef DEBUG_CALC_ENERGY
	printf( "StructureL %.3lf, StructureD %.3lf, ShapeL %.3lf, ShapeD %.3lf, Temporal %.3lf\n", energyStructureL, energyStructureD, energyShapeL, energyShapeD, energyTemporal );
#endif
	double energy = ALPHA_STRUCTURE_L * energyStructureL + ALPHA_STRUCTURE_D * energyStructureD + ALPHA_SHAPE_L * energyShapeL + ALPHA_SHAPE_D * energyShapeD + ALPHA_TEMPORAL * energyTemporal;

	return energy;

}

void Deformation::MinEnergyStructureL( vector<Point2f> &newControlPoints, double lambda ) {

//#define DEBUG_MIN_ENERGY_STRUCTURE_L

	for ( const auto &edge : structureEdges ) {

		ControlPoint &p0 = controlPoints[edge.first];
		ControlPoint &p1 = controlPoints[edge.second];

		double saliency = max( p0.saliency, p1.saliency );
		Point2f mlclr = (p0.pos - p1.pos) - (p0.originPos - p1.originPos);
		double dnmtr = NormL2( p0.pos - p1.pos, p0.originPos - p1.originPos );

		if ( SignNumber( dnmtr ) == 0 ) continue;

		Point2f deltaP0 = lambda * ALPHA_STRUCTURE_L * saliency * 1.0 / dnmtr * mlclr;
		Point2f deltaP1 = -deltaP0;

		newControlPoints[edge.first] -= deltaP0;
		newControlPoints[edge.second] -= deltaP1;

#ifdef DEBUG_MIN_ENERGY_STRUCTURE_L
		if ( p0.frameId == 0 ) {
			cout << saliency << " " << p0.pos << p1.pos << " " << deltaP0 << deltaP1 << endl;
		}
#endif

	}

}

void Deformation::MinEnergyStructureD( vector<Point2f> &newControlPoints, double lambda ) {

//#define DEBUG_MIN_ENERGY_STRUCTURE_D

	for ( size_t i = 0; i < controlPoints.size(); i++ ) {

		ControlPoint &centerPoint = controlPoints[i];
		if ( centerPoint.anchorType != ControlPoint::ANCHOR_CENTER ) continue;

		int structureNeighborsSize = centerPoint.structureNeighbors.size();
		double avgDistortion = 0;
		Point2f avgPartialDistortion = 0;

		for ( const auto &neighborIndex : centerPoint.structureNeighbors ) {

			ControlPoint &neighborPoint = controlPoints[neighborIndex];
			double weight = 1 - max( centerPoint.saliency, neighborPoint.saliency );

			double dnmtr = NormL2( centerPoint.originPos - neighborPoint.originPos );
			if ( SignNumber( dnmtr ) != 0 ) {
				avgDistortion += weight * NormL2( centerPoint.pos - neighborPoint.pos ) / dnmtr;
			}
			dnmtr = NormL2( centerPoint.originPos - neighborPoint.originPos ) * NormL2( centerPoint.pos - neighborPoint.pos );
			if ( SignNumber( dnmtr ) != 0 ) {
				avgPartialDistortion += weight * 1 / dnmtr * (centerPoint.pos - neighborPoint.pos);
			}

		}

		avgDistortion = 1.0 / structureNeighborsSize * avgDistortion;
		avgPartialDistortion = 1.0 / structureNeighborsSize * avgPartialDistortion;

#ifdef DEBUG_MIN_ENERGY_STRUCTURE_D
		if ( centerPoint.frameId == 0 ) {
			cout << "STRUCTURE_D avg distortion " << avgDistortion << ", avg partial distortion " << avgPartialDistortion << endl;
		}
		
#endif

		Point2f PiDelta = 0;

		for ( const auto &neighborIndex : centerPoint.structureNeighbors ) {

			ControlPoint &neighborPoint = controlPoints[neighborIndex];
			double weight = 1 - max( centerPoint.saliency, neighborPoint.saliency );
			double distortion = 0;
			Point2f partialDistortion( 0, 0 );

			double dnmtr = NormL2( centerPoint.originPos - neighborPoint.originPos );
			if ( SignNumber( dnmtr ) != 0 ) {
				distortion = weight * NormL2( centerPoint.pos - neighborPoint.pos ) / dnmtr;
			}

#ifdef DEBUG_MIN_ENERGY_STRUCTURE_D
			cout << dnmtr << " " << distortion << " ";
#endif

			dnmtr = NormL2( centerPoint.originPos - neighborPoint.originPos ) * NormL2( centerPoint.pos - neighborPoint.pos );
			if ( SignNumber( dnmtr ) != 0 ) {
				partialDistortion = weight * 1 / dnmtr * (centerPoint.pos - neighborPoint.pos);
			}

#ifdef DEBUG_MIN_ENERGY_STRUCTURE_D
			cout << dnmtr << " " << partialDistortion << endl;
#endif

			PiDelta += lambda * ALPHA_STRUCTURE_D * 1.0 / structureNeighborsSize * 2 * distortion * partialDistortion;

			Point2f PjDelta = lambda * ALPHA_STRUCTURE_D / structureNeighborsSize * (-2 * distortion * partialDistortion + 2 * partialDistortion * avgDistortion);

#ifdef DEBUG_MIN_ENERGY_STRUCTURE_D
			if ( centerPoint.frameId == 0 ) {
				cout << "STRUCTURE_D bound point " << newControlPoints[neighborIndex] << " weight " << weight << " delta " << PjDelta << endl;
			}
#endif
			newControlPoints[neighborIndex] -= PjDelta;

		}

		PiDelta -= lambda * ALPHA_STRUCTURE_D * 2 * avgDistortion * avgPartialDistortion;

#ifdef DEBUG_MIN_ENERGY_STRUCTURE_D
		if ( centerPoint.frameId == 0 ) {
			cout << "STRUCTURE_D center point " << newControlPoints[i] << " " << PiDelta << endl << endl;
		}
#endif
		newControlPoints[i] -= PiDelta;

	}

}

void Deformation::MinEnergyShapeL( vector<Point2f> &newControlPoints, double lambda ) {

//#define DEBUG_MIN_ENERGY_SHAPE_L

	for ( size_t i = 0; i < controlPoints.size(); i++ ) {

		ControlPoint &centerPoint = controlPoints[i];
		if ( centerPoint.anchorType != ControlPoint::ANCHOR_CENTER ) continue;

		double saliency = centerPoint.saliency;
		Point2f sumDelta( 0, 0 );

		for ( const auto &neighborIndex : centerPoint.shapeNeighbors ) {

			ControlPoint &neighborPoint = controlPoints[neighborIndex];

			Point2f mlclr = (centerPoint.pos - neighborPoint.pos) - (centerPoint.originPos - neighborPoint.originPos);
			double dnmtr = NormL2( centerPoint.pos - neighborPoint.pos, centerPoint.originPos - neighborPoint.originPos );

			if ( SignNumber( dnmtr ) == 0 ) continue;

			Point2f tmp = 1 / dnmtr * mlclr;

#ifdef DEBUG_MIN_ENERGY_SHAPE_L
			if ( centerPoint.frameId == 0 ) {
				cout << "E_L Diff " << centerPoint.pos - neighborPoint.pos << " " << centerPoint.originPos - neighborPoint.originPos << " " << centerPoint.pos - neighborPoint.pos - (centerPoint.originPos - neighborPoint.originPos) << endl;
				cout << "E_L Bound " << newControlPoints[neighborIndex] << " " << tmp << " " << -lambda * ALPHA_SHAPE_L * saliency * tmp << endl;
			}
#endif
			//if ( centerPoint.frameId == 0 )
			//cout << "BEFORE " << newControlPoints[neighborIndex] << endl;
			if ( neighborPoint.anchorType == ControlPoint::ANCHOR_BOUND ) {
				newControlPoints[neighborIndex] -= -lambda * ALPHA_SHAPE_L * saliency * tmp;
			} else {
				sumDelta += lambda * ALPHA_SHAPE_L * saliency * tmp;
			}
			
			//if ( centerPoint.frameId == 0 )
			//cout << "AFTER " << newControlPoints[neighborIndex] << endl;
			sumDelta += lambda * ALPHA_SHAPE_L * saliency * tmp;

		}

#ifdef DEBUG_MIN_ENERGY_SHAPE_L
		/*if ( centerPoint.frameId == 0 ) {
			cout << "E_L Center " << newControlPoints[i] << " " << lambda * ALPHA_SHAPE_L * saliency * sumDelta << endl << endl;
		}*/
#endif
		newControlPoints[i] -= sumDelta;

	}
}

void Deformation::MinEnergyShapeD( vector<Point2f> &newControlPoints, double lambda ) {

// #define DEBUG_MIN_ENERGY_SHAPE_D

	for ( size_t i = 0; i < controlPoints.size(); i++ ) {

		ControlPoint &centerPoint = controlPoints[i];
		if ( centerPoint.anchorType != ControlPoint::ANCHOR_CENTER ) continue;

		int spatialBoundSize = centerPoint.shapeNeighbors.size();
		double avgDistortion = 0;
		Point2f avgPartialDistortion = 0;

		for ( const auto &neighborIndex : centerPoint.shapeNeighbors ) {

			ControlPoint &neighborPoint = controlPoints[neighborIndex];

			double dnmtr = NormL2( centerPoint.originPos - neighborPoint.originPos );
			if ( SignNumber( dnmtr ) != 0 ) {
				avgDistortion += NormL2( centerPoint.pos - neighborPoint.pos ) / dnmtr;
			}
			dnmtr = NormL2( centerPoint.originPos - neighborPoint.originPos ) * NormL2( centerPoint.pos - neighborPoint.pos );
			if ( SignNumber( dnmtr ) != 0 ) {
				avgPartialDistortion += 1 / dnmtr * (centerPoint.pos - neighborPoint.pos);
			}

		}

		avgDistortion = 1.0 / spatialBoundSize * avgDistortion;
		avgPartialDistortion = 1.0 / spatialBoundSize * avgPartialDistortion;
		double saliency = centerPoint.saliency;

#ifdef DEBUG_MIN_ENERGY_SHAPE_D
		//if ( centerPoint.frameId == 0 )
		//cout << "E_D avg distortion" << avgDistortion << ", avg partial distortion " << avgPartialDistortion << endl;
#endif

		Point2f PiDelta = 0;

		for ( const auto &neighborIndex : centerPoint.shapeNeighbors ) {

			ControlPoint &neighborPoint = controlPoints[neighborIndex];

			double distortion = 0;
			Point2f partialDistortion( 0, 0 );

			double dnmtr = NormL2( centerPoint.originPos - neighborPoint.originPos );
			if ( SignNumber( dnmtr ) != 0 ) {
				distortion = NormL2( centerPoint.pos - neighborPoint.pos ) / dnmtr;
			}

			dnmtr = NormL2( centerPoint.originPos - neighborPoint.originPos ) * NormL2( centerPoint.pos - neighborPoint.pos );
			if ( SignNumber( dnmtr ) != 0 ) {
				partialDistortion = 1 / dnmtr * (centerPoint.pos - neighborPoint.pos);
			}

			PiDelta += lambda * ALPHA_SHAPE_D * saliency * 1.0 / spatialBoundSize * 2 * distortion * partialDistortion;

			Point2f PjDelta = lambda * ALPHA_SHAPE_D * saliency / spatialBoundSize * (-2 * distortion * partialDistortion + 2 * partialDistortion * avgDistortion);

#ifdef DEBUG_MIN_ENERGY_SHAPE_D
			//if ( centerPoint.frameId == 0 )
			//cout << "E_D bound point " << newControlPoints[neighborIndex] << " " << PjDelta << endl;
#endif
			if ( neighborPoint.anchorType == ControlPoint::ANCHOR_BOUND ) {
				newControlPoints[neighborIndex] -= PjDelta;
			} else {
				PiDelta += -PjDelta;
			}

		}

		PiDelta -= lambda * ALPHA_SHAPE_D * saliency * 2 * avgDistortion * avgPartialDistortion;

#ifdef DEBUG_MIN_ENERGY_SHAPE_D
		//if (centerPoint.frameId == 0)
		//cout << "E_D center point " << newControlPoints[i] << " " << PiDelta << endl;
#endif
		newControlPoints[i] -= PiDelta;

	}

}

void Deformation::MinEnergyTemporal( vector<Point2f> &newControlPoints, double lambda ) {

//#define DEBUG_MIN_ENERGY_TEMPORAL

	for ( int i = 0; i < controlPointsNum; i++ ) {

		ControlPoint &point = controlPoints[i];
		if ( point.frameId + 1 == frameNum ) continue;
		if ( point.anchorType != ControlPoint::ANCHOR_CENTER && point.anchorType != ControlPoint::ANCHOR_BOUND ) continue;

		Point2f nextFramePos = CalcPointByBaryCoord( point.temporalNeighbors, DEFORMED_POS );

		double dnmtr = NormL2( nextFramePos - point.pos, point.flow );
		Point2f mlclr = nextFramePos - point.pos - point.flow;

		if ( SignNumber( dnmtr ) == 0 ) continue;

		for ( const auto &vertex : point.temporalNeighbors ) {

			Point2f delta = lambda * ALPHA_TEMPORAL * vertex.first * 1.0 / dnmtr * mlclr;
			newControlPoints[vertex.second] -= delta;

#ifdef DEBUG_MIN_ENERGY_TEMPORAL
			if ( point.frameId == 0 ) {
				cout << "Minimize Energy Temporal \tnext pos " << controlPoints[vertex.second].pos << " delta " << delta << endl;
			}
#endif

		}

		Point2f delta = -lambda * ALPHA_TEMPORAL * 1.0 / dnmtr * mlclr;
		newControlPoints[i] -= delta;

#ifdef DEBUG_MIN_ENERGY_TEMPORAL
		if ( point.frameId == 0 ) {
			cout << "Flow " << nextFramePos - point.pos << " Origin Flow " << point.flow << endl;
			cout << "Minimize Energy Temporal cur pos " << point.pos << " delta " << delta << endl;
		}
#endif

	}

}

void Deformation::CollinearConstraint( vector<Point2f> &newControlPoints ) {

// #define DEBUG_COLLINEAR

	for ( size_t i = 0; i < controlPoints.size(); i++ ) {

		if ( controlPoints[i].anchorType != ControlPoint::ANCHOR_BOUND ) continue;
		
		Point2f p0 = newControlPoints[i];
		Point2f p1 = newControlPoints[controlPoints[i].shapeNeighbors[0]];
		Point2f p2 = newControlPoints[controlPoints[i].shapeNeighbors[1]];

		Point2f u1 = p0 - p1;
		Point2f u2 = p2 - p1;

		double norm = NormL2( u2 );
		if ( SignNumber( norm ) == 0 ) {
			newControlPoints[i] = p1;
			continue;
		} 

		double projection = DotProduct( u1, u2 ) / sqr( norm );
		
		projection = max( min( projection, 1.0 ), 0.0 );
		newControlPoints[i] = p1 + projection * u2;
	
	}

}

void Deformation::UpdateControlPoints( const vector<Point2f> &newControlPoints ) {
	
//#define DEBUG_MIN_ENERGY_UPDATE
	
	for ( int i = 0; i < controlPointsNum; i++ ) {

		if ( controlPoints[i].anchorType != ControlPoint::ANCHOR_STATIC ) {
#ifdef DEBUG_MIN_ENERGY_UPDATE
			if ( controlPoints[i].frameId == 0 )
			cout << controlPoints[i].pos << " " << newControlPoints[i] << endl;
#endif
			controlPoints[i].pos = newControlPoints[i];

			Point2f hostPos( -1, -1 );
			if ( controlPoints[i].shapeNeighbors.size() > 0 ) {
				int hostPointIndex = controlPoints[i].shapeNeighbors[0];
				hostPos = controlPoints[hostPointIndex].pos;
			}

			switch ( controlPoints[i].anchorType ) {
				case ControlPoint::ANCHOR_STATIC_LEFT:
					controlPoints[i].pos.x = 0;
					controlPoints[i].pos.y = hostPos.y;
					break;
				case ControlPoint::ANCHOR_STATIC_TOP:
					controlPoints[i].pos.x = hostPos.x;
					controlPoints[i].pos.y = 0;
					break;
				case ControlPoint::ANCHOR_STATIC_RIGHT:
					controlPoints[i].pos.x = deformedFrameSize.width - 1;
					controlPoints[i].pos.y = hostPos.y;
					break;
				case ControlPoint::ANCHOR_STATIC_BOTTOM:
					controlPoints[i].pos.x = hostPos.x;
					controlPoints[i].pos.y = deformedFrameSize.height - 1;
					break;
				case ControlPoint::ANCHOR_BOUND:
				case ControlPoint::ANCHOR_CENTER:
					RestrictInside( controlPoints[i].pos, deformedFrameSize );
					break;
				default:
					break;

			}

		}

	}

}

void Deformation::MinimizeEnergy() {

#define DEBUG_MIN_ENERGY
	
	printf( "Minimize deformation energy.\n" );

	double lambda = 1;
	double curE = CalcEnergy();
	printf( "\tIter 0. Energy: %.3lf. Learning rate: %.3lf.\n", curE, lambda );

	for ( int iter = 0; iter < MIN_ENERGY_ITERS; iter++ ) {

		vector<Point2f> newControlPoints( controlPointsNum );

		for ( int i = 0; i < controlPointsNum; i++ ) {
			newControlPoints[i] = controlPoints[i].pos;
		}

		MinEnergyStructureL( newControlPoints, lambda );

		MinEnergyStructureD( newControlPoints, lambda );

		MinEnergyShapeL( newControlPoints, lambda );

		MinEnergyShapeD( newControlPoints, lambda );

		if ( lambda < 0.8 ) {
			MinEnergyTemporal( newControlPoints, lambda );
		}

		CollinearConstraint( newControlPoints );
		
		UpdateControlPoints( newControlPoints );


		double preE = curE;
		curE = CalcEnergy();

		printf( "\tIter %d. Energy: %.3lf. Learning rate: %.3lf.\n", iter + 1, curE, lambda );

#ifdef DEBUG_MIN_ENERGY
		Mat edgeImg;
		DrawEdge(0, DEFORMED_POS, edgeImg);
		imshow( "Edge Image", edgeImg );
		waitKey(0);
#endif 

		if ( curE >= preE ) {
			lambda *= 0.8;
		}

		if ( lambda < ITER_TERMINATE ) break;

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

}

void Deformation::RenderKeyFrames() {

	printf( "Render key frames.\n" );

	for ( int i = 0; i < frameNum; i++ ) {

		Mat deformedFrame, edgeImg;

		RenderFrame( frames[i].img, deformedMap[i], deformedFrame );

		deformedFrames.push_back( deformedFrame );

		//imshow( "Saliency Map", frames[i].saliencyMap );
		//imshow( "Origin Frame", frames[i].img );
		DrawEdge( i, DEFORMED_POS_WITH_FRAME, edgeImg );
		WriteKeyFrameEdgeImg( frames[i].frameId, edgeImg, videoName );

		//waitKey();

	}

}

void Deformation::RenderFrames( const vector<Mat> &_frames, int shotSt, int shotEd ) {

	int keyFrameIndex = 0;

	for ( int i = shotSt; i < shotEd; i++ ) {

		Mat deformedFrame;
		int keyFrameIndexInSeries = frames[keyFrameIndex].frameId;

		if ( i == keyFrameIndexInSeries ) {

			deformedFrame = deformedFrames[keyFrameIndex];
			keyFrameIndex++;

		} else if ( i < keyFrameIndexInSeries ) {

			RenderFrame( _frames[i - shotSt], deformedMap[keyFrameIndex], deformedFrame );

		} else if ( i > keyFrameIndexInSeries ) {

			cout << "Error" << i << " " << keyFrameIndex << " " << keyFrameIndexInSeries << endl;

		}

		WriteDeformedImg( i, deformedFrame, videoName );
		
		imshow( "Deformed Frame", deformedFrame );
		waitKey( 1 );


	}

}