#include "Deformation.h"

Deformation::Deformation( vector<KeyFrame> &_frames, const string &_videoName ) : frames( _frames ) {

	videoName = _videoName;
	frameNum = _frames.size();
	frameSize = _frames[0].img.size();

	staticPoints.push_back( Point2f( 0, 0 ) );
	staticPoints.push_back( Point2f( frameSize.width - 1, 0 ) );
	staticPoints.push_back( Point2f( 0, frameSize.height - 1 ) );
	staticPoints.push_back( Point2f( frameSize.width - 1, frameSize.height - 1 ) );

	controlPointsNum = 0;
	freeEleNum = 0;
	freeEleMap.clear();
	controlPoints.clear();
	centerControlPointIndex.clear();
	temporalControlPointIndex.clear();
	spatialEdges.clear();
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

		for ( const auto &neighborIndex : controlPoint.boundNeighbors ) {

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

void Deformation::BuildControlPoints() {

	printf( "\tBuild key frames control points. " );

// #define DEBUG_DELAUNAY_DIVIDE

	for ( int i = 0; i < frameNum; i++ ) {

		Rect rect( 0, 0, frameSize.width, frameSize.height );
		Subdiv2D subdiv( rect );
		double superpixelMaxDist = 1.8 * sqrt( frameSize.width * frameSize.height / (double)frames[i].superpixelNum );
		controlPointsMap.push_back( Mat( frameSize, CV_32SC1, Scalar( -1 ) ) );

		// Add superpixel center points.
		frameControlPointIndex[i] = vector<int>( frames[i].superpixelNum );
		for ( int j = 0; j < frames[i].superpixelNum; j++ ) {
			double saliency = frames[i].superpixelSaliency[j];
			controlPoints.push_back( ControlPoint( i, frames[i].superpixelCenter[j], ControlPoint::ANCHOR_CENTER, j, saliency ) );
			controlPointsMap[i].at<int>( frames[i].superpixelCenter[j] ) = controlPointsNum;
			frameControlPointIndex[i][j] = controlPointsNum;
			centerControlPointIndex.push_back( controlPointsNum );

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

			int index0 = controlPointsMap[i].at<int>( p0 );
			int index1 = controlPointsMap[i].at<int>( p1 );

			string edgeHash0 = to_string( index0 ) + " " + to_string( index1 );
			string edgeHash1 = to_string( index1 ) + " " + to_string( index0 );
			if ( edgeExist.count( edgeHash0 ) > 0 ) continue;
			if ( edgeExist.count( edgeHash1 ) > 0 ) continue;
			edgeExist[edgeHash0] = 1;
			edgeExist[edgeHash1] = 1;

			if ( frames[i].superpixelBoundLabel[controlPoints[index0].superpixelIndex] != KeyFrame::BOUND_NONE &&
				 frames[i].superpixelBoundLabel[controlPoints[index1].superpixelIndex] != KeyFrame::BOUND_NONE ) {
				double superpixelDist = NormL2( p0 - p1 );
				if ( superpixelDist > superpixelMaxDist ) continue;
			}

			Point2f neighborPoint = GetBoundPoint( index0, index1 );
			controlPoints.push_back( ControlPoint( i, neighborPoint, ControlPoint::ANCHOR_BOUND, -1, -1 ) );
			controlPointsMap[i].at<int>( neighborPoint ) = controlPointsNum;

			controlPoints[index0].AddBoundNeighbor( controlPointsNum );
			controlPoints[index1].AddBoundNeighbor( controlPointsNum );
			controlPoints[controlPointsNum].AddBoundNeighbor( index0 );
			controlPoints[controlPointsNum].AddBoundNeighbor( index1 );

			controlPoints[index0].AddSuperpixelNeighbor( index1 );
			controlPoints[index1].AddSuperpixelNeighbor( index0 );

			controlPointsNum++;

		}

		// Add anchor points.
		for ( const auto &point : staticPoints ) {

			int label = frames[i].pixelLabel.at<int>( point );
			int controlPointIndex = frameControlPointIndex[i][label];

			controlPoints.push_back( ControlPoint( i, point, ControlPoint::ANCHOR_STATIC, label, -1 ) );
			controlPointsMap[i].at<int>( point ) = controlPointsNum;
			controlPoints[controlPointIndex].AddBoundNeighbor( controlPointsNum );
			controlPoints[controlPointsNum].AddBoundNeighbor( controlPointIndex );

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
					controlPointsMap[i].at<int>( pointBound ) = controlPointsNum;
					controlPoints[controlPointIndex].AddBoundNeighbor( controlPointsNum );
					controlPoints[controlPointsNum].AddBoundNeighbor( controlPointIndex );
					controlPointsNum++;
					
					break;

				case KeyFrame::BOUND_TOP:
					pointBound = Point( center.x, 0 );
					controlPoints.push_back( ControlPoint( i, pointBound, ControlPoint::ANCHOR_STATIC_TOP, j, -1 ) );
					controlPointsMap[i].at<int>( pointBound ) = controlPointsNum;
					controlPoints[controlPointIndex].AddBoundNeighbor( controlPointsNum );
					controlPoints[controlPointsNum].AddBoundNeighbor( controlPointIndex );
					controlPointsNum++;
					
					break;

				case KeyFrame::BOUND_RIGHT:
					pointBound = Point( frameSize.width - 1, center.y );
					controlPoints.push_back( ControlPoint( i, pointBound, ControlPoint::ANCHOR_STATIC_RIGHT, j, -1 ) );
					controlPointsMap[i].at<int>( pointBound ) = controlPointsNum;
					controlPoints[controlPointIndex].AddBoundNeighbor( controlPointsNum );
					controlPoints[controlPointsNum].AddBoundNeighbor( controlPointIndex );
					controlPointsNum++;
					
					break;

				case KeyFrame::BOUND_BOTTOM:
					pointBound = Point( center.x, frameSize.height - 1 );
					controlPoints.push_back( ControlPoint( i, pointBound, ControlPoint::ANCHOR_STATIC_BOTTOM, j, -1 ) );
					controlPointsMap[i].at<int>( pointBound ) = controlPointsNum;
					controlPoints[controlPointIndex].AddBoundNeighbor( controlPointsNum );
					controlPoints[controlPointsNum].AddBoundNeighbor( controlPointIndex );
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
		waitKey( 0 );
#endif

	}

	freeEleNum = 0;
	freeEleMap = vector<int>( 2 * controlPointsNum, -1 );

	for ( int i = 0; i < controlPointsNum; i++ ) {
		const ControlPoint &p = controlPoints[i];
		switch ( p.anchorType ) {
			case ControlPoint::ANCHOR_CENTER:
			case ControlPoint::ANCHOR_BOUND:
				freeEleMap[2 * i] = freeEleNum++;
				freeEleMap[2 * i + 1] = freeEleNum++;
				break;
			case ControlPoint::ANCHOR_STATIC_BOTTOM:
			case ControlPoint::ANCHOR_STATIC_TOP:
				freeEleMap[2 * i] = freeEleNum++;
				break;
			case ControlPoint::ANCHOR_STATIC_LEFT:
			case ControlPoint::ANCHOR_STATIC_RIGHT:
				freeEleMap[2 * i + 1] = freeEleNum++;
				break;
			default:
				break;
		}
	}


	printf( "Control point num: %d.\n", controlPointsNum );

}

void Deformation::AddSpatialNeighbors() {

// #define DEBUG_ADD_SPATIAL_NEIGHBORS

	printf( "\tAdd spatial neighbors to control points.\n" );

	int frameId = -1;
	Mat cannyImg, visitedMap;
	
	for ( int controlPointIndex = 0; controlPointIndex < controlPointsNum; controlPointIndex++ ) {

		ControlPoint &controlPoint = controlPoints[controlPointIndex];

		if ( frameId != controlPoint.frameId ) {
			
			frameId = controlPoint.frameId;
			Canny( frames[frameId].grayImg, cannyImg, 70, 140 );
			visitedMap = Mat( frameSize, CV_32SC1, Scalar(-1) );
#ifdef DEBUG_ADD_SPATIAL_NEIGHBORS
			//imshow( "canny", cannyImg );
			//waitKey( 0 );
#endif

		}

		queue<Point> que;
		Point seedPoint( controlPoint.originPos );
		vector<int> spatialEdge;

		if ( cannyImg.at<uchar>( controlPoint.originPos ) == 1 &&
			 visitedMap.at<int>( seedPoint ) == -1 ) {
			visitedMap.at<int>( seedPoint ) = controlPointIndex;
			que.push( seedPoint );
		}

		for ( int k = 0; k < NEIGHBORS_NUM; k++ ) {
			
			Point nextPos = seedPoint + neighbors[k];
			if ( CheckOutside( nextPos, frameSize ) ) continue;
			if ( cannyImg.at<uchar>( nextPos ) == 0 ) continue;
			if ( visitedMap.at<int>( nextPos ) != -1 )continue;

			visitedMap.at<int>( nextPos ) = controlPointIndex;
			que.push( nextPos );

		}
		
		if ( !que.empty() ) spatialEdge.push_back( controlPointIndex );

		while ( !que.empty() ) {

			Point curPos = que.front();
			que.pop();

			for ( int k = 0; k < NEIGHBORS_NUM; k++ ) {

				Point nextPos = curPos + neighbors[k];
				if ( CheckOutside( nextPos, frameSize ) ) continue;

				int nextControlPointIndex = controlPointsMap[frameId].at<int>( nextPos );
				if ( nextControlPointIndex != -1 ) spatialEdge.push_back( nextControlPointIndex );
			}

			for ( int k = 0; k < NEIGHBORS_NUM; k++ ) {

				Point nextPos = curPos + neighbors[k];
				if ( CheckOutside( nextPos, frameSize ) ) continue;
				if ( cannyImg.at<uchar>( nextPos ) == 0 ) continue;
				if ( visitedMap.at<int>( nextPos ) != -1 )continue;

				visitedMap.at<int>( nextPos ) = controlPointIndex;
				que.push( nextPos );

			}

		}

		// Erase duplications.
		sort( spatialEdge.begin(), spatialEdge.end() );

#ifdef DEBUG_ADD_SPATIAL_NEIGHBORS
		if ( spatialEdge.size() > 2 ) {
			for ( size_t i = 0; i < spatialEdge.size(); i++ ) {
				cout << spatialEdge[i] << " ";
			}
			cout << endl;
		}
#endif

		for ( size_t i = 1; i < spatialEdge.size(); i++ ) {
			if ( spatialEdge[i] == spatialEdge[i - 1] ) {
				spatialEdge.erase( spatialEdge.begin() + i );
				i--;
			}
		}

		if ( spatialEdge.size() > 2 ) {
			spatialEdges.push_back( spatialEdge );
		}

	}	

#ifdef DEBUG_ADD_SPATIAL_NEIGHBORS

	cout << endl << endl;

	for ( const auto &spatialEdge : spatialEdges ) {
		for ( const auto &controlPointIndex : spatialEdge ) {
			cout << controlPointIndex << " ";
		}
		cout << endl;
	}

	for ( const auto &spatialEdge: spatialEdges ) {
		Mat debugImg = frames[controlPoints[spatialEdge[0]].frameId].img.clone();
		for ( const auto &controlPointIndex : spatialEdge ) {
			ControlPoint &p = controlPoints[controlPointIndex];
			circle( debugImg, p.originPos, 2, Scalar( 0, 0, 255 ), 2 );
		}
		imshow( "Spatial Neighbors", debugImg );
		waitKey( 0 );
	}
#endif

}

void Deformation::CalcBaryCoordLambda( const Point2f &p, vector<Point2f> &vertices, vector<double> &lambda ) {

	if ( vertices.size() == 3 ) {

		double detT = (vertices[1].y - vertices[2].y) * (vertices[0].x - vertices[2].x) +
			(vertices[2].x - vertices[1].x) * (vertices[0].y - vertices[2].y);

		if ( SignNumber( detT ) == 0 ) {
			vertices.erase( vertices.begin() );
			lambda.erase( lambda.begin() );
			CalcBaryCoordLambda( p, vertices, lambda );
			return;
		}

		lambda[0] = ((vertices[1].y - vertices[2].y) * (p.x - vertices[2].x) +
					  (vertices[2].x - vertices[1].x) * (p.y - vertices[2].y)) / detT;
		lambda[1] = ((vertices[2].y - vertices[0].y) * (p.x - vertices[2].x) +
					  (vertices[0].x - vertices[2].x) * (p.y - vertices[2].y)) / detT;
		lambda[2] = 1 - lambda[0] - lambda[1];

	} else if ( vertices.size() == 2 ) {

		double detT = vertices[0].x * vertices[1].y - vertices[0].y * vertices[1].x;

		if ( SignNumber( detT ) == 0 ) {
			vertices.erase( vertices.begin() );
			lambda.erase( lambda.begin() );
			lambda[0] = 1;
			return;
		}

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

void Deformation::CalcBaryCoord1( const Mat &cpMap, const Point2f &p, vector<BaryCoord> &baryCoord ) {
	int controlPointIndex = cpMap.at<int>( p );
	if ( controlPointIndex == -1 ) {
		cout << "1 " << p << endl;
	}
	baryCoord.push_back( make_pair( 1, controlPointIndex ) );
}

void Deformation::CalcBaryCoord2( Subdiv2D &subdiv, const Mat &cpMap, int e0, const Point2f &p, vector<BaryCoord> &baryCoord ) {

	Point2f pointOrg, pointDst;
	vector<Point2f> vertices;

	if ( subdiv.edgeOrg( e0, &pointOrg ) > 0 && subdiv.edgeDst( e0, &pointDst ) > 0 ) {
		RestrictInside( pointOrg, frameSize );
		RestrictInside( pointDst, frameSize );
		vertices.push_back( pointOrg );
		vertices.push_back( pointDst );
	} else {
		cout << "[CalcBaryCoord2] Get points error: pointOrg " << subdiv.edgeOrg( e0, &pointOrg ) << ", pointDst " << subdiv.edgeDst( e0, &pointDst ) << endl;
	}

	vector<double> lambda( vertices.size() );

	CalcBaryCoordLambda( p, vertices, lambda );

	for ( size_t i = 0; i < vertices.size(); i++ ) {
		int vertex = cpMap.at<int>( vertices[i] );
		if ( vertex == -1 ) {
			cout << "2 " << vertices[i] << endl;
		}
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

void Deformation::CalcBaryCoord3( Subdiv2D &subdiv, const Mat &cpMap, int e0, const Point2f &p, vector<BaryCoord> &baryCoord ) {

	vector<Point2f> vertices;
	int e = e0;

	do {
		Point2f pointOrg, pointDst;
		if ( subdiv.edgeOrg( e, &pointOrg ) > 0 && subdiv.edgeDst( e, &pointDst ) > 0 ) {

			bool vertexExistFlag = false;
			RestrictInside( pointOrg, frameSize );
			for ( const auto &vertex : vertices ) {
				if ( vertex == pointOrg ) {
					vertexExistFlag = true;
					break;
				}
			}
			if ( !vertexExistFlag ) {
				vertices.push_back( pointOrg );
				if ( vertices.size() >= 3 ) break;
			}

			vertexExistFlag = false;
			RestrictInside( pointDst, frameSize );
			for ( const auto &vertex : vertices ) {
				if ( vertex == pointDst ) {
					vertexExistFlag = true;
					break;
				}
			}
			if ( !vertexExistFlag ) {
				vertices.push_back( pointDst );
				if ( vertices.size() >= 3 ) break;
			}

		}

		e = subdiv.getEdge( e, Subdiv2D::NEXT_AROUND_LEFT );

	} while ( e != e0 );

	vector<double> lambda( vertices.size() );

	CalcBaryCoordLambda( p, vertices, lambda );

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

	for ( size_t i = 0; i < vertices.size(); i++ ) {
		int vertex = cpMap.at<int>( vertices[i] );
		if ( vertex == -1 ) {
			cout << "3 " << vertices[i] << endl;
		}
		baryCoord.push_back( make_pair( lambda[i], vertex ) );
	}

}

int Deformation::LocatePoint( Subdiv2D &subdiv, const Mat &cpMap, const Point2f &p, vector<BaryCoord> &baryCoord ) {

//#define DEBUG_LOCATE_POINT

	int e0, vertex, locateStatus;

	baryCoord.clear();
	locateStatus = subdiv.locate( p, e0, vertex );

	switch ( locateStatus ) {
		case CV_PTLOC_INSIDE:
			CalcBaryCoord3( subdiv, cpMap, e0, p, baryCoord );
			break;
		case CV_PTLOC_ON_EDGE:
			CalcBaryCoord2( subdiv, cpMap, e0, p, baryCoord );
			break;
		case CV_PTLOC_VERTEX:
			CalcBaryCoord1( cpMap, p, baryCoord );
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

	for ( int frameId = 0; frameId < frameNum - 1; frameId++ ) {

		Subdiv2D subdiv( rect );

		for ( ; nextFramePointIndex < controlPointsNum; nextFramePointIndex++ ) {
			
			ControlPoint &controlPoint = controlPoints[nextFramePointIndex];
			if ( controlPoint.frameId == frameId + 2 ) break;
			subdiv.insert( controlPoint.originPos );

		}

		for ( ; curFramePointIndex < controlPointsNum; curFramePointIndex++ ) {
			
			ControlPoint &controlPoint = controlPoints[curFramePointIndex];
			if ( controlPoint.anchorType != ControlPoint::ANCHOR_CENTER && controlPoint.anchorType != ControlPoint::ANCHOR_BOUND ) continue;
			if ( controlPoint.frameId != frameId ) break;

			Point2f flow = frames[controlPoint.frameId].forwardFlowMap.at<Point2f>( Point( controlPoint.originPos ) );
			Point2f nextFramePos = controlPoint.originPos + flow;
			controlPoint.flow = Point2f( flow.x * deformedScaleX, flow.y * deformedScaleY );

			if ( CheckOutside( nextFramePos, frameSize ) ) continue;

			vector<BaryCoord> baryCoord;
			int locateStatus = LocatePoint( subdiv, controlPointsMap[frameId + 1], nextFramePos, baryCoord );
			if ( locateStatus == CV_PTLOC_INSIDE || locateStatus == CV_PTLOC_ON_EDGE || locateStatus == CV_PTLOC_VERTEX ) {
				controlPoint.AddTemporalNeighbor( baryCoord );
				temporalControlPointIndex.push_back( curFramePointIndex );
			}
			
		}

	}
	
}

void Deformation::InitDeformation( double _deformedScaleX, double _deformedScaleY ) {

	printf( "Initialize deformation.\n" );

	deformedScaleX = _deformedScaleX;
	deformedScaleY = _deformedScaleY;
	deformedFrameSize = Size( CeilToInt( frameSize.width * deformedScaleX ), CeilToInt( frameSize.height * deformedScaleY ) );

	BuildControlPoints();
	AddSpatialNeighbors();
	AddTemporalNeighbors();

	for ( auto &controlPoint : controlPoints ) {
		controlPoint.pos.x *= deformedScaleX;
		controlPoint.pos.y *= deformedScaleY;
	}

#ifdef DEBUG_INIT_DEFORMATION
	//DrawEdge( ORIGIN_POS );
	//DrawEdge( DEFORMED_POS );
#endif

}

double Deformation::CalcSaliencyEnergy() {

	double saliencyEnergy = 0;

	for ( const auto &centerPointIndex : centerControlPointIndex ) {

		double tmpEnergy = 0;
		const ControlPoint &centerPoint = controlPoints[centerPointIndex];

		for ( const auto &boundPointIndex : centerPoint.boundNeighbors ) {

			const ControlPoint &boundPoint = controlPoints[boundPointIndex];
			tmpEnergy += SqrNormL2( (centerPoint.pos - boundPoint.pos) - (centerPoint.originPos - boundPoint.originPos) );

		}

		saliencyEnergy += centerPoint.saliency * tmpEnergy;

	}

	saliencyEnergy *= 0.5;

	return saliencyEnergy;

}

double Deformation::CalcObjectEnergy() {
	
	double objectEnergy = 0;

	for ( const auto &centerPointIndex : centerControlPointIndex ) {

		const ControlPoint &centerPoint = controlPoints[centerPointIndex];

		for ( const auto &boundPointIndex : centerPoint.boundNeighbors ) {

			const ControlPoint &boundPoint = controlPoints[boundPointIndex];

			Point2f originVec = boundPoint.originPos - centerPoint.originPos;
			Point2f vec = boundPoint.pos - centerPoint.pos;
			double xRatio = (abs( originVec.x ) < SMALL_LEN) ? deformedScaleX : vec.x / originVec.x;
			double yRatio = (abs( originVec.y ) < SMALL_LEN) ? deformedScaleY : vec.y / originVec.y;

			objectEnergy += centerPoint.saliency * sqr( xRatio - yRatio );

		}

	}

	objectEnergy *= 0.5;

	return objectEnergy;
}


double Deformation::CalcStructureEnergy() {

	double structureEnergy = 0;

	for ( const auto &spatialEdge : spatialEdges ) {

		Point2f sum( 0, 0 );
		Point2f squaredSum( 0, 0 );
		for ( const auto &controlPointIndex : spatialEdge ) {
			const ControlPoint &p = controlPoints[controlPointIndex];
			sum.x += CalcRatio( p.pos.x, p.originPos.x );
			sum.y += CalcRatio( p.pos.y, p.originPos.y );
			squaredSum.x += sqr( CalcRatio( p.pos.x, p.originPos.x ) );
			squaredSum.y += sqr( CalcRatio( p.pos.y, p.originPos.y ) );
		}
		
		structureEnergy += 1.0f / spatialEdge.size() * (squaredSum.x + squaredSum.y)
			- 1.0f / sqr( spatialEdge.size() ) *(sqr( sum.x ) + sqr( sum.y ));
		
	}

	structureEnergy *= 0.5;

	return structureEnergy;

}

double Deformation::CalcTemporalEnergy() {

// #define DEBUG_CALC_TEMPORAL

	double temporalEnergy = 0;

	for ( auto &controlPointIndex : temporalControlPointIndex ) {

		const ControlPoint &controlPoint = controlPoints[controlPointIndex];

		Point2f nextFramePointPos = CalcPointByBaryCoord( controlPoint.temporalNeighbors, DEFORMED_POS );
		
#ifdef DEBUG_CALC_TEMPORAL
		cout << temporalEnergy << " " << nextFramePointPos << " " << controlPoint.pos << " " << controlPoint.flow << endl;
		for ( size_t i = 0; i < controlPoint.temporalNeighbors.size(); i++ ) {
			cout << "\tneighbors " << controlPoint.temporalNeighbors[i].first << " " << controlPoints[controlPoint.temporalNeighbors[i].second].pos << endl;
		}
#endif

		temporalEnergy += SqrNormL2( (nextFramePointPos - controlPoint.pos) - controlPoint.flow );

	}

	return temporalEnergy;

}

double Deformation::CalcEnergy() {

#define DEBUG_CALC_ENERGY

	double saliencyEnergy = CalcSaliencyEnergy();
	double objectEnergy = CalcObjectEnergy();
	double structureEnergy = CalcStructureEnergy();
	double temporalEnergy = CalcTemporalEnergy();

#ifdef DEBUG_CALC_ENERGY
	printf( "SaliencyE %.3lf, ObjectE %.3lf, StructureE %.3lf TemporalE %.3lf\n", saliencyEnergy, objectEnergy, structureEnergy, temporalEnergy );
#endif
	double energy = ALPHA_SALIENCY * saliencyEnergy + ALPHA_OBJECT * objectEnergy + ALPHA_STRUCTURE * structureEnergy + ALPHA_TEMPORAL * temporalEnergy;

	return energy;

}

void Deformation::AddSaliencyConstraints( Mat &coefMat, Mat &constVec ) {

	for ( auto &centerPointIndex : centerControlPointIndex ) {

		const ControlPoint &centerPoint = controlPoints[centerPointIndex];

		double saliencySum = centerPoint.saliency * centerPoint.boundNeighbors.size();

		// row centerPoint, col centerPoint
		coefMat.at<float>( freeEleMap[2 * centerPointIndex], freeEleMap[2 * centerPointIndex] ) += ALPHA_SALIENCY * saliencySum;
		coefMat.at<float>( freeEleMap[2 * centerPointIndex + 1], freeEleMap[2 * centerPointIndex + 1] ) += ALPHA_SALIENCY * saliencySum;

		for ( auto &boundPointIndex : centerPoint.boundNeighbors ) {

			const ControlPoint &boundPoint = controlPoints[boundPointIndex];

			switch ( boundPoint.anchorType ) {
				case ControlPoint::ANCHOR_BOUND:

					// row centerPoint, col boundPoint
					coefMat.at<float>( freeEleMap[2 * centerPointIndex], freeEleMap[2 * boundPointIndex] ) -= ALPHA_SALIENCY * centerPoint.saliency;
					coefMat.at<float>( freeEleMap[2 * centerPointIndex + 1], freeEleMap[2 * boundPointIndex + 1] ) -= ALPHA_SALIENCY * centerPoint.saliency;

					// row boundPoint
					coefMat.at<float>( freeEleMap[2 * boundPointIndex], freeEleMap[2 * centerPointIndex] ) -= ALPHA_SALIENCY * centerPoint.saliency;
					coefMat.at<float>( freeEleMap[2 * boundPointIndex + 1], freeEleMap[2 * centerPointIndex + 1] ) -= ALPHA_SALIENCY * centerPoint.saliency;
					coefMat.at<float>( freeEleMap[2 * boundPointIndex], freeEleMap[2 * boundPointIndex] ) += ALPHA_SALIENCY * centerPoint.saliency;
					coefMat.at<float>( freeEleMap[2 * boundPointIndex + 1], freeEleMap[2 * boundPointIndex + 1] ) += ALPHA_SALIENCY * centerPoint.saliency;

					constVec.at<float>( freeEleMap[2 * boundPointIndex], 0 ) -= ALPHA_SALIENCY * centerPoint.saliency * (centerPoint.originPos.x - boundPoint.originPos.x);
					constVec.at<float>( freeEleMap[2 * boundPointIndex + 1], 0 ) -= ALPHA_SALIENCY * centerPoint.saliency * (centerPoint.originPos.y - boundPoint.originPos.y);

					break;

				case ControlPoint::ANCHOR_STATIC_LEFT:
				case ControlPoint::ANCHOR_STATIC_RIGHT:

					// row centerPoint
					constVec.at<float>( freeEleMap[2 * centerPointIndex], 0 ) += ALPHA_SALIENCY * centerPoint.saliency * boundPoint.pos.x;
					coefMat.at<float>( freeEleMap[2 * centerPointIndex + 1], freeEleMap[2 * boundPointIndex + 1] ) -= ALPHA_SALIENCY * centerPoint.saliency;

					// row boundPoint
					coefMat.at<float>( freeEleMap[2 * boundPointIndex + 1], freeEleMap[2 * centerPointIndex + 1] ) -= ALPHA_SALIENCY * centerPoint.saliency;
					coefMat.at<float>( freeEleMap[2 * boundPointIndex + 1], freeEleMap[2 * boundPointIndex + 1] ) += ALPHA_SALIENCY * centerPoint.saliency;
					constVec.at<float>( freeEleMap[2 * boundPointIndex + 1], 0 ) -= ALPHA_SALIENCY * centerPoint.saliency * (centerPoint.originPos.y - boundPoint.originPos.y);

					break;

				case ControlPoint::ANCHOR_STATIC_TOP:
				case ControlPoint::ANCHOR_STATIC_BOTTOM:

					// row centerPoint
					coefMat.at<float>( freeEleMap[2 * centerPointIndex], freeEleMap[2 * boundPointIndex] ) -= ALPHA_SALIENCY * centerPoint.saliency;
					constVec.at<float>( freeEleMap[2 * centerPointIndex + 1], 0 ) += ALPHA_SALIENCY * centerPoint.saliency * boundPoint.pos.y;

					// row boundPoint
					coefMat.at<float>( freeEleMap[2 * boundPointIndex], freeEleMap[2 * centerPointIndex] ) -= ALPHA_SALIENCY * centerPoint.saliency;
					coefMat.at<float>( freeEleMap[2 * boundPointIndex], freeEleMap[2 * boundPointIndex] ) += ALPHA_SALIENCY * centerPoint.saliency;
					constVec.at<float>( freeEleMap[2 * boundPointIndex], 0 ) -= ALPHA_SALIENCY * centerPoint.saliency * (centerPoint.originPos.x - boundPoint.originPos.x);

					break;

				case ControlPoint::ANCHOR_STATIC:

					// row centerPoint
					constVec.at<float>( freeEleMap[2 * centerPointIndex], 0 ) += ALPHA_SALIENCY * centerPoint.saliency * boundPoint.pos.x;
					constVec.at<float>( freeEleMap[2 * centerPointIndex + 1], 0 ) += ALPHA_SALIENCY * centerPoint.saliency * boundPoint.pos.y;

					break;

				default:
					cerr << "Wrong bound point type in function AddSaliencyConstraints." << endl;
					break;
			}

			// row centerPoint
			constVec.at<float>( freeEleMap[2 * centerPointIndex], 0 ) += ALPHA_SALIENCY * centerPoint.saliency * (centerPoint.originPos.x - boundPoint.originPos.x);
			constVec.at<float>( freeEleMap[2 * centerPointIndex + 1], 0 ) += ALPHA_SALIENCY * centerPoint.saliency * (centerPoint.originPos.y - boundPoint.originPos.y);

		}

	}

}

void Deformation::AddObjectConstraints( Mat &coefMat, Mat &constVec ) {

	for ( const auto &centerPointIndex : centerControlPointIndex ) {

		const ControlPoint &centerPoint = controlPoints[centerPointIndex];
		const double saliency = centerPoint.saliency;

		for ( const auto &boundPointIndex : centerPoint.boundNeighbors ) {

			const ControlPoint &boundPoint = controlPoints[boundPointIndex];
			Point2f originVec = boundPoint.originPos - centerPoint.originPos;
			Point2f invOriginVec;

			invOriginVec.x = (abs( originVec.x ) < SMALL_LEN) ? 0 : 1 / originVec.x;
			invOriginVec.y = (abs( originVec.y ) < SMALL_LEN) ? 0 : 1 / originVec.y;

			if ( abs( originVec.x ) < SMALL_LEN && abs( originVec.y ) < SMALL_LEN ) continue;

			if ( abs( originVec.x ) < SMALL_LEN ) {
				if ( freeEleMap[2 * boundPointIndex + 1] != -1 ) {
					constVec.at<float>( freeEleMap[2 * boundPointIndex + 1], 0 ) += ALPHA_OBJECT * saliency * deformedScaleX * invOriginVec.y;
				}
				if ( freeEleMap[2 * centerPointIndex + 1] != -1 ) {
					constVec.at<float>( freeEleMap[2 * centerPointIndex + 1], 0 ) -= ALPHA_OBJECT * saliency * deformedScaleX * invOriginVec.y;
				}
			} else {
				if ( freeEleMap[2 * boundPointIndex] != -1 ) {
					if ( freeEleMap[2 * boundPointIndex] != -1 ) {
						coefMat.at<float>( freeEleMap[2 * boundPointIndex], freeEleMap[2 * boundPointIndex] ) += ALPHA_OBJECT * saliency * sqr( invOriginVec.x );
					} else {
						constVec.at<float>( freeEleMap[2 * boundPointIndex], 0 ) -= ALPHA_OBJECT * saliency * controlPoints[boundPointIndex].pos.x * sqr( invOriginVec.x );
					}
					if ( freeEleMap[2 * centerPointIndex] != -1 ) {
						coefMat.at<float>( freeEleMap[2 * boundPointIndex], freeEleMap[2 * centerPointIndex] ) -= ALPHA_OBJECT * saliency * sqr( invOriginVec.x );
					} else {
						constVec.at<float>( freeEleMap[2 * boundPointIndex], 0 ) += ALPHA_OBJECT * saliency * controlPoints[centerPointIndex].pos.x * sqr( invOriginVec.x );
					}
				}
				if ( freeEleMap[2 * centerPointIndex] != -1 ) {
					if ( freeEleMap[2 * boundPointIndex] != -1 ) {
						coefMat.at<float>( freeEleMap[2 * centerPointIndex], freeEleMap[2 * boundPointIndex] ) -= ALPHA_OBJECT * saliency * sqr( invOriginVec.x );
					} else {
						constVec.at<float>( freeEleMap[2 * centerPointIndex], 0 ) += ALPHA_OBJECT * saliency * controlPoints[boundPointIndex].pos.x * sqr( invOriginVec.x );
					}
					if ( freeEleMap[2 * centerPointIndex] != -1 ) {
						coefMat.at<float>( freeEleMap[2 * centerPointIndex], freeEleMap[2 * centerPointIndex] ) += ALPHA_OBJECT * saliency * sqr( invOriginVec.x );
					} else {
						constVec.at<float>( freeEleMap[2 * centerPointIndex], 0 ) -= ALPHA_OBJECT * saliency * controlPoints[centerPointIndex].pos.x * sqr( invOriginVec.x );
					}
				}
				if ( freeEleMap[2 * boundPointIndex + 1] != -1 && abs( originVec.y ) > SMALL_LEN ) {
					if ( freeEleMap[2 * boundPointIndex] != -1 ) {
						coefMat.at<float>( freeEleMap[2 * boundPointIndex + 1], freeEleMap[2 * boundPointIndex] ) -= ALPHA_OBJECT * saliency * invOriginVec.x * invOriginVec.y;
					} else {
						constVec.at<float>( freeEleMap[2 * boundPointIndex + 1], 0 ) += ALPHA_OBJECT * saliency * controlPoints[boundPointIndex].pos.x * invOriginVec.x * invOriginVec.y;
					}
					if ( freeEleMap[2 * centerPointIndex] != -1 ) {
						coefMat.at<float>( freeEleMap[2 * boundPointIndex + 1], freeEleMap[2 * centerPointIndex] ) += ALPHA_OBJECT * saliency * invOriginVec.x * invOriginVec.y;
					} else {
						constVec.at<float>( freeEleMap[2 * boundPointIndex + 1], 0 ) -= ALPHA_OBJECT * saliency * controlPoints[centerPointIndex].pos.x * invOriginVec.x * invOriginVec.y;
					}
				}
				if ( freeEleMap[2 * centerPointIndex + 1] != -1 && abs( originVec.y ) > SMALL_LEN ) {
					if ( freeEleMap[2 * boundPointIndex] != -1 ) {
						coefMat.at<float>( freeEleMap[2 * centerPointIndex + 1], freeEleMap[2 * boundPointIndex] ) += ALPHA_OBJECT * saliency * invOriginVec.x * invOriginVec.y;
					} else {
						constVec.at<float>( freeEleMap[2 * centerPointIndex + 1], 0 ) -= ALPHA_OBJECT * saliency * controlPoints[boundPointIndex].pos.x * invOriginVec.x * invOriginVec.y;
					}
					if ( freeEleMap[2 * centerPointIndex] != -1 ) {
						coefMat.at<float>( freeEleMap[2 * centerPointIndex + 1], freeEleMap[2 * centerPointIndex] ) -= ALPHA_OBJECT * saliency * invOriginVec.x * invOriginVec.y;
					} else {
						constVec.at<float>( freeEleMap[2 * centerPointIndex + 1], 0 ) += ALPHA_OBJECT * saliency * controlPoints[centerPointIndex].pos.x * invOriginVec.x * invOriginVec.y;
					}
				}
			}

			if ( abs( originVec.y ) < SMALL_LEN ) {
				if ( freeEleMap[2 * boundPointIndex] != -1 ) {
					constVec.at<float>( freeEleMap[2 * boundPointIndex], 0 ) += ALPHA_OBJECT * saliency * deformedScaleY * invOriginVec.x;
				}
				if ( freeEleMap[2 * centerPointIndex] != -1 ) {
					constVec.at<float>( freeEleMap[2 * centerPointIndex], 0 ) -= ALPHA_OBJECT * saliency * deformedScaleY * invOriginVec.x;
				}
			} else {
				if ( freeEleMap[2 * boundPointIndex] != -1 && abs( originVec.x ) > SMALL_LEN ) {
					if ( freeEleMap[2 * boundPointIndex + 1] != -1 ) {
						coefMat.at<float>( freeEleMap[2 * boundPointIndex], freeEleMap[2 * boundPointIndex + 1] ) -= ALPHA_OBJECT * saliency * invOriginVec.x * invOriginVec.y;
					} else {
						constVec.at<float>( freeEleMap[2 * boundPointIndex], 0 ) += ALPHA_OBJECT * saliency * controlPoints[boundPointIndex].pos.y * invOriginVec.x * invOriginVec.y;
					}
					if ( freeEleMap[2 * centerPointIndex + 1] != -1 ) {
						coefMat.at<float>( freeEleMap[2 * boundPointIndex], freeEleMap[2 * centerPointIndex + 1] ) += ALPHA_OBJECT * saliency * invOriginVec.x * invOriginVec.y;
					} else {
						constVec.at<float>( freeEleMap[2 * boundPointIndex], 0 ) -= ALPHA_OBJECT * saliency * controlPoints[centerPointIndex].pos.y * invOriginVec.x * invOriginVec.y;
					}
				}
				if ( freeEleMap[2 * centerPointIndex] != -1 && abs( originVec.x ) > SMALL_LEN ) {
					if ( freeEleMap[2 * boundPointIndex + 1] != -1 ) {
						coefMat.at<float>( freeEleMap[2 * centerPointIndex], freeEleMap[2 * boundPointIndex + 1] ) += ALPHA_OBJECT * saliency * invOriginVec.x * invOriginVec.y;
					} else {
						constVec.at<float>( freeEleMap[2 * centerPointIndex], 0 ) -= ALPHA_OBJECT * saliency * controlPoints[boundPointIndex].pos.y * invOriginVec.x * invOriginVec.y;
					}
					if ( freeEleMap[2 * centerPointIndex + 1] != -1 ) {
						coefMat.at<float>( freeEleMap[2 * centerPointIndex], freeEleMap[2 * centerPointIndex + 1] ) -= ALPHA_OBJECT * saliency * invOriginVec.x * invOriginVec.y;
					} else {
						constVec.at<float>( freeEleMap[2 * centerPointIndex], 0 ) += ALPHA_OBJECT * saliency * controlPoints[centerPointIndex].pos.y * invOriginVec.x * invOriginVec.y;
					}

				}
				if ( freeEleMap[2 * boundPointIndex + 1] != -1 ) {
					if ( freeEleMap[2 * boundPointIndex + 1] != -1 ) {
						coefMat.at<float>( freeEleMap[2 * boundPointIndex + 1], freeEleMap[2 * boundPointIndex + 1] ) += ALPHA_OBJECT * saliency * sqr( invOriginVec.y );
					} else {
						constVec.at<float>( freeEleMap[2 * boundPointIndex + 1], 0 ) -= ALPHA_OBJECT * saliency * controlPoints[boundPointIndex].pos.y * sqr( invOriginVec.y );
					}
					if ( freeEleMap[2 * centerPointIndex + 1] != -1 ) {
						coefMat.at<float>( freeEleMap[2 * boundPointIndex + 1], freeEleMap[2 * centerPointIndex + 1] ) -= ALPHA_OBJECT * saliency * sqr( invOriginVec.y );
					} else {
						constVec.at<float>( freeEleMap[2 * boundPointIndex + 1], 0 ) += ALPHA_OBJECT * saliency * controlPoints[centerPointIndex].pos.y * sqr( invOriginVec.y );
					}
				}
				if ( freeEleMap[2 * centerPointIndex + 1] != -1 ) {
					if ( freeEleMap[2 * boundPointIndex + 1] != -1 ) {
						coefMat.at<float>( freeEleMap[2 * centerPointIndex + 1], freeEleMap[2 * boundPointIndex + 1] ) -= ALPHA_OBJECT * saliency * sqr( invOriginVec.y );
					} else {
						constVec.at<float>( freeEleMap[2 * centerPointIndex + 1], 0 ) += ALPHA_OBJECT * saliency * controlPoints[boundPointIndex].pos.y * sqr( invOriginVec.y );
					}
					if ( freeEleMap[2 * centerPointIndex + 1] != -1 ) {
						coefMat.at<float>( freeEleMap[2 * centerPointIndex + 1], freeEleMap[2 * centerPointIndex + 1] ) += ALPHA_OBJECT * saliency * sqr( invOriginVec.y );
					} else {
						constVec.at<float>( freeEleMap[2 * centerPointIndex + 1], 0 ) -= ALPHA_OBJECT * saliency * controlPoints[centerPointIndex].pos.y * sqr( invOriginVec.y );
					}
				}
			}

			/*if ( centerPoint.frameId == 0 ) {
				cout << centerPoint.originPos << " " << boundPoint.originPos << endl;
				cout << coefMat << endl;
				cout << constVec << endl;
				cout << endl;
			}*/

		}

	}

}

void Deformation::AddStructureConstraints( Mat &coefMat, Mat &constVec ) {

	for ( const auto &spatialEdge : spatialEdges ) {

		int spatialNum = spatialEdge.size();
		for ( const auto &controlPointIndex : spatialEdge ) {
			
			if ( controlPoints[controlPointIndex].anchorType != ControlPoint::ANCHOR_CENTER &&
				 controlPoints[controlPointIndex].anchorType != ControlPoint::ANCHOR_BOUND ) {
				continue;
			}
			for ( const auto &colIndex : spatialEdge ) {

				if ( controlPointIndex == colIndex ) {
					coefMat.at<float>( freeEleMap[2 * controlPointIndex], freeEleMap[2 * colIndex] ) = ALPHA_STRUCTURE * 2.0f * (spatialNum - 1) / sqr( spatialNum );
					coefMat.at<float>( freeEleMap[2 * controlPointIndex + 1], freeEleMap[2 * colIndex + 1] ) = ALPHA_STRUCTURE * 2.0f * (spatialNum - 1) / sqr( spatialNum );
				} else {

					if ( controlPoints[colIndex].anchorType != ControlPoint::ANCHOR_CENTER &&
						 controlPoints[colIndex].anchorType != ControlPoint::ANCHOR_BOUND ) {
						constVec.at<float>( freeEleMap[2 * controlPointIndex], 0 ) += ALPHA_STRUCTURE * 2.0f / sqr( spatialNum );
						constVec.at<float>( freeEleMap[2 * controlPointIndex + 1], 0 ) += ALPHA_STRUCTURE * 2.0f / sqr( spatialNum );
					} else {
						coefMat.at<float>( freeEleMap[2 * controlPointIndex], freeEleMap[2 * colIndex] ) = -ALPHA_STRUCTURE * 2.0f / sqr( spatialNum );
						coefMat.at<float>( freeEleMap[2 * controlPointIndex + 1], freeEleMap[2 * colIndex + 1] ) = -ALPHA_STRUCTURE * 2.0f / sqr( spatialNum );
					}
					
				}
			}
			
		}

	}

}

void Deformation::AddTemporalConstraints( Mat &coefMat, Mat &constVec ) {

}

void Deformation::OptimizeEnergyFunction() {

#define OPTIMIZE_ENERGY_FUNC

	Mat coefMat( freeEleNum, freeEleNum, CV_32FC1, Scalar( 0 ) );
	Mat constVec( freeEleNum, 1, CV_32FC1, Scalar( 0 ) );

	AddSaliencyConstraints( coefMat, constVec );
	AddObjectConstraints( coefMat, constVec );
	// AddStructureConstraints( coefMat, constVec );
	// AddTemporalConstraints( coefMat, constVec );

	Mat tmpCoefMat( 10, 10, CV_32FC1, Scalar( 0 ) );
	Mat tmpConstVec( 10, 1, CV_32FC1, Scalar( 0 ) );

#ifdef OPTIMIZE_ENERGY_FUNC
	//coefMat( Rect( 0, 0, 5, 5 ) ).copyTo( tmpCoefMat( Rect( 0, 0, 5, 5 ) ) );
	//coefMat( Rect( 8, 8, 5, 5 ) ).copyTo( tmpCoefMat( Rect( 5, 5, 5, 5 ) ) );
	//constVec( Rect( 0, 0, 1, 5 ) ).copyTo( tmpConstVec( Rect( 0, 0, 1, 5 ) ) );
	//constVec( Rect( 0, 8, 1, 5 ) ).copyTo( tmpConstVec( Rect( 0, 5, 1, 5 ) ) );
	//
	//cout << tmpCoefMat << endl << tmpConstVec << endl;

	//coefMat = tmpCoefMat.clone();
	//constVec = tmpConstVec.clone();
#endif

	Mat resVec;
	bool t = solve( coefMat, constVec, resVec, DECOMP_NORMAL );
	cout << "solve " << t << endl;

#ifdef OPTIMIZE_ENERGY_FUNC
	// cout << coefMat << endl;
	// cout << constVec << endl;
	// cout << resVec << endl;

	//Mat tmpResVec( 16, 1, CV_32FC1, Scalar( 0 ) );
	//resVec( Rect( 0, 0, 1, 5 ) ).copyTo( tmpResVec( Rect( 0, 0, 1, 5 ) ) );
	//resVec( Rect( 0, 5, 1, 5 ) ).copyTo( tmpResVec( Rect( 0, 8, 1, 5 ) ) );
	//resVec = tmpResVec.clone();

#endif

	for ( int i = 0; i < controlPointsNum; i++ ) {
		if ( freeEleMap[2 * i] != -1 ) controlPoints[i].pos.x = resVec.at<float>( freeEleMap[2 * i], 0 );
		if ( freeEleMap[2 * i + 1] != -1 ) controlPoints[i].pos.y = resVec.at<float>( freeEleMap[2 * i + 1], 0 );
	}

#ifdef OPTIMIZE_ENERGY_FUNC
	for ( int i = 0; i < controlPointsNum; i++ ) {
		if ( controlPoints[i].frameId != 0 ) break;
		cout << controlPoints[i].originPos << " " << controlPoints[i].pos << " " << controlPoints[i].saliency << " ";
		switch ( controlPoints[i].anchorType ) {
			case ControlPoint::ANCHOR_CENTER:
				cout << "CENTER" << endl;
				break;
			case ControlPoint::ANCHOR_BOUND:
				cout << "BOUND" << endl;
				break;
			case ControlPoint::ANCHOR_STATIC:
				cout << "STATIC" << endl;
				break;
			case ControlPoint::ANCHOR_STATIC_LEFT:
				cout << "STATIC_LEFT" << endl;
				break;
			case ControlPoint::ANCHOR_STATIC_RIGHT:
				cout << "STATIC_RIGHT" << endl;
				break;
			case ControlPoint::ANCHOR_STATIC_TOP:
				cout << "STATIC_TOP" << endl;
				break;
			case ControlPoint::ANCHOR_STATIC_BOTTOM:
				cout << "STATIC_BOTTOM" << endl;
				break;
			default:
				break;
		}
	}
#endif

}


void Deformation::CollinearConstraints() {

// #define DEBUG_COLLINEAR

	for ( int i = 0; i < controlPointsNum; i++ ) {

		if ( controlPoints[i].anchorType != ControlPoint::ANCHOR_BOUND ) continue;

		Point2f p0 = controlPoints[i].pos;
		Point2f p1 = controlPoints[controlPoints[i].boundNeighbors[0]].pos;
		Point2f p2 = controlPoints[controlPoints[i].boundNeighbors[1]].pos;

		Point2f u1 = p0 - p1;
		Point2f u2 = p2 - p1;

		double norm = NormL2( u2 );
		if ( SignNumber( norm ) == 0 ) {
			controlPoints[i].pos = p1;
			continue;
		}

		double projection = DotProduct( u1, u2 ) / sqr( norm );

		projection = max( min( projection, 1.0 ), 0.0 );
		controlPoints[i].pos = p1 + projection * u2;

	}

}

void Deformation::UpdateControlPoints() {

//#define DEBUG_MIN_ENERGY_UPDATE

	for ( int i = 0; i < controlPointsNum; i++ ) {

		switch ( controlPoints[i].anchorType ) {
			case ControlPoint::ANCHOR_BOUND:
			case ControlPoint::ANCHOR_CENTER:
				RestrictInside( controlPoints[i].pos, deformedFrameSize );
				break;
			default:
				break;
				

		}

	}

}

void Deformation::MinimizeEnergy() {

#define DEBUG_MIN_ENERGY

	printf( "\nMinimize deformation energy.\n" );

	double energy = CalcEnergy();
	printf( "\tInit Energy: %.3lf.\n", energy );

	OptimizeEnergyFunction();
	CollinearConstraints();
	UpdateControlPoints();

	energy = CalcEnergy();
	printf( "\tOptimized Energy: %.3lf.\n", energy );

#ifdef DEBUG_MIN_ENERGY
	Mat edgeImg;
	DrawEdge( 0, DEFORMED_POS, edgeImg );
	imshow( "Deformed Image 0", edgeImg );
	DrawEdge( 0, ORIGIN_POS, edgeImg );
	imshow( "Origin Image 0", edgeImg );
	waitKey( 1 );
#endif 
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

void Deformation::RenderFrames( const vector<Mat> &_frames, int shotSt, int shotEd ) {

	//int keyframeId = 0;

	//for ( int i = shotSt; i < shotEd; i++ ) {

	//	Mat deformedFrame;
	//	int keyframeIdInSeries = frames[keyframeId].frameId;

	//	if ( i == keyframeIdInSeries ) {

	//		deformedFrame = deformedFrames[keyframeId];
	//		keyframeId++;

	//	} else if ( i < keyframeIdInSeries ) {

	//		RenderFrame( _frames[i - shotSt], deformedMap[keyframeId], deformedFrame );

	//	} else if ( i > keyframeIdInSeries ) {

	//		cout << "Error" << i << " " << keyframeId << " " << keyframeIdInSeries << endl;

	//	}

	//	WriteDeformedImg( i, deformedFrame, videoName );
	//	
	//	imshow( "Deformed Frame", deformedFrame );
	//	waitKey( 1 );


	//}

}