#include "Deformation.h"

Deformation::Deformation( vector<KeyFrame> &_frames ) :frames( _frames ) {
	frameNum = _frames.size();
	frameSize = _frames[0].img.size();

	frameSubdiv.clear();
	anchorPoints.push_back( make_pair( Point2f( 0, 0 ), ControlPoint::ANCHOR_TOP_LEFT ) );
	anchorPoints.push_back( make_pair( Point2f( frameSize.width - 1, 0 ), ControlPoint::ANCHOR_TOP_RIGHT ) );
	anchorPoints.push_back( make_pair( Point2f( 0, frameSize.height - 1 ), ControlPoint::ANCHOR_BOTTOM_LEFT ) );
	anchorPoints.push_back( make_pair( Point2f( frameSize.width - 1, frameSize.height - 1 ), ControlPoint::ANCHOR_BOTTOM_RIGHT ) );

	float bigCoord = 3.f * max( frameSize.width, frameSize.height );
	staticPoints.push_back( make_pair( Point2f( bigCoord, 0 ), ControlPoint::ANCHOR_STATIC ) );
	staticPoints.push_back( make_pair( Point2f( 0, bigCoord ), ControlPoint::ANCHOR_STATIC ) );
	staticPoints.push_back( make_pair( Point2f( -bigCoord, -bigCoord ), ControlPoint::ANCHOR_STATIC ) );

	controlPointsNum = 0;
	controlPoints.clear();
	posToPointIndexMap = vector< map<string, int> >( frameNum );
	frameControlPointIndex = vector< vector<int> >( frameNum );

	for ( auto &frame : frames ) {
		frame.FreeMemory();
	}

}

void Deformation::DrawSubdiv( const Mat &_img, const Subdiv2D &subdiv ) {

	Scalar delaunay_color( 255, 255, 255 );
	Mat img = _img.clone();

#if 0
	vector<Vec6f> triList;
	subdiv.getTriangleList( triList );
	vector<Point> pt( 3 );

	for ( size_t i = 0; i < triList.size(); i++ ) {
		Vec6f t = triList[i];
		pt[0] = Point( cvRound( t[0] ), cvRound( t[1] ) );
		pt[1] = Point( cvRound( t[2] ), cvRound( t[3] ) );
		pt[2] = Point( cvRound( t[4] ), cvRound( t[5] ) );
		line( img, pt[0], pt[1], delaunay_color, 1, CV_AA, 0 );
		line( img, pt[1], pt[2], delaunay_color, 1, CV_AA, 0 );
		line( img, pt[2], pt[0], delaunay_color, 1, CV_AA, 0 );

		cout << pt[0] << pt[1] << pt[2] << endl;
	}
#else
	vector<Vec4f> edgeList;
	subdiv.getEdgeList( edgeList );
	for ( size_t i = 0; i < edgeList.size(); i++ ) {
		Vec4f e = edgeList[i];
		Point pt0 = Point( cvRound( e[0] ), cvRound( e[1] ) );
		Point pt1 = Point( cvRound( e[2] ), cvRound( e[3] ) );
		line( img, pt0, pt1, delaunay_color, 1, CV_AA, 0 );
	}
#endif

#ifdef DEBUG
	imshow( "SubDiv", img );
	waitKey( 100 );
#endif

}

void Deformation::DrawEdge( int posType ) {

	Mat img;
	Scalar lineColor( 255, 255, 255 );
	int lastFrameId = -1;

	if ( posType == ORIGIN_POS ) {

		for ( const auto &e : spatialEdge ) {

			ControlPoint p0 = controlPoints[e.first];
			ControlPoint p1 = controlPoints[e.second];

			// cout << p0.frameId << " " << p0.originPos << " " << p1.originPos << endl;

			if ( p0.frameId != lastFrameId ) {
#ifdef DEBUG
				if ( lastFrameId != -1 ) {
					imshow( "Deformation Edge", img );
					waitKey( 0 );
				}
#endif
				lastFrameId = p0.frameId;
				img = Mat::zeros( frameSize, CV_8UC3 );
			}

			line( img, p0.originPos, p1.originPos, lineColor, 1, CV_AA );
		}

	} else if ( posType == DEFORMED_POS ) {

		for ( const auto &e : spatialEdge ) {

			ControlPoint p0 = controlPoints[e.first];
			ControlPoint p1 = controlPoints[e.second];

			// cout << p0.frameId << " " << p0.pos << " " << p1.pos << endl;

			if ( p0.frameId != lastFrameId ) {
#ifdef DEBUG
				if ( lastFrameId != -1 ) {
					imshow( "Deformation Edge", img );
					waitKey( 0 );
				}
#endif
				lastFrameId = p0.frameId;
				img = Mat::zeros( frameSize, CV_8UC3 );
			}

			line( img, p0.pos, p1.pos, lineColor, 1, CV_AA );
		}

	}

	imshow( "Deformation Edge", img );
	waitKey( 0 );

}

void Deformation::DrawLocate( const Point2f &p, const vector<BaryCoord> &baryCoord ) {

	Mat img = Mat::zeros( frameSize, CV_8UC3 );

	circle( img, p, 5, Scalar( 0, 0, 255 ), 2, CV_AA );
	
	for ( const auto &vertex : baryCoord ) {
		ControlPoint controlPoint = controlPoints[vertex.second];
		circle( img, controlPoint.originPos, 5, Scalar( 0, 0, 128 ), 2, CV_AA );
		circle( img, controlPoint.pos, 5, Scalar( 128, 0, 0 ), 2, CV_AA );
	}

	Point2f deformedPoint = CalcPointByBaryCoord( baryCoord );
	circle( img, deformedPoint, 5, Scalar( 255, 0, 0 ), 2, CV_AA );

	imshow( "Locate", img );
	waitKey( 1 );

}

void Deformation::CalcBaryCooordLambda(const Point2f &p, const vector<Point2f> &vertices, vector<double> &lambda) {
	
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

void Deformation::CalcBaryCoord3( int frameId, int e0, const Point2f &nextFramePos, vector<BaryCoord> &baryCoord ) {

	vector<Point2f> triVertices;
	int e = e0;

	do {
		Point2f pointOrg, pointDst;
		if ( frameSubdiv[frameId].edgeOrg( e, &pointOrg ) > 0 && frameSubdiv[frameId].edgeDst( e, &pointDst ) > 0 ) {

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

		e = frameSubdiv[frameId].getEdge( e, Subdiv2D::NEXT_AROUND_LEFT );

	} while ( e != e0 );

	if ( triVertices.size() != 3 ) {
		printf( "[CalcBaryCoord3] Triangle vertices size is inequal 3.\n" );
		return;
	}

	vector<double> lambda( 3 );

	CalcBaryCooordLambda( nextFramePos, triVertices, lambda );

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
		int vertex = posToPointIndexMap[frameId][Point2fToString( triVertices[i] )];
		baryCoord.push_back( make_pair( lambda[i], vertex ) );
	}

}

void Deformation::CalcBaryCoord2( int frameId, int e0, const Point2f &nextFramePos, vector<BaryCoord> &baryCoord ) {

	Point2f pointOrg, pointDst;
	vector<Point2f> biVertices;

	if ( frameSubdiv[frameId].edgeOrg( e0, &pointOrg ) > 0 && frameSubdiv[frameId].edgeDst( e0, &pointDst ) > 0 ) {
		biVertices.push_back( pointOrg );
		biVertices.push_back( pointDst );
	} else {
		printf( "[CalcBaryCoord2] Get points error: frameId %d, e0 %d, pointOrg %d, pointDst %d.\n", frameId, e0, frameSubdiv[frameId].edgeOrg( e0, &pointOrg ) << frameSubdiv[frameId].edgeDst( e0, &pointDst ) );
	}

	vector<double> lambda( 2 );
	
	CalcBaryCooordLambda( nextFramePos, biVertices, lambda );
	
	for ( int i = 0; i < 2; i++ ) {
		int vertex = posToPointIndexMap[frameId][Point2fToString( biVertices[i] )];
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

void Deformation::CalcBaryCoord1( int frameId, int vertex, vector<BaryCoord> &baryCoord ) {
	Point2f p = frameSubdiv[frameId].getVertex( vertex );
	int controlPointIndex = posToPointIndexMap[frameId][Point2fToString( p )];
	baryCoord.push_back( make_pair( 1, controlPointIndex ) );
}

int Deformation::LocateSubdivPoint( int frameId, const Point2f &p, vector<BaryCoord> &baryCoord ) {

	int e0, vertex, locateStatus;

	baryCoord.clear();

	locateStatus = frameSubdiv[frameId].locate( p, e0, vertex );

	switch ( locateStatus ) {
		case CV_PTLOC_INSIDE:
			CalcBaryCoord3( frameId, e0, p, baryCoord );
			break;
		case CV_PTLOC_ON_EDGE:
			CalcBaryCoord2( frameId, e0, p, baryCoord );
			break;
		case CV_PTLOC_VERTEX:
			CalcBaryCoord1( frameId, vertex, baryCoord );
			break;
		default:
			break;
	}

#ifdef DEBUG
	// DrawLocate( p, baryCoord );
#endif

	return locateStatus;
}

int Deformation::LocateNearestPoint( int frameId, const Point2f &p, vector<BaryCoord> &baryCoord ) {

	baryCoord.clear();

	vector< pair<double, int> > nearestPoints( 3, make_pair( INF, -1 ) );

	for ( size_t i = 0; i < frameControlPointIndex[frameId].size(); i++ ) {

		int controlPointIndex = frameControlPointIndex[frameId][i];
		ControlPoint controlPoint = controlPoints[controlPointIndex];
		double dist = NormL2( p, controlPoint.pos );
		pair<double, int> nearPoint(dist, controlPointIndex);

		for ( size_t j = 0; j < nearestPoints.size(); j++ ) {

			if ( nearPoint.first < nearestPoints[j].first ) {
				swap( nearPoint, nearestPoints[j] );
			}

		}

	}

#ifdef DEBUG
	/*for ( const auto &nearPoint : nearestPoints ) {
		cout << nearPoint.first << " " << nearPoint.second << endl;
	}*/
#endif

	if ( nearestPoints.back().first != INF ) {
		return CV_PTLOC_INSIDE;
	} else {
		return CV_PTLOC_ERROR;
	}

}

void Deformation::DelaunayDivide() {

	printf( "\tDelaunay divide each key frames.\n" );

	for ( int i = 0; i < frameNum; i++ ) {

		Rect rect( 0, 0, frameSize.width, frameSize.height );
		Subdiv2D subdiv( rect );

		for ( const auto &point : staticPoints ) {
			controlPoints.push_back( ControlPoint( i, point.first, point.second, -1, -1 ) );
			posToPointIndexMap[i][Point2fToString( point.first )] = controlPointsNum;
			controlPointsNum++;
		}

		for ( const auto &point : anchorPoints ) {
			
			int label = frames[i].pixelLabel.at<int>( point.first );
			double saliency = frames[i].superpixelSaliency[label];
			controlPoints.push_back( ControlPoint( i, point.first, point.second, label, saliency ) );

			subdiv.insert( point.first );
			posToPointIndexMap[i][Point2fToString( point.first )] = controlPointsNum;
			frameControlPointIndex[i].push_back( controlPointsNum );
			controlPointsNum++;

			//DrawSubdiv( frames[i].img, subdiv );
		}

		for ( int j = 0; j < frames[i].superpixelNum; j++ ) {

			if ( posToPointIndexMap[i].count( Point2fToString( frames[i].superpixelCenter[j] ) ) > 0 ) continue;

			double saliency = frames[i].superpixelSaliency[j];
			controlPoints.push_back( ControlPoint( i, frames[i].superpixelCenter[j], ControlPoint::ANCHOR_NONE, j, saliency ) );

			subdiv.insert( frames[i].superpixelCenter[j] );
			posToPointIndexMap[i][Point2fToString( frames[i].superpixelCenter[j] )] = controlPointsNum;
			frameControlPointIndex[i].push_back( controlPointsNum );
			controlPointsNum++;

			//DrawSubdiv( frames[i].img, subdiv );
		}

		frameSubdiv.push_back( subdiv );

	}

	//DrawSubdiv( frames[0].img, frameSubdiv[0] );

	printf( "\tControl point num: %d.\n", controlPoints.size() );

#ifdef DEBUG
	/*for ( auto &p : controlPoints ) {
		cout << p.frameId << " " << p.pos << endl;
		}*/
#endif

}

void Deformation::AddSpatialNeighbors() {

	printf( "\tAdd spatial neighbors to control points.\n" );

	for ( int i = 0; i < frameNum; i++ ) {

		vector<Vec4f> edgeList;
		frameSubdiv[i].getEdgeList( edgeList );
		map< string, int> edgeExist;

		for ( const auto &e : edgeList ) {

			Point2f p0( e.val[0], e.val[1] );
			Point2f p1( e.val[2], e.val[3] );

			if ( CheckOutside( p0, frameSize ) || CheckOutside( p1, frameSize ) ) {
				continue;
			}

			int index0 = posToPointIndexMap[i][Point2fToString( p0 )];
			int index1 = posToPointIndexMap[i][Point2fToString( p1 )];

			controlPoints[index0].AddSpatialNeighbor( index1 );
			controlPoints[index1].AddSpatialNeighbor( index0 );

			string edgeHash0 = to_string( index0 ) + " " + to_string( index1 );
			string edgeHash1 = to_string( index1 ) + " " + to_string( index0 );
			if ( edgeExist.count( edgeHash0 ) > 0 ) continue;
			if ( edgeExist.count( edgeHash1 ) > 0 ) continue;
			edgeExist[edgeHash0] = 1;
			edgeExist[edgeHash1] = 1;

			spatialEdge.push_back( make_pair( index0, index1 ) );

		}
	}

	printf( "\tSpatial edge num: %d.\n", spatialEdge.size() );

#ifdef DEBUG
	/*for ( auto &p : controlPoints ) {
		p.PrintSpatialNeighbors();
		}*/
#endif
}

void Deformation::AddTemporalNeighbors() {

	printf( "\tAdd temporal neighbors to control points.\n" );

	for ( auto &controlPoint : controlPoints ) {

		int nextFrameId = controlPoint.frameId + 1;

		if ( nextFrameId == frameNum ) continue;

		if ( controlPoint.anchorType != ControlPoint::ANCHOR_NONE ) continue;

		Point2f flow = frames[controlPoint.frameId].forwardFlowMap.at<Point2f>( Point2fToPoint( controlPoint.pos ) );
		Point2f nextFramePos;

#ifdef DEBUG
		//cout << controlPoint.frameId << " " << controlPoint.pos << Point2fToPoint( controlPoint.pos ) << endl;
		//cout << flow << endl;*/
#endif

		nextFramePos.x = controlPoint.pos.x + flow.x;
		nextFramePos.y = controlPoint.pos.y + flow.y;

		if ( CheckOutside( nextFramePos, frameSize ) ) continue;

		vector<BaryCoord> baryCoord;
		LocateSubdivPoint( nextFrameId, nextFramePos, baryCoord );

		controlPoint.AddTemporalNeighbor( baryCoord );

	}
}

void Deformation::BuildControlPoints() {

	printf( "Build key frames control points.\n" );

	DelaunayDivide();

	AddSpatialNeighbors();

	AddTemporalNeighbors();

}

void Deformation::InitDeformation( double _deformedScaleX, double _deformedScaleY ) {

	printf( "Initialize deformation.\n" );

	deformedScaleX = _deformedScaleX;
	deformedScaleY = _deformedScaleY;
	deformedFrameSize = Size( CeilToInt( frameSize.width * deformedScaleX ), CeilToInt( frameSize.height * deformedScaleY ) );

	for ( auto &controlPoint : controlPoints ) {
		if ( controlPoint.anchorType != ControlPoint::ANCHOR_STATIC ) {
			controlPoint.pos.x *= deformedScaleX;
			controlPoint.pos.y *= deformedScaleY;
		}
	}

#ifdef DEBUG
	//DrawEdge( ORIGIN_POS );
	//DrawEdge( DEFORMED_POS );
#endif

}

double Deformation::CalcEnergy() {

	double E, E_L, E_D;

	E = 0;

	E_L = 0;
	for ( const auto &e : spatialEdge ) {

		ControlPoint p0 = controlPoints[e.first];
		ControlPoint p1 = controlPoints[e.second];

		double saliencyWeight = p0.saliency + p1.saliency;
		double distortion = NormL2( p0.pos - p1.pos, p0.originPos - p1.originPos );

		E_L += saliencyWeight * distortion;

	}

	E_D = 0;

	E = E_L + E_D;

	return E;

}

void Deformation::MinimizeEnergy() {

	printf( "Minimize resize energy.\n" );

	double lambda = 1;
	double curE = CalcEnergy();
	printf( "\tIter 0. Energy: %.3lf. Learning rate: %.3lf.\n", curE, lambda );

	for ( int iter = 0; iter < MIN_ENERGY_ITERS; iter++ ) {

		vector<Point2f> newControlPoints( controlPointsNum );

		for ( int i = 0; i < controlPointsNum; i++ ) {
			newControlPoints[i] = controlPoints[i].pos;
		}

		for ( const auto &e : spatialEdge ) {
			ControlPoint p0 = controlPoints[e.first];
			ControlPoint p1 = controlPoints[e.second];

			double dnmtr = NormL2( p0.pos - p1.pos, p0.originPos - p1.originPos );
			
			if ( SignNumber( dnmtr ) == 0 ) continue;

			double saliency = p0.saliency + p1.saliency;
			Point2f mlclr = (p0.pos - p1.pos) - (p0.originPos - p1.originPos);

			newControlPoints[e.first] -= lambda * saliency / dnmtr * mlclr;
			newControlPoints[e.second] -= -lambda * saliency / dnmtr * mlclr;

		}

		for ( int i = 0; i < controlPointsNum; i++ ) {
			if ( controlPoints[i].anchorType == ControlPoint::ANCHOR_NONE ) {
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


Point2f Deformation::CalcPointByBaryCoord( const vector<BaryCoord> &baryCoord ) {

	Point2f deformedPoint( 0, 0 );

	for ( const auto &vertex : baryCoord ) {
		ControlPoint point = controlPoints[vertex.second];
		deformedPoint.x += vertex.first * point.pos.x;
		deformedPoint.y += vertex.first * point.pos.y;
	}

	return deformedPoint;

}

void Deformation::CalcDeformedMap() {

	clock_t timeSt = clock();

	deformedMap = vector<Mat>( frameNum, Mat( frameSize, CV_32SC2 ) );

	for ( int i = 0; i < frameNum; i++ ) {

		printf( "Calculate key frames deformed map. Progress rate %d/%d.\r", i + 1, frameNum );

		for ( int y = 0; y < frameSize.height; y++ ) {
			for ( int x = 0; x < frameSize.width; x++ ) {

				vector<BaryCoord> baryCoord;
				Point2f originPoint, deformedPoint;
				int locateStatus;

				originPoint = Point2f( x, y );
				locateStatus = LocateSubdivPoint( i, originPoint, baryCoord );
				deformedPoint = CalcPointByBaryCoord( baryCoord );

				deformedMap[i].at<Point>( originPoint ) = Point2fToPoint( deformedPoint);

#ifdef DEBUG
				//DrawLocate( originPoint, baryCoord );
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

	deformedImg = Mat::zeros( deformedFrameSize, CV_32SC3 );
	Mat mapCountMat = Mat::zeros( deformedFrameSize, CV_32SC1 );

	for ( int y = 0; y < frameSize.height; y++ ) {
		for ( int x = 0; x < frameSize.width; x++ ) {

			Point2f originPoint, deformedPoint;
			originPoint = Point( x, y );
			deformedPoint = deformedMap.at<Point>( originPoint );

#ifdef DEBUG
			/*		if ( CheckOutside( deformedPoint, frameSize ) ) {
			cout << "[RenderKeyFrames] Outside " << originPoint << " " << deformedPoint << endl;
			continue;
			}*/
#endif

			if ( CheckOutside( deformedPoint, deformedFrameSize ) ) continue;

			Vec3b pixelColor = img.at<Vec3b>( originPoint );
			deformedImg.at<Vec3i>( deformedPoint ) += pixelColor;
			mapCountMat.at<int>( deformedPoint ) = mapCountMat.at<int>( deformedPoint ) + 1;

		}
	}

	for ( int y = 0; y < deformedFrameSize.height; y++ ) {
		for ( int x = 0; x < deformedFrameSize.width; x++ ) {

			Point p( x, y );
			switch ( mapCountMat.at<int>( p ) ) {
				case 0: {

					int neighborNum = 0;
					Vec3i color( 0, 0, 0 );

					for ( int k = 0; k < DIRECTIONS_NUM; k++ ) {

						Point nextP = p + directions[k];
						if ( CheckOutside( nextP, deformedFrameSize ) ) continue;

						switch ( mapCountMat.at<int>( nextP ) ) {
							case 0:
								continue;
								break;
							case 1:
								neighborNum++;
								color += deformedImg.at<Vec3i>( nextP );
							default:
								deformedImg.at<Vec3i>( nextP ) /= mapCountMat.at<int>( nextP );
								mapCountMat.at<int>( nextP ) = 1;
								neighborNum++;
								color += deformedImg.at<Vec3i>( nextP );
								break;
						}

					}

					if ( neighborNum == 0 ) {
						cout << "[RenderFrame] Blur error: " << p << endl;
					} else {
						deformedImg.at<Vec3i>( p ) = color /= neighborNum;
					}

					break;
				}

				case 1:

					continue;
					break;

				default:

					deformedImg.at<Vec3i>( p ) /= mapCountMat.at<int>( p );
					mapCountMat.at<int>( p ) = 1;
					break;

			}

		}
	}

	deformedImg.convertTo( deformedImg, CV_8UC3 );

	imshow( "Deformed Frame", deformedImg );
	waitKey( 0 );

}

void Deformation::RenderKeyFrames() {

	printf( "Render key frames.\n" );

	for ( int i = 0; i < frameNum; i++ ) {

		Mat deformedFrame;

		RenderFrame( frames[i].img, deformedMap[i], deformedFrame );

	}

	DrawEdge( DEFORMED_POS );

}