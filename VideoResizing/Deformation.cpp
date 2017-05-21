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

	for (const auto &controlPointIndex: frameControlPointIndex[frameId] ) {
		
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

Point2f Deformation::GetBoundPoint( int index0, int index1 ) {

	ControlPoint p0 = controlPoints[index0];
	ControlPoint p1 = controlPoints[index1];

	if ( p0.saliency < p1.saliency ) {
		swap( p0, p1 );
	}

	int superpixelIndex = p0.superpixelIndex;
	int frameId = p0.frameId;
	Point2f boundPoint( -1, -1 );

	if ( SignNumber( p0.pos.x - p1.pos.x ) == 0 ) {

		int x = p0.pos.x;
		if ( p0.pos.y < p1.pos.y ) {
			for ( int y = p0.pos.y; y < p1.pos.y; y++ ) {
				if ( frames[frameId].pixelLabel.at<int>( y, x ) != superpixelIndex ) {
					boundPoint = Point( x, y );
					break;
				}
			}
		} else {
			for ( int y = p0.pos.y; y > p1.pos.y; y-- ) {
				if ( frames[frameId].pixelLabel.at<int>( y, x ) != superpixelIndex ) {
					boundPoint = Point( x, y );
					break;
				}
			}
		}

	} else {

		int dx = 1;
		double k = (p1.pos.y - p0.pos.y) / abs( p0.pos.x - p1.pos.x );
		if ( p0.pos.x > p1.pos.x ) dx = -1;

		if ( dx > 0 ) {

			for ( int x = p0.pos.x; x < p1.pos.x; x += dx ) {

				int y = RoundToInt( p0.pos.y + k * abs( x - p0.pos.x ) );
				int tmpY = RoundToInt( p0.pos.y + k * abs( x + dx - p0.pos.x ));

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

			for ( int x = p0.pos.x; x > p1.pos.x; x += dx ) {

				int y = RoundToInt( p0.pos.y + k * abs( x - p0.pos.x ) );
				int tmpY = RoundToInt( p0.pos.y + k * abs( x + dx - p0.pos.x ) );

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

#ifdef DEBUG
			// DrawSubdiv( frames[i].img, subdiv );
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

#ifdef DEBUG
		//DrawEdge( i, ORIGIN_POS_WITH_FRAME );
		//waitKey( 0 );
#endif

	}

	printf( "\tControl point num: %d.\n", controlPoints.size() );

#ifdef DEBUG
	/*for ( auto &p : controlPoints ) {
		cout << p.frameId << " " << p.pos << endl;
		}*/
#endif

}

void Deformation::CalcBaryCooordLambda( const Point2f &p, const vector<Point2f> &vertices, vector<double> &lambda ) {

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

int Deformation::LocateNearestPoint( int frameId, const Point2f &p, vector<BaryCoord> &baryCoord ) {

	vector< pair<double, int> > nearestPoints( 3, make_pair( INF, -1 ) );
	int label = frames[frameId].pixelLabel.at<int>( p );
	int centerPointIndex = frameControlPointIndex[frameId][label];

	double dist = NormL2( p, controlPoints[centerPointIndex].pos );
	pair<double, int> nearPoint( dist, centerPointIndex );

	for ( size_t j = 0; j < nearestPoints.size(); j++ ) {

		if ( nearPoint.first < nearestPoints[j].first ) {
			swap( nearPoint, nearestPoints[j] );
		}

	}

	for ( const auto &neighborIndex : controlPoints[centerPointIndex].spatialBound ) {
		dist = NormL2( p, controlPoints[neighborIndex].pos );
		nearPoint = make_pair( dist, neighborIndex );

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

	if ( nearestPoints[2].first == INF ) nearestPoints.resize( 2 );

	vector<double> lambda;
	vector<Point2f> vertices;
	for ( size_t i = 0; i < nearestPoints.size(); i++ ) {
		lambda.push_back( 0 );
		vertices.push_back( controlPoints[nearestPoints[i].second].pos );
	}

	CalcBaryCooordLambda( p, vertices, lambda );

	baryCoord.clear();
	for ( size_t i = 0; i < nearestPoints.size(); i++ ) {
		baryCoord.push_back( make_pair( lambda[i], nearestPoints[i].second ) );
	}

#ifdef DEBUG
	/*for ( size_t i = 0; i < nearestPoints.size(); i++ ) {
		cout << vertices[i] << " " << lambda[i] << " ";
	}
	Point2f tmpPoint = CalcPointByBaryCoord( baryCoord );
	cout << endl << p << " " << tmpPoint << endl;*/
#endif

}

void Deformation::AddTemporalNeighbors() {

	printf( "\tAdd temporal neighbors to control points.\n" );

	for ( auto &controlPoint : controlPoints ) {

		int nextFrameId = controlPoint.frameId + 1;

		if ( nextFrameId == frameNum ) continue;

		if ( controlPoint.anchorType == ControlPoint::ANCHOR_STATIC ) continue;

		Point2f flow = frames[controlPoint.frameId].forwardFlowMap.at<Point2f>( Point2fToPoint( controlPoint.pos ) );
		Point2f nextFramePos = controlPoint.pos + flow;

		controlPoint.flow = flow;

#ifdef DEBUG
		// cout << controlPoint.frameId << " " << controlPoint.pos << Point2fToPoint( controlPoint.pos ) << endl;
		// cout << flow << endl;
#endif

		if ( CheckOutside( nextFramePos, frameSize ) ) continue;

		vector<BaryCoord> baryCoord;
		LocateNearestPoint( nextFrameId, nextFramePos, baryCoord );

		controlPoint.AddTemporalNeighbor( baryCoord );

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
//
//double Deformation::CalcEnergy() {
//
//	double E, E_L, E_D;
//
//	E = 0;
//
//	E_L = 0;
//	for ( const auto &e : spatialEdge ) {
//
//		ControlPoint p0 = controlPoints[e.first];
//		ControlPoint p1 = controlPoints[e.second];
//
//		double saliencyWeight = p0.saliency + p1.saliency;
//		double distortion = NormL2( p0.pos - p1.pos, p0.originPos - p1.originPos );
//
//		E_L += saliencyWeight * distortion;
//
//	}
//
//	E_D = 0;
//
//	E = E_L + E_D;
//
//	return E;
//
//}
//
//void Deformation::MinimizeEnergy() {
//
//	printf( "Minimize resize energy.\n" );
//
//	double lambda = 1;
//	double curE = CalcEnergy();
//	printf( "\tIter 0. Energy: %.3lf. Learning rate: %.3lf.\n", curE, lambda );
//
//	for ( int iter = 0; iter < MIN_ENERGY_ITERS; iter++ ) {
//
//		vector<Point2f> newControlPoints( controlPointsNum );
//
//		for ( int i = 0; i < controlPointsNum; i++ ) {
//			newControlPoints[i] = controlPoints[i].pos;
//		}
//
//		for ( const auto &e : spatialEdge ) {
//			ControlPoint p0 = controlPoints[e.first];
//			ControlPoint p1 = controlPoints[e.second];
//
//			double dnmtr = NormL2( p0.pos - p1.pos, p0.originPos - p1.originPos );
//			
//			if ( SignNumber( dnmtr ) == 0 ) continue;
//
//			double saliency = p0.saliency + p1.saliency;
//			Point2f mlclr = (p0.pos - p1.pos) - (p0.originPos - p1.originPos);
//
//			newControlPoints[e.first] -= lambda * saliency / dnmtr * mlclr;
//			newControlPoints[e.second] -= -lambda * saliency / dnmtr * mlclr;
//
//		}
//
//		for ( int i = 0; i < controlPointsNum; i++ ) {
//			if ( controlPoints[i].anchorType == ControlPoint::ANCHOR_NONE ) {
//				controlPoints[i].pos = newControlPoints[i];
//				RestrictInside( controlPoints[i].pos, deformedFrameSize );
//			}
//		}
//
//		double preE = curE;
//		curE = CalcEnergy();
//		if ( curE >= preE ) {
//			lambda /= 2;
//		}
//
//		if ( lambda < ITER_TERMINATE ) break;
//
//		printf( "\tIter %d. Energy: %.3lf. Learning rate: %.3lf.\n", iter + 1, curE, lambda );
//
//		
//	}
//}
//

Point2f Deformation::CalcPointByBaryCoord( const vector<BaryCoord> &baryCoord ) {

	Point2f deformedPoint( 0, 0 );

	for ( const auto &vertex : baryCoord ) {
		ControlPoint point = controlPoints[vertex.second];
		deformedPoint.x += vertex.first * point.pos.x;
		deformedPoint.y += vertex.first * point.pos.y;
	}

	return deformedPoint;

}

//void Deformation::CalcDeformedMap() {
//
//	clock_t timeSt = clock();
//
//	deformedMap = vector<Mat>( frameNum, Mat( frameSize, CV_32SC2 ) );
//
//	for ( int i = 0; i < frameNum; i++ ) {
//
//		printf( "Calculate key frames deformed map. Progress rate %d/%d.\r", i + 1, frameNum );
//
//		for ( int y = 0; y < frameSize.height; y++ ) {
//			for ( int x = 0; x < frameSize.width; x++ ) {
//
//				vector<BaryCoord> baryCoord;
//				Point2f originPoint, deformedPoint;
//				int locateStatus;
//
//				originPoint = Point2f( x, y );
//				locateStatus = LocateSubdivPoint( i, originPoint, baryCoord );
//				deformedPoint = CalcPointByBaryCoord( baryCoord );
//
//				deformedMap[i].at<Point>( originPoint ) = Point2fToPoint( deformedPoint);
//
//#ifdef DEBUG
//				//DrawLocate( originPoint, baryCoord );
//#endif
//			}
//		}
//
//	}
//
//	printf( "\n" );
//	clock_t timeEd = clock();
//	printf( "Calculate key frames deformed map. Time used %ds.\n", (timeEd - timeSt) / 1000 );
//
//}
//
//void Deformation::RenderFrame( const Mat &img, const Mat &deformedMap, Mat &deformedImg ) {
//
//	Size frameSize = img.size();
//
//	deformedImg = Mat::zeros( deformedFrameSize, CV_32SC3 );
//	Mat mapCountMat = Mat::zeros( deformedFrameSize, CV_32SC1 );
//
//	for ( int y = 0; y < frameSize.height; y++ ) {
//		for ( int x = 0; x < frameSize.width; x++ ) {
//
//			Point2f originPoint, deformedPoint;
//			originPoint = Point( x, y );
//			deformedPoint = deformedMap.at<Point>( originPoint );
//
//#ifdef DEBUG
//			/*		if ( CheckOutside( deformedPoint, frameSize ) ) {
//			cout << "[RenderKeyFrames] Outside " << originPoint << " " << deformedPoint << endl;
//			continue;
//			}*/
//#endif
//
//			if ( CheckOutside( deformedPoint, deformedFrameSize ) ) continue;
//
//			Vec3b pixelColor = img.at<Vec3b>( originPoint );
//			deformedImg.at<Vec3i>( deformedPoint ) += pixelColor;
//			mapCountMat.at<int>( deformedPoint ) = mapCountMat.at<int>( deformedPoint ) + 1;
//
//		}
//	}
//
//	for ( int y = 0; y < deformedFrameSize.height; y++ ) {
//		for ( int x = 0; x < deformedFrameSize.width; x++ ) {
//
//			Point p( x, y );
//			switch ( mapCountMat.at<int>( p ) ) {
//				case 0: {
//
//					int neighborNum = 0;
//					Vec3i color( 0, 0, 0 );
//
//					for ( int k = 0; k < DIRECTIONS_NUM; k++ ) {
//
//						Point nextP = p + directions[k];
//						if ( CheckOutside( nextP, deformedFrameSize ) ) continue;
//
//						switch ( mapCountMat.at<int>( nextP ) ) {
//							case 0:
//								continue;
//								break;
//							case 1:
//								neighborNum++;
//								color += deformedImg.at<Vec3i>( nextP );
//							default:
//								deformedImg.at<Vec3i>( nextP ) /= mapCountMat.at<int>( nextP );
//								mapCountMat.at<int>( nextP ) = 1;
//								neighborNum++;
//								color += deformedImg.at<Vec3i>( nextP );
//								break;
//						}
//
//					}
//
//					if ( neighborNum == 0 ) {
//						cout << "[RenderFrame] Blur error: " << p << endl;
//					} else {
//						deformedImg.at<Vec3i>( p ) = color /= neighborNum;
//					}
//
//					break;
//				}
//
//				case 1:
//
//					continue;
//					break;
//
//				default:
//
//					deformedImg.at<Vec3i>( p ) /= mapCountMat.at<int>( p );
//					mapCountMat.at<int>( p ) = 1;
//					break;
//
//			}
//
//		}
//	}
//
//	deformedImg.convertTo( deformedImg, CV_8UC3 );
//
//	imshow( "Deformed Frame", deformedImg );
//	waitKey( 0 );
//
//}
//
//void Deformation::RenderKeyFrames() {
//
//	printf( "Render key frames.\n" );
//
//	for ( int i = 0; i < frameNum; i++ ) {
//
//		Mat deformedFrame;
//
//		RenderFrame( frames[i].img, deformedMap[i], deformedFrame );
//
//		deformedFrames.push_back( deformedFrame );
//
//	}
//
//	DrawEdge( DEFORMED_POS );
//
//}