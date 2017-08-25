# Video Resizing Documents

## Classes

### KeyFrame.h & KeyFrame.cpp

#### Private


  - Mat imgWithContours 
  - Mat spatialContrastMap, temporalContrastMap 
  - Mat paletteMap, paletteDist
  - vector\<Vec3f\> palette



​		





## Functions

## common.h & common.cpp

- bool **CheckEleExist**( const T( &eleArray )[N], const string &eleVal )

  Check whether the element is in the array.

- void **NormalizeVec**( vector\<T\> &vec )

  Normalize the value of the vector to scale [0, 1].

- double **NormL2**( const T &p )

  Calculate the L2 normalization of the vector p.

- bool **CheckOutside**( const T &p, const Size &size )

  Check whether the point is outside in the area.

- int **RoundToInt**( const T &d )

  Round the float number to integer.

- int **CeilToInt**( const T &d )

  Ceil the float number to integer.

- int **FloorToInt**( const T &d )

  Calculate the floor of the float number to integer.

- int **SignNumber**( const T d )

  Calculate the sign of the number.

- double **CrossProduct**( const T &p1, const T &p2 )

  Calculate the cross product result of the two vectors.

- double **DotProduct**( const T &p1, const T &p2 )

  Calculate the dot product result of the two vectors.

- double **CmpPairFirst**( const T &e0, const T &e1) 

  Boolean function for sort comparison by their first sub-element.

- double **CmpPairSecond**( const T &e0, const T &e1) 

  Boolean function for sort comparison by their first sub-element.

- void  **RestrictInside**( T &p, const Size &size )

  Restrict the point position by the specified area size.

- double **CalcVec3fDiff**( const Vec3f &, const Vec3f & )

  Calculate the difference of two Vec3fs.

- string **Point2fToString**( const Point2f & )

  Convert the Point2f to string.

- Point2f **StringToPoint2f**( const string & )

  Convert string to the Point2f.

- Point **Point2fToPoint**( const Point2f & )

  Convert Point2f to Point.

### io.h & io.cpp

- string **GetRootFolderPath**( const string &videoName )

  Get the root folder path of the testing video by its name.

- string **GetFramesFolderPath**( const string &videoName )

  Get the frames folder path of the testing video by its name.

- string **GetResultsFolderPath**( const string &videoName )

  Get the results folder path of the testing video by its name.

- string **GetKeyFramesFolderPath**( const string &videoName )

  Get the key frames folder path of the testing video by its name.

- string **GetOutputVideoPath**( const string &videoName, int type )

  Get the output video path of the testing video by its name and type.

- void **ConvertVideoToFrames**( const string &videoName )

  Convert the input video to a sequence of frames and save them in the folder.

- void **ReadShotCut**( vector\<int\> &, const string &videoName )

  Read the frame ID of shot cuts from shot cut file.

- void **ReadKeyArr**( vector\<int\> &, const string &videoName )

  Read the frame ID of key frames from key frame file.

- void **ReadKeyFrames**( int, int, const vector\<int\> &, vector\<KeyFrame\> &, const string &videoName )

  Read the key frames by key frame array from key frames folder.

- void **ReadFrames**( int, int, vector\<Mat\> &, const string &videoName  )

  Read the frames from frames folder.

- void **WriteKeyFrameEdgeImg**( int frameId, const Mat &edgeImg, const string &videoName )

  Save the key frame edge image in image file.

- void **WriteDeformedImg**( int frameId, const Mat &img, const string &videoName )

  Save the deformed image in the image file.

- void **WriteResizedVideo**( const string &videoName )

  Save the resized video in the video file.

- void **WriteMixedVideo**( const string &videoName, double deformedScaleX, double deformedScaleY )

  Save the mixed video in the video file.

### pretreat.h & pretreat.cpp

- void **SegFramesToShotCutKeyFrames**( const string &videoName )

  Segment the frames to shot cuts and key frames.

- void **CalcSuperpixel**( vector\<KeyFrame\> & )

  Calculate the superpixels of each key frames.

- bool **CmpVec3f0**( const Vec3f &, const Vec3f & )

  Boolean function for sort comparison by their first sub-element.

- bool **CmpVec3f1**( const Vec3f &, const Vec3f & )

  Boolean function for sort comparison by their second sub-element.

- bool **CmpVec3f2**( const Vec3f &, const Vec3f & )

  Boolean function for sort comparison by their third sub-element.

- void **CalcPalette**( const vector\<KeyFrame\> &, vector\<Vec3f\> & )

  Calculate the quantitative palette of all key frames.

- void **QuantizeFrames**( vector\<KeyFrame\> & )

  Quantize the color space of key frames.

- void **DetectSpatialStructure**( vector\<KeyFrame\> & )

  Detect the spatial structure of key frames.

### saliency.h & saliency.cpp

- void **DrawOpticalFlow**( const Mat &, const Mat & )

  Show the optical flow of each frame.

- void **CalcMotion**( const Mat &, Mat &, Point2f & )

  Calculate the motion of pixels between two continuous frames.

- void **CalcSaliencyMap**( vector\<KeyFrame\> & )

  Calculate the saliency map of the frame.

- void **SmoothSaliencyMap**( vector\<KeyFrame\> & )

  Smooth the saliency maps.