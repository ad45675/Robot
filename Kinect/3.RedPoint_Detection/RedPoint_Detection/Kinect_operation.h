#ifndef KIENCT_OPERATION
#define KINECT_OPERATION

#include "Setting.h"

class Kinect{

public:
	bool OpenKinect();
	bool Capture_Color(Mat &);
	bool Capture_Depth(Mat &);
	bool Capture_Body(double RightHandPos[3]);
	bool CloseKinect();

	IKinectSensor* pSensor;
	IColorFrameReader* pColorFrameReader;
	IDepthFrameReader* pDepthFrameReader;
	IBodyFrameReader*  pBodyFrameReader;
	ICoordinateMapper* pCoordinateMapper;


	int iColorWidth, iColorHeight;
	Size imgColorSize;
	UINT uColorBufferSize;
	UINT uColorPointNum;

	int iDepthWidth, iDepthHeight;
	UINT uDepthPointNum;
	UINT16 uDepthMin, uDepthMax;

	UINT16*				pDepthPoints;
	DepthSpacePoint*	pPointArray;

	CameraIntrinsics* m_pCameraIntrinsics;

	INT32 iBodyCount;
	IBody** aBody;
};

#endif

