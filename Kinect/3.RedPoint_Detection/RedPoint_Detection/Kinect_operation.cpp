#include "Kinect_operation.h"

bool Kinect::OpenKinect()
{
	//  -------------------- Kinect Initialization --------------------
	cout << "Try to get default sensor..." << endl;
	pSensor = nullptr;

	{
		if (GetDefaultKinectSensor(&pSensor) != S_OK)
		{
			cerr << "Get Sensor failed" << endl;
			return false;
		}

		cout << "Try to open sensor..." << endl;
		if (pSensor->Open() != S_OK)
		{
			cerr << "Can't open sensor" << endl;
			return false;
		}
	}

	//  -------------------- Color Frame Initialization -------------------
	pColorFrameReader = nullptr;
	cout << "Try to get color source..." << endl;
	{
		// Get frame source
		IColorFrameSource* pColorFrameSource = nullptr;
		if (pSensor->get_ColorFrameSource(&pColorFrameSource) != S_OK)
		{
			cerr << "Can't get color frame source" << endl;
			return false;
		}

		// Get frame description
		cout << "Try to get color frame description..." << endl;
		IFrameDescription* pFrameDescription = nullptr;
		if (pColorFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
		{
			pFrameDescription->get_Width(&iColorWidth);
			pFrameDescription->get_Height(&iColorHeight);

			uColorPointNum = iColorWidth * iColorHeight;
			uColorBufferSize = uColorPointNum * 4 * sizeof(BYTE);
		}
		pFrameDescription->Release();
		pFrameDescription = nullptr;

		imgColorSize = Size(iColorWidth, iColorHeight);
		// get frame reader
		cout << "Try to get color frame reader..." << endl;
		if (pColorFrameSource->OpenReader(&pColorFrameReader) != S_OK)
		{
			cerr << "Can't get color frame reader" << endl;
			return false;
		}

		// release Frame source
		cout << "Release frame source..." << endl;
		pColorFrameSource->Release();
		pColorFrameSource = nullptr;


		printf("Color Frame-\nWidth: %d\nHeight: %d\n\n", iColorWidth, iColorHeight);
	}

	//  -------------------- Depth Frame Initialization -------------------

	cout << "Try to get depth source..." << endl;
	{
		// Get frame source
		IDepthFrameSource* pDepthFrameSource = nullptr;
		if (pSensor->get_DepthFrameSource(&pDepthFrameSource) != S_OK)
		{
			cerr << "Can't get depth frame source" << endl;
			return false;
		}

		// Get frame description
		cout << "Try to get depth frame description..." << endl;
		IFrameDescription* pFrameDescription = nullptr;
		if (pDepthFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
		{
			pFrameDescription->get_Width(&iDepthWidth);
			pFrameDescription->get_Height(&iDepthHeight);
			uDepthPointNum = iDepthWidth * iDepthHeight;
		}
		pFrameDescription->Release();
		pFrameDescription = nullptr;


		// get reliable distance
		if (pDepthFrameSource->get_DepthMinReliableDistance(&uDepthMin) != S_OK)
		{
			cerr << "Can't get reliable Min distance" << endl;
			return false;
		}

		if (pDepthFrameSource->get_DepthMaxReliableDistance(&uDepthMax) != S_OK)
		{
			cerr << "Can't get reliable Max distance" << endl;
			return false;
		}

		// get frame reader

		cout << "Try to get depth frame reader..." << endl;

		pDepthFrameReader = nullptr;
		if (pDepthFrameSource->OpenReader(&pDepthFrameReader) != S_OK)
		{
			cerr << "Can't get depth frame reader" << endl;
			return false;
		}

		// release Frame source
		cout << "Release frame source..." << endl;
		pDepthFrameSource->Release();
		pDepthFrameSource = nullptr;


		printf("Depth Frame-\nWidth: %d\nHeight: %d\n", iDepthWidth, iDepthHeight);
		cout << "Reliable Distance: "
			<< uDepthMin << " – " << uDepthMax << endl << endl;

	}

	//  -------------------- Coordinate Mapper Initialization -------------------

	cout << "Try to get coordinate mapper..." << endl;

	{

		pCoordinateMapper = nullptr;
		if (pSensor->get_CoordinateMapper(&pCoordinateMapper) != S_OK)
		{
			cerr << "get CoordinateMapper failed" << endl;
			return false;
		}

		pDepthPoints = new UINT16[uDepthPointNum];
		pPointArray = new DepthSpacePoint[uColorPointNum];

	}

	// -------------------- Depth Intrinsic Initial -------------------

	m_pCameraIntrinsics = new CameraIntrinsics(); // 取得深度camera內部參數

												  // -------------------- Body Inframe Initial ----------------------

	cout << "Try to get body source..." << endl;
	{
		// Get frame source
		IBodyFrameSource* pBodyFrameSource = nullptr;
		if (pSensor->get_BodyFrameSource(&pBodyFrameSource) != S_OK)
		{
			cerr << "Can't get body frame source" << endl;
			return false;
		}

		// Get the number of body

		cout << "Try to get body count..." << endl;

		iBodyCount = 0;
		if (pBodyFrameSource->get_BodyCount(&iBodyCount) != S_OK)
		{
			cerr << "Can't get body count" << endl;
			return false;
		}
		cout << "Can trace " << iBodyCount << " bodies" << endl;
		aBody = new IBody*[iBodyCount];
		for (int i = 0; i < iBodyCount; ++i)
			aBody[i] = nullptr;


		// Get frame reader

		cout << "Try to get body frame reader..." << endl;
		pBodyFrameReader = nullptr;

		if (pBodyFrameSource->OpenReader(&pBodyFrameReader) != S_OK)
		{
			cerr << "Can't get body frame reader" << endl;
			return false;
		}

		// Release Frame source
		cout << "Release frame source..." << endl;
		pBodyFrameSource->Release();
		pBodyFrameSource = nullptr;

	}


	return true;
}

bool Kinect::Capture_Color(Mat& mColorImg)
{
	IColorFrame* pColorFrame = nullptr;

	if (pColorFrameReader->AcquireLatestFrame(&pColorFrame) != S_OK)
		return false;

	// Read color frame
	{
		pColorFrame->CopyConvertedFrameDataToArray(uColorBufferSize, mColorImg.data, ColorImageFormat_Bgra);
		pColorFrame->Release();
		pColorFrame = nullptr;
	}


	return true;
}

bool Kinect::Capture_Depth(Mat& mDepthImg)
{
	IDepthFrame* pDepthFrame = nullptr;

	if (pDepthFrameReader->AcquireLatestFrame(&pDepthFrame) != S_OK)
		return false;

	// Read depth frame
	{
		pDepthFrame->CopyFrameDataToArray(uDepthPointNum, pDepthPoints);
		pDepthFrame->CopyFrameDataToArray(iDepthWidth * iDepthHeight, reinterpret_cast<UINT16*>(mDepthImg.data)); //存取各元素值需要使用Mat type::<short>

		pDepthFrame->Release();
		pDepthFrame = nullptr;
	}

	return true;
}

bool Kinect::Capture_Body(double RightHandPos[3])
{
	IBodyFrame* pBodyFrame = nullptr;

	if (pBodyFrameReader->AcquireLatestFrame(&pBodyFrame) != S_OK)
		return false;

	// Get Body data
	if (pBodyFrame->GetAndRefreshBodyData(iBodyCount, aBody) == S_OK)
	{
		int iTrackedBodyCount = 0;

		// For each body
		for (int i = 0; i < iBodyCount; i++)
		{
			IBody* pBody = aBody[i];

			// Check if is tracked
			BOOLEAN bTracked = false;
			if ((pBody->get_IsTracked(&bTracked) == S_OK) && bTracked)
			{
				++iTrackedBodyCount;
				cout << "User " << i << " is under tracking" << endl;

				// Get joint position
				Joint aJoints[JointType::JointType_Count];
				if (pBody->GetJoints(JointType::JointType_Count, aJoints) != S_OK)
				{
					cerr << "Get joints fail" << endl;
				}

				// Get joint orientation
				JointOrientation aOrientations[JointType::JointType_Count];
				if (pBody->GetJointOrientations(JointType::JointType_Count, aOrientations) != S_OK)
				{
					cerr << "Get joints fail" << endl;
				}

				// Output information
				JointType eJointType = JointType::JointType_HandRight;
				const Joint& rJointPos = aJoints[eJointType];
				const JointOrientation& rJointOri = aOrientations[eJointType];

				cout << " > Right Hand is ";
				if (rJointPos.TrackingState == TrackingState_NotTracked)
				{
					cout << "not tracked" << endl;
				}
				else
				{
					//if (rJointPos.TrackingState == TrackingState_Inferred)
					//{
					//	cout << "inferred ";
					//}
					//else if (rJointPos.TrackingState == TrackingState_Tracked)
					//{
					//	cout << "tracked ";
					//}

					//cout << "At [ " << rJointPos.Position.X << " , " << rJointPos.Position.Y << " , " << rJointPos.Position.Z << " ] " << ",\n\t orientation: " << rJointOri.Orientation.w << " " << rJointOri.Orientation.x << " " << rJointOri.Orientation.y << " " << rJointOri.Orientation.z << endl;



					RightHandPos[0] = rJointPos.Position.X;
					RightHandPos[1] = rJointPos.Position.Y;
					RightHandPos[2] = rJointPos.Position.Z;

				}
			}
		}

		/*if (iTrackedBodyCount > 0)
		cout << "Total " << iTrackedBodyCount << " bodies in this time\n" << endl;*/
	}


	// Release frame
	pBodyFrame->Release();


	return true;

}

bool Kinect::CloseKinect()
{
	// Delete pointer
	delete[] pPointArray;
	delete[] pDepthPoints;
	delete[] m_pCameraIntrinsics;

	// release coordinate mapper
	cout << "Release coordinate mapper" << endl;
	pCoordinateMapper->Release();
	pCoordinateMapper = nullptr;

	// release frame reader
	cout << "Release frame reader" << endl;
	pColorFrameReader->Release();
	pColorFrameReader = nullptr;
	pDepthFrameReader->Release();
	pDepthFrameReader = nullptr;
	pBodyFrameReader->Release();
	pBodyFrameReader = nullptr;

	// Close Sensor
	cout << "close sensor" << endl;
	pSensor->Close();

	// Release Sensor
	cout << "Release sensor" << endl;
	pSensor->Release();
	pSensor = nullptr;

	return true;
}
