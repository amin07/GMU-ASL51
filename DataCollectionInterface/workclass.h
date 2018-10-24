///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////                          University of Bristol                          ////////////////
//////////////                       Computer Science Department                       ////////////////
//===================================================================================================//
///////                            This is an open source code for:                             ///////
////////           "3D Data Acquisition and Registration using Two Opposing Kinects"         //////////
////////////////      V. Soleimani, M. Mirmehdi, D. Damen, S. Hannuna, M. Camplani    /////////////////
////////////////         International Conference on 3D Vision, Stanford, 2016        /////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once
#include "KinectFunctions.h"
#include <opencv2/videoio/videoio_c.h>
using namespace cv;

class workclass
{

public:
	workclass(void)
	{
	}

	~workclass(void)
	{
	}

	
	static bool writeSkeleton()
	{
		Skeleton * tmp;
		if (bufferSkelSave.try_pop(tmp))
		{
			//generate the time stamp string
			char myDate[12];
			sprintf(myDate, "%.2hd%.2hd%.2hd%.3hd", tmp->st.wHour, tmp->st.wMinute, tmp->st.wSecond, tmp->st.wMilliseconds);

			fprintf(m_fpSkel, "%s %d %lld %g ", myDate, m_iBodyWriteCnt, tmp->i64DeviceTimeStamp, (tmp->tbbtcNativeTimeStamp - m_tcStart).seconds());
			for (int i = 0; i < 25; i++)
			{
				fprintf(m_fpSkel, "%d %f %f %f ", tmp->TrackingState[i], tmp->x[i], tmp->y[i], tmp->z[i]);

			}
			fprintf(m_fpSkel, "\n");

			//Free memory
			free(tmp);
			m_iBodyWriteCnt++;
			return true;
		}
		return false;
	}

	static bool writeDepth()
	{
		ImageStruct * tmp;

		if (bufferDepthSave.try_pop(tmp))
		{
			sprintf(m_cpDepthfilepathTmp, "%s\\depth%s%d.png",
				cpFullDirectPathDepth,
				(m_iDepthWriteCnt<10) ? "0000" : (m_iDepthWriteCnt<100) ? "000" : (m_iDepthWriteCnt<1000) ? "00" : (m_iDepthWriteCnt<10000) ? "0" : "",
				m_iDepthWriteCnt);

			imwrite(m_cpDepthfilepathTmp, *(tmp->matrix));


			//Now write in metadata file
			char myDate[12];
			sprintf(myDate, "%.2hd%.2hd%.2hd%.3hd", tmp->st.wHour, tmp->st.wMinute, tmp->st.wSecond, tmp->st.wMilliseconds);
			fprintf(m_fpDepthMeta, "%s, %d, %lld, %g\n", myDate, m_iDepthWriteCnt, tmp->i64DeviceTimeStamp, (tmp->tbbtcNativeTimeStamp - m_tcStart).seconds());

			//Free memory
			tmp->matrix->release();
			free(tmp);

			m_iDepthWriteCnt++;

			return true;
		}
		return false;
	}


	static bool writeRGB()
	{
		ImageStruct * tmp;

		if (bufferRGBSave.try_pop(tmp))
		{
			cv::Size sz;

			sz.height = (int)(((float)tmp->matrix->rows) * m_fRGBscaleFactor);
			sz.width = (int)(((float)tmp->matrix->cols) * m_fRGBscaleFactor);

			resize(*(tmp->matrix), *(tmp->matrix), sz);

			if (m_bUseBMPencode)
			{
				sprintf(m_cpRGBfilepathTmp, "%s\\rgb%s%d.bmp",
					cpFullDirectPathRGB,
					(m_iColourWriteCnt<10) ? "0000" : (m_iColourWriteCnt<100) ? "000" : (m_iColourWriteCnt<1000) ? "00" : (m_iColourWriteCnt<10000) ? "0" : "",
					m_iColourWriteCnt);
				imwrite(m_cpRGBfilepathTmp, *(tmp->matrix));
			}
			else
			{
				sprintf(m_cpRGBfilepathTmp, "%s\\rgb%s%d.jpg",
					cpFullDirectPathRGB,
					(m_iColourWriteCnt<10) ? "0000" : (m_iColourWriteCnt<100) ? "000" : (m_iColourWriteCnt<1000) ? "00" : (m_iColourWriteCnt<10000) ? "0" : "",
					m_iColourWriteCnt);
				std::vector<int> qualityType;
				qualityType.push_back(CV_IMWRITE_JPEG_QUALITY);
				qualityType.push_back(95);
				imwrite(m_cpRGBfilepathTmp, *(tmp->matrix), qualityType);
			}


			//Now write in metadata file
			char myDate[12];
			sprintf(myDate, "%.2hd%.2hd%.2hd%.3hd", tmp->st.wHour, tmp->st.wMinute, tmp->st.wSecond, tmp->st.wMilliseconds);
			fprintf(m_fpRGBMeta, "%s, %d, %lld, %g\n", myDate, m_iColourWriteCnt, tmp->i64DeviceTimeStamp, (tmp->tbbtcNativeTimeStamp - m_tcStart).seconds());

			//Free memo
			tmp->matrix->release();
			free(tmp);

			m_iColourWriteCnt++;

			return true;
		}
		return false;
	}

	static bool myPushDepthSave(Mat *depthImSave, INT64 ts, tbb::tick_count nts)
	{
		ImageStruct * tmp = new ImageStruct();

		tmp->matrix = new Mat(depthImSave->rows, depthImSave->cols, CV_16U);
		int dataSize = depthImSave->rows*depthImSave->cols*((int)depthImSave->elemSize());

		memcpy(tmp->matrix->data, depthImSave->data, (size_t)dataSize);

		tmp->i64DeviceTimeStamp = ts;
		tmp->tbbtcNativeTimeStamp = nts;
		GetSystemTime(&(tmp->st));

		bufferDepthSave.push(tmp);

		return true;
	}

	static bool myPushRGBSave(Mat *rgbImSave, INT64 ts, tbb::tick_count nts)
	{
		ImageStruct * tmp = new ImageStruct();

		tmp->matrix = new Mat(rgbImSave->rows, rgbImSave->cols, CV_8UC4);
		int dataSize = rgbImSave->rows*rgbImSave->cols*((int)rgbImSave->elemSize());

		memcpy(tmp->matrix->data, rgbImSave->data, (size_t)dataSize);

		tmp->i64DeviceTimeStamp = ts;
		tmp->tbbtcNativeTimeStamp = nts;
		GetSystemTime(&(tmp->st));
		bufferRGBSave.push(tmp);

		return true;
	}

	static bool myPushSkelSaveScreen(Joint joints[], INT64 ts, tbb::tick_count nts, cv::Point m_2dpScreenCoords[JointType_Count])
	{
		Skeleton * tmp = new Skeleton();

		for (int j = 0; j < JointType_Count && j < 25; ++j)
		{
			if (joints[j].TrackingState != TrackingState_Inferred && joints[j].TrackingState != TrackingState_Tracked)
			{
				tmp->x[j] = tmp->y[j] = tmp->z[j] = -10000;
				tmp->TrackingState[j] = 0;
			}
			else
			{

				tmp->x[j] = (float)(m_2dpScreenCoords[j].x);
				tmp->y[j] = (float)(m_2dpScreenCoords[j].y);
				tmp->z[j] = joints[j].Position.Z;
				tmp->TrackingState[j] = joints[j].TrackingState;
			}
		}
		tmp->i64DeviceTimeStamp = ts;
		tmp->tbbtcNativeTimeStamp = nts;
		GetSystemTime(&(tmp->st));

		bufferSkelSave.push(tmp);

		return true;
	}


	static bool myPushSkelSave(Joint joints[], INT64 ts, tbb::tick_count nts)
	{
		Skeleton * tmp = new Skeleton();

		for (int j = 0; j < JointType_Count && j < 25; ++j)
		{
			if (joints[j].TrackingState != TrackingState_Inferred && joints[j].TrackingState != TrackingState_Tracked)
			{
				tmp->x[j] = tmp->y[j] = tmp->z[j] = -10000;
				tmp->TrackingState[j] = 0;
			}
			else
			{
				tmp->x[j] = joints[j].Position.X;
				tmp->y[j] = joints[j].Position.Y;
				tmp->z[j] = joints[j].Position.Z;
				tmp->TrackingState[j] = joints[j].TrackingState;
			}
		}
		tmp->i64DeviceTimeStamp = ts;
		tmp->tbbtcNativeTimeStamp = nts;
		GetSystemTime(&(tmp->st));

		bufferSkelSave.push(tmp);

		return true;
	}

	static bool myPushForceSave(float forces[], tbb::tick_count nts)
	{
		ForceStruct * tmp = new ForceStruct();

		for (int j = 0; j < 6; ++j)
		{
			tmp->fValues[j] = forces[j];
		}

		tmp->tbbtcNativeTimeStamp = nts;

		bufferForceSave.push(tmp);


		return true;
	}

	static bool myPushDepthDisplay(Mat *depthImDisplay)
	{
		Mat * tmp = new Mat(depthImDisplay->rows, depthImDisplay->cols, CV_8UC4);
		int dataSize = depthImDisplay->rows*depthImDisplay->cols*((int)depthImDisplay->elemSize());

		memcpy(tmp->data, depthImDisplay->data, (size_t)dataSize);

		bufferDepthDisplay.push(tmp);
		return true;
	}

	static bool myPushRGBDisplay(Mat *rgbImDisplay)
	{
		Mat * tmp = new Mat(rgbImDisplay->rows, rgbImDisplay->cols, CV_8UC4);
		int dataSize = rgbImDisplay->rows*rgbImDisplay->cols*((int)rgbImDisplay->elemSize());

		memcpy(tmp->data, rgbImDisplay->data, (size_t)dataSize);

		bufferRGBDisplay.push(tmp);
		return true;
	}



	static void CloseSavePaths()
	{
		if (m_bSaveRGB)
		{
			fclose(m_fpRGBMeta);

		}
		if (m_bSaveDepth)
		{
			fclose(m_fpDepthMeta);
		}
		if (m_bSaveBody)
		{
			fclose(m_fpSkel);
		}
	}

	static bool InitSavePaths()
	{
		//strcpy(cpCurrFileName, "dummy");
		char cpTmpPath[1000];

		//If saving anything create a directory to store it in
		if (m_bSaveRGB || m_bSaveDepth || m_bSaveBody)
		{
			//CreateFolder(); // we don't need to create folder
			//sprintf(cpFullDirectPath, "%s\\%s", cpRootDirect, cp);

		}
		//Colour
		if (m_bSaveRGB)
		{
			//CreateRGBFolder();
			//sprintf(cpTmpPath, "%s\\%s_rgbMeta.txt", cpFullDirectPath, cpCurrFileName);
			sprintf(cpTmpPath, "%s\\%s_rgb.avi", cpFullDirectPath, cpCurrFileName);
			cout << "rgb file name "<<cpTmpPath << endl;
			//m_fpRGBMeta = fopen(cpTmpPath, "w");


			//vwriter = cv::VideoWriter::VideoWriter(cpTmpPath, cv::VideoWriter::fourcc('M','J','P','G'), 20.0, cvSize(cColorWidth, cColorHeight), true);
			//vwriter = cv::VideoWriter::VideoWriter(cpTmpPath, cv::VideoWriter::fourcc('D', 'I', 'V', '3'), 20, cvSize(cColorWidth, cColorHeight), true);
			//vwriter = cv::VideoWriter::VideoWriter(cpTmpPath, -1, 10.0, cvSize(cColorWidth, cColorHeight), true);
			vwriter = cv::VideoWriter::VideoWriter(cpTmpPath, cv::VideoWriter::fourcc('M','P','4','2'), 20.0, cvSize(cColorWidth, cColorHeight), true);
			
		}
		if (m_bSaveDepth)
		{
			char tmpDepthFolderName[1000];
			// creating depth folder for this file
			sprintf(tmpDepthFolderName, "%s\\DEPTH_%s", cpFullDirectPath, cpCurrFileName);
			wchar_t wtext[1000];
			mbstowcs(wtext, tmpDepthFolderName, strlen(tmpDepthFolderName) + 1);//Plus null
			LPWSTR ptr = wtext;
			if (CreateDirectory(ptr, NULL) ||
				ERROR_ALREADY_EXISTS == GetLastError())
			{
				// CopyFile(...)
			}
			else
			{
				// Failed to create directory.
			}

			sprintf(cp_depthFilePath, "%s", tmpDepthFolderName);
			cout << "cp_depthFilePath" << cp_depthFilePath << endl;
			depthwriter = cv::VideoWriter::VideoWriter(cpTmpPath, CV_FOURCC('P', 'I', 'M', '1'), 20, cvSize(cDepthWidth, cDepthHeight), 0);
		}
		if (m_bSaveBody)
		{
			sprintf(cpTmpPath, "%s\\%s_bodyData.txt", cpFullDirectPath, cpCurrFileName);
			m_fpSkel = fopen(cpTmpPath, "w");

		}

		//Reset counters
		m_iDepthWriteCnt = 0;
		m_iColourWriteCnt = 0;
		m_iBodyWriteCnt = 0;

		return true;
	}


	static cv::Point changeCoordinates(Joint joint[JointType::JointType_Count], int type)
	{
		ColorSpacePoint colorSpacePoint = { 0 };
		pCoordinateMapper->MapCameraPointToColorSpace(joint[type].Position, &colorSpacePoint);
		int x = static_cast<int>(colorSpacePoint.X);
		int y = static_cast<int>(colorSpacePoint.Y);
		return cv::Point(x, y);
	}


	static cv::Point changeDepthCoordinates(Joint joint[JointType::JointType_Count], int type)
	{
		DepthSpacePoint depthSpacePoint = { 0 };
		//pCoordinateMapper->MapCameraPointsToDepthSpace(cDepthHeight*cDepthWidth, [type].Position, &depthSpacePoint);
		pCoordinateMapper->MapCameraPointToDepthSpace(joint[type].Position, &depthSpacePoint);
		int x = static_cast<int>(depthSpacePoint.X);
		int y = static_cast<int>(depthSpacePoint.Y);

		return cv::Point(x, y);
	}

	static void drawSkeleton(cv::Mat canvas, Joint joint[JointType::JointType_Count])
	{
		const cv::Scalar GREEN = cv::Scalar(0, 255, 0);
		cv::line(canvas, changeCoordinates(joint, JointType_Head), changeCoordinates(joint, JointType_Neck), GREEN, 3);
		cv::line(canvas, changeCoordinates(joint, JointType_Neck), changeCoordinates(joint, JointType_SpineShoulder), GREEN, 3);
		cv::line(canvas, changeCoordinates(joint, JointType_SpineShoulder), changeCoordinates(joint, JointType_ShoulderLeft), GREEN, 3);
		cv::line(canvas, changeCoordinates(joint, JointType_SpineShoulder), changeCoordinates(joint, JointType_ShoulderRight), GREEN, 3);
		cv::line(canvas, changeCoordinates(joint, JointType_SpineShoulder), changeCoordinates(joint, JointType_SpineMid), GREEN, 3);
		cv::line(canvas, changeCoordinates(joint, JointType_ShoulderLeft), changeCoordinates(joint, JointType_ElbowLeft), GREEN, 3);
		cv::line(canvas, changeCoordinates(joint, JointType_ShoulderRight), changeCoordinates(joint, JointType_ElbowRight), GREEN, 3);
		cv::line(canvas, changeCoordinates(joint, JointType_ElbowLeft), changeCoordinates(joint, JointType_WristLeft), GREEN, 3);
		cv::line(canvas, changeCoordinates(joint, JointType_ElbowRight), changeCoordinates(joint, JointType_WristRight), GREEN, 3);
		cv::line(canvas, changeCoordinates(joint, JointType_WristLeft), changeCoordinates(joint, JointType_HandLeft), GREEN, 3);
		cv::line(canvas, changeCoordinates(joint, JointType_WristRight), changeCoordinates(joint, JointType_HandRight), GREEN, 3);
		cv::line(canvas, changeCoordinates(joint, JointType_HandLeft), changeCoordinates(joint, JointType_HandTipLeft), GREEN, 3);
		cv::line(canvas, changeCoordinates(joint, JointType_HandRight), changeCoordinates(joint, JointType_HandTipRight), GREEN, 3);
		cv::line(canvas, changeCoordinates(joint, JointType_WristLeft), changeCoordinates(joint, JointType_ThumbLeft), GREEN, 3);
		cv::line(canvas, changeCoordinates(joint, JointType_WristRight), changeCoordinates(joint, JointType_ThumbRight), GREEN, 3);
		cv::line(canvas, changeCoordinates(joint, JointType_SpineMid), changeCoordinates(joint, JointType_SpineBase), GREEN, 3);
		cv::line(canvas, changeCoordinates(joint, JointType_SpineBase), changeCoordinates(joint, JointType_HipLeft), GREEN, 3);
		cv::line(canvas, changeCoordinates(joint, JointType_SpineBase), changeCoordinates(joint, JointType_HipRight), GREEN, 3);
		cv::line(canvas, changeCoordinates(joint, JointType_HipLeft), changeCoordinates(joint, JointType_KneeLeft), GREEN, 3);
		cv::line(canvas, changeCoordinates(joint, JointType_HipRight), changeCoordinates(joint, JointType_KneeRight), GREEN, 3);
		cv::line(canvas, changeCoordinates(joint, JointType_KneeLeft), changeCoordinates(joint, JointType_AnkleLeft), GREEN, 3);
		cv::line(canvas, changeCoordinates(joint, JointType_KneeRight), changeCoordinates(joint, JointType_AnkleRight), GREEN, 3);
		cv::line(canvas, changeCoordinates(joint, JointType_AnkleLeft), changeCoordinates(joint, JointType_FootLeft), GREEN, 3);
		cv::line(canvas, changeCoordinates(joint, JointType_AnkleRight), changeCoordinates(joint, JointType_FootRight), GREEN, 3);
	}



	static void skeletonTracking()
	{
		cout << "inside skeletontracking" << cpRootDirect << endl;
		cv::setUseOptimized(true);

		// sensor
		HRESULT hResult = S_OK;
		hResult = GetDefaultKinectSensor(&pSensor);
		if (FAILED(hResult))
		{
			std::cerr << "Error: GetDefaultKinectSensor" << std::endl;
			exit(-1);
		}

		hResult = pSensor->Open();
		if (FAILED(hResult))
		{
			std::cerr << "Error: IKinectSensor::Open()" << std::endl;
			exit(-1);
		}

		// Source
		hResult = pSensor->get_ColorFrameSource(&pColorSource);
		if (FAILED(hResult))
		{
			std::cerr << "Error: IKinectSensor::get_ColorFrameSource()" << std::endl;
			exit(-1);
		}

		hResult = pSensor->get_BodyFrameSource(&pBodySource);
		if (FAILED(hResult))
		{
			std::cerr << "Error: IKinectSensor::get_BodyFrameSource()" << std::endl;
			exit(-1);
		}

		hResult = pSensor->get_DepthFrameSource(&pDepthSource);
		if (FAILED(hResult))
		{
			std::cerr << "Error: IKinectSensor::get_DepthFrameSource()" << std::endl;
			exit(-1);
		}

		// Reader
		hResult = pColorSource->OpenReader(&pColorReader);
		if (FAILED(hResult))
		{
			std::cerr << "Error: IColorFrameSource::OpenReader()" << std::endl;
			exit(-1);
		}

		hResult = pBodySource->OpenReader(&pBodyReader);
		if (FAILED(hResult))
		{
			std::cerr << "Error: IBodyFrameSource::OpenReader()" << std::endl;
			exit(-1);
		}

		hResult = pDepthSource->OpenReader(&pDepthReader);
		if (FAILED(hResult))
		{
			std::cerr << "Error: IDepthFrameSource::OpenReader()" << std::endl;
			exit(-1);
		}

		// Description
		hResult = pColorSource->get_FrameDescription(&pColorDescription);
		if (FAILED(hResult))
		{
			std::cerr << "Error: IColorFrameSource::get_FrameDescription()" << std::endl;
			exit(-1);
		}

		hResult = pDepthSource->get_FrameDescription(&pDepthDescription);
		if (FAILED(hResult))
		{
			std::cerr << "Error: IDepthFrameSource::get_FrameDescription()" << std::endl;
			exit(-1);
		}

		colorWidth = 0;
		colorHeight = 0;
		depthWidth = 0;
		depthHeight = 0;
		pColorDescription->get_Width(&colorWidth);   // 1920
		pColorDescription->get_Height(&colorHeight); // 1080
		pDepthDescription->get_Width(&depthWidth);   // 512
		pDepthDescription->get_Height(&depthHeight); // 424
		unsigned int colorbufferSize = colorWidth * colorHeight * 4 * sizeof(unsigned char);
		//unsigned int depthbufferSize = depthWidth * depthHeight * 4 * sizeof(unsigned char);
		unsigned int depthbufferSize = depthWidth * depthHeight * sizeof(unsigned char);

		cv::Mat colorBufferMat(colorHeight, colorWidth, CV_8UC4);
		cv::Mat depthBufferMat(depthHeight, depthWidth, CV_16UC1);
		cv::Mat bodyMat(colorHeight / 2, colorWidth / 2, CV_8UC4);
		cv::Mat bodyMatRGB(colorHeight / 2, colorWidth / 2, CV_8UC4);

		cv::Mat depthMat(depthHeight, depthWidth, CV_8UC1);

		std::string colorWinName = "Skeleton RGB";
		std::string depthWinName = "Skeleton Depth";
		cv::namedWindow(colorWinName);
		cv::namedWindow(depthWinName);

		// Color Table
		cv::Vec3b color[BODY_COUNT];
		color[0] = cv::Vec3b(255, 0, 0);
		color[1] = cv::Vec3b(0, 255, 0);
		color[2] = cv::Vec3b(0, 0, 255);
		color[3] = cv::Vec3b(255, 255, 0);
		color[4] = cv::Vec3b(255, 0, 255);
		color[5] = cv::Vec3b(0, 255, 255);

		// Range (Range of Depth is 500-8000[mm], Range of Detection is 500-45000[mm])
		unsigned short min = 0;
		unsigned short max = 0;
		pDepthSource->get_DepthMinReliableDistance(&min);
		pDepthSource->get_DepthMaxReliableDistance(&max);
		std::cout << "Range: " << min << " - " << max << std::endl;

		// Coordinate Mapper
		hResult = pSensor->get_CoordinateMapper(&pCoordinateMapper);
		if (FAILED(hResult))
		{
			std::cerr << "Error: IKinectSensor::get_CoordinateMapper()" << std::endl;
			exit(-1);
		}
		// mainLoop where frame by frame data is gotten
		InitSavePaths();
		SkeletonFile* skFileDat = new SkeletonFile();
		skFileDat->frameCount = 0;
		int whileCount = 0;
		int skFramecount = 0;

		while (1)
		{
			// Frame
			IColorFrame* pColorFrame = 0;
			IDepthFrame* pDepthFrame = 0;
			hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame);
			if (SUCCEEDED(hResult))
			{
				hResult = pDepthFrame->AccessUnderlyingBuffer(&depthbufferSize, reinterpret_cast<UINT16**>(&depthBufferMat.data));
			
				if (SUCCEEDED(hResult))
					//depthBufferMat.convertTo(depthMat, CV_8U, -255.0f / 8000.0f, 255.0f);
					depthBufferMat.convertTo(depthMat, CV_8U, -255.0f / 8000.0f, 100.0f);

				//pDepthFrame->CopyFrameDataToArray
			}

			hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
			if (SUCCEEDED(hResult))
			{
				hResult = pColorFrame->CopyConvertedFrameDataToArray(colorbufferSize, reinterpret_cast<BYTE*>(colorBufferMat.data), ColorImageFormat::ColorImageFormat_Bgra);
				if (SUCCEEDED(hResult))
					//cv::resize(colorBufferMat, bodyMat, cv::Size(), 0.5, 0.5);
					//cv::resize(colorBufferMat, bodyMatRGB, cv::Size(), 0.5, 0.5);
					//memcpy(bodyMatRGB, &colorBufferMat.data, colorbufferSize);
					bodyMatRGB = colorBufferMat.clone();	// for writing video
			}
			
			IBodyFrame* pBodyFrame = 0;
			hResult = pBodyReader->AcquireLatestFrame(&pBodyFrame);
			if (SUCCEEDED(hResult))
			{
				IBody* pBody[BODY_COUNT] = { 0 };
				hResult = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBody);
				if (SUCCEEDED(hResult))
				{
					bool least_one_body = false;
					for (int count = 0; count < BODY_COUNT; count++)
					{
						BOOLEAN bTracked = false;
						hResult = pBody[count]->get_IsTracked(&bTracked);
						if (SUCCEEDED(hResult) && bTracked)
						{
							least_one_body = true;
							Joint joint[JointType::JointType_Count];
							JointXYZ* jointDat = new JointXYZ();
							hResult = pBody[count]->GetJoints(JointType::JointType_Count, joint);
							//if (SUCCEEDED(hResult) && TrackingConfidence_High)
							if (SUCCEEDED(hResult) && TrackingConfidence_High)
							{
								// Joints
								for (int type = 0; type < JointType::JointType_Count; type++)
								{
									cv::Point jointPoint = changeCoordinates(joint, type);
									cv::Point jointPointDepth = changeDepthCoordinates(joint, type);
									if ((jointPoint.x >= 0) && (jointPoint.y >= 0) && (jointPoint.x < colorWidth) && (jointPoint.y < colorHeight))
									{
										cv::circle(colorBufferMat, jointPoint, 8, static_cast<cv::Scalar>(color[count]), -1, CV_AA);
									}

									// writing skeleeton data
									if (type)
										fprintf(m_fpSkel, ", ");
									fprintf(m_fpSkel, "%f %f %f %d %d %d %d", joint[type].Position.X, joint[type].Position.Y, joint[type].Position.Z, jointPoint.x, jointPoint.y, jointPointDepth.x, jointPointDepth.y);
								}
								fprintf(m_fpSkel, "\n");
								drawSkeleton(colorBufferMat, joint);

							}
							else
							{
								std::cout << "Tracking confidene low!!"<< std::endl;
							}

							// Lean
							PointF amount;
							hResult = pBody[count]->get_Lean(&amount);
							if (SUCCEEDED(hResult))
								std::cout << "amount: " << amount.X << ", " << amount.Y << std::endl;
						}
							
					}

					whileCount++;
					if (least_one_body) {
						skFramecount++;
						skFileDat->frameCount++;			// cause here at least one body

						vwriter.write(bodyMatRGB);
						//vwriter.write(colorBufferMat);
						sprintf(m_cpDepthfilepathTmp, "%s\\depth%s%d.png",
							cp_depthFilePath,
							(skFramecount<10) ? "0000" : (skFramecount<100) ? "000" : (skFramecount<1000) ? "00" : (skFramecount<10000) ? "0" : "",
							skFramecount);

						printf("%s \n", m_cpDepthfilepathTmp);
						imwrite(m_cpDepthfilepathTmp, depthMat);

					}
					else {

						SafeRelease(pColorFrame);
						SafeRelease(pBodyFrame);
						SafeRelease(pDepthFrame);
						continue;

					}
				}
				for (int count = 0; count<BODY_COUNT; count++)
				{
					SafeRelease(pBody[count]);
				}
				
			}
			else
			{
				std::cout << "Body frame is not acquiring!" << std::endl;
			}
			SafeRelease(pColorFrame);
			SafeRelease(pBodyFrame);
			SafeRelease(pDepthFrame);

			// save color image, depth image, video is running here
			cv::resize(colorBufferMat, bodyMat, cv::Size(), 0.5, 0.5);
			cv::imshow(colorWinName, bodyMat);
			cv::imshow(depthWinName, depthMat);

			if (stopFlag == true)
				break;
			waitKey(10);		// put current frame for this milliseconds
			
		}
		
		printf("frame count %d, sk frame %d \n", whileCount, skFramecount);
		fclose(m_fpSkel);
		SafeRelease(pColorSource);
		SafeRelease(pBodySource);
		SafeRelease(pColorReader);
		SafeRelease(pBodyReader);
		SafeRelease(pColorDescription);
		SafeRelease(pCoordinateMapper);
		SafeRelease(pDepthSource);
		SafeRelease(pDepthReader);
		SafeRelease(pDepthDescription);
		if (pSensor)
			pSensor->Close();
		SafeRelease(pSensor);
		cv::destroyAllWindows();
		vwriter.release();
		depthwriter.release();
	}


	static void  InitStuff()
	{
		//Do timestamp start
		m_tcStart = tbb::tick_count::now();
		InitSavePaths();
		// create heap storage for color pixel data in RGBX format
		if (m_bViewRGB)
		{
			imgColourDisplay = new Mat(cColorHeight / m_iDisplayScaleFactor, cColorWidth / m_iDisplayScaleFactor, CV_8UC4);
			namedWindow("Colour window", WINDOW_AUTOSIZE);
			moveWindow("Colour window", 600, 10);
		}
		if (m_bViewRGB || m_bSaveRGB)
		{
			m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
			imgColour = new Mat(cColorHeight, cColorWidth, CV_8UC4);
		}
		// create heap storage for color pixel data in RGBX format
		if (m_bViewDepth || m_bViewBody)
		{
			// create heap storage for depth pixel data in RGBX format
			m_pDepthRGBX = new RGBQUAD[cDepthWidth * cDepthHeight];
			imgDepthDisplay = new Mat(cDepthHeight, cDepthWidth, CV_8UC4);
			namedWindow("Depth window", WINDOW_NORMAL);
			moveWindow("Depth window", 10, 10);
		}
		if (m_bViewDepth || m_bSaveDepth) //Need to show body in a window - use depth 
		{
			imgDepth = new Mat(cDepthHeight, cDepthWidth, CV_16U);
		}
		if (!(m_bViewBody || m_bViewDepth || m_bViewRGB))
		{
			namedWindow("Info Window", WINDOW_AUTOSIZE);

			Mat Img = imread("im.png", CV_LOAD_IMAGE_COLOR);
			resize(Img, Img, cv::Size(290, 80));
			imshow("Info Window", Img);
			moveWindow("Info Window", m_iLeft, m_iTop + m_iHeight);
			waitKey(10);

		}
	}


	// Safe release for interfaces
	template<class Interface>
	static inline void SafeRelease(Interface *& pInterfaceToRelease)
	{
		if (pInterfaceToRelease != NULL)
		{
			pInterfaceToRelease->Release();
			pInterfaceToRelease = NULL;
		}
	}

	static void UpdateDepthWrap()
	{
		if (m_bSaveDepth || m_bViewDepth)
		{
			while (m_bRunMainLoop)
			{
				UpdateDepth();
			}
		}
	}

	static void UpdateColourWrap()
	{
		if (m_bSaveRGB || m_bViewRGB)
		{
			while (m_bRunMainLoop)
			{
				UpdateColour();
			}
		}
	}

	static void UpdateBodyWrap()
	{
		if (m_bSaveBody || m_bViewBody)
		{
			while (m_bRunMainLoop)
			{
				UpdateBody();
			}
		}
	}

	static void writeRGBwrap()
	{
		while (1)
		{
			writeRGB();
		}
	}
	static void writeDepthwrap()
	{
		while (1)
		{
			writeDepth();
		}
	}
	static void writeSkeletonwrap()
	{
		while (1)
		{
			writeSkeleton();
		}
	}
	static bool SaveThread()
	{

		if (m_bSaveRGB)
		{
			tbb::tbb_thread pMyThread1 = tbb::tbb_thread(writeRGBwrap);
		}
		if (m_bSaveDepth)
		{
			tbb::tbb_thread pMyThread2 = tbb::tbb_thread(writeDepthwrap);
		}
		if (m_bSaveBody)
		{
			tbb::tbb_thread pMyThread4 = tbb::tbb_thread(writeSkeletonwrap);
		}


		return true;
	}

	static void  mainLoop(int iNumIts, int numSaves)
	{
		/*
		strcpy(m_cpDebugStr, "Entering main loop");
		tbb::tbb_thread pMyThread1 = tbb::tbb_thread(UpdateDepthWrap);
		tbb::tbb_thread pMyThread2 = tbb::tbb_thread(UpdateColourWrap);
		tbb::tbb_thread pMyThread3 = tbb::tbb_thread(UpdateBodyWrap);
		Sleep(100);
		SaveThread();

		Mat *tmp;
		*/
		int break_cnt = 0;
		while (m_bRunMainLoop)
		{
			printf("inside MainLoop! %d", ++break_cnt);

			if (break_cnt >= 20000)
				break;
			/*
			if (m_bViewDepth && !m_bViewBody && bufferDepthDisplay.try_pop(tmp))
			{
				imshow("Depth window", *tmp);
				waitKey(10);
				tmp->release();
			}
			if (m_bViewRGB && bufferRGBDisplay.try_pop(tmp))
			{

				imshow("Colour window", *tmp);
				waitKey(10);
				tmp->release();
			}
			if (m_bViewBody && bufferDepthDisplay.try_pop(tmp))
			{
				imshow("Depth window", *tmp);
				waitKey(10);
				tmp->release();
			}
			if (!(m_bViewBody || m_bViewDepth || m_bViewRGB))
			{
				waitKey(100);
			}
			*/
		}

		//finishing writing files
		while (1)
		{
			Sleep(300);
			break;
			/*
			if (bufferDepthSave.unsafe_size() > 0 || bufferRGBSave.unsafe_size() > 0 || bufferForceSave.unsafe_size() > 0 || bufferSkelSave.unsafe_size() > 0)
			{
				continue;
			}
			else
			{
				break;
			}
			*/
		}


	}

	static HRESULT InitializeDefaultSensor()
	{
		HRESULT hr;

		hr = GetDefaultKinectSensor(&m_pKinectSensor);
		if (FAILED(hr))
		{
			printf("Failed to init the sensor");
			return hr;
		}
		else
		{
			system("pause");
			printf("Sensor successfully initialized!");
		}

		if (m_pKinectSensor)
		{
			// Initialize the Kinect and get the depth reader
			IDepthFrameSource* pDepthFrameSource = NULL;

			// Initialize the Kinect and get the color reader
			IColorFrameSource* pColorFrameSource = NULL;

			// Initialize the Kinect and get coordinate mapper and the body reader
			IBodyFrameSource* pBodyFrameSource = NULL;

			hr = m_pKinectSensor->Open();

			if (m_bViewBody || m_bSaveBody)
			{
				if (SUCCEEDED(hr))
				{
					hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
				}

				if (SUCCEEDED(hr))
				{
					hr = m_pKinectSensor->get_BodyFrameSource(&m_pBodyFrameSource);
				}

				if (SUCCEEDED(hr))
				{
					hr = m_pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
				}

				SafeRelease(m_pBodyFrameSource);
			}

			if (m_bViewDepth || m_bSaveDepth)
			{
				if (SUCCEEDED(hr))
				{
					hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
				}

				if (SUCCEEDED(hr))
				{
					hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
				}
				SafeRelease(pDepthFrameSource);
			}

			if (m_bViewRGB || m_bSaveRGB)
			{
				if (SUCCEEDED(hr))
				{
					hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
				}

				if (SUCCEEDED(hr))
				{
					hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
				}
				SafeRelease(pColorFrameSource);
			}

		}

		if (!m_pKinectSensor || FAILED(hr))
		{
			printf("No ready Kinect found!");
			getchar();
			return E_FAIL;
		}


		return hr;
	}

	static void BodyToDepthScreen(int *x, int *y, const CameraSpacePoint& bodyPoint, int width, int height)
	{
		// Calculate the body's position on the screen
		DepthSpacePoint depthPoint = { 0 };
		m_pCoordinateMapper->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);
		*x = static_cast<int>(depthPoint.X * width) / cDepthWidth;
		*y = static_cast<int>(depthPoint.Y * height) / cDepthHeight;

	}

	static void BodyToColourScreen(int *x, int *y, const CameraSpacePoint& bodyPoint, int width, int height)
	{
		// Calculate the body's position on the screen
		DepthSpacePoint depthPoint = { 0 };
		m_pCoordinateMapper->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);
		*x = static_cast<int>(depthPoint.X * width) / cColorWidth;
		*y = static_cast<int>(depthPoint.Y * height) / cColorHeight;

	}



	static void textMessage()
	{
		cv::putText(*imgDepthDisplay, m_cpDebugStr, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
	}

	static void ProcessBodyOriginal(INT64 nTime, int nBodyCount, IBody** ppBodies)
	{

		HRESULT hr;
		if (m_pCoordinateMapper)
		{

			int width = cDepthWidth;
			int height = cDepthHeight;

			for (int i = 0; i < nBodyCount; ++i)
			{
				IBody* pBody = ppBodies[i];
				if (pBody)
				{
					BOOLEAN bTracked = false;
					hr = pBody->get_IsTracked(&bTracked);

					if (SUCCEEDED(hr) && bTracked)
					{
						Joint joints[JointType_Count];
						HandState leftHandState = HandState_Unknown;
						HandState rightHandState = HandState_Unknown;

						pBody->get_HandLeftState(&leftHandState);
						pBody->get_HandRightState(&rightHandState);

						hr = pBody->GetJoints(_countof(joints), joints);
						if (SUCCEEDED(hr))
						{
							if (m_bSaveBody)
							{
								if (m_bScreenSkeleton)
								{
									for (int j = 0; j < _countof(joints); ++j)
									{
										BodyToDepthScreen(&(m_2dpScreenCoords[j].x), &(m_2dpScreenCoords[j].y), joints[j].Position, width, height);
									}
									myPushSkelSaveScreen(joints, nTime, tbb::tick_count::now(), m_2dpScreenCoords);

								}
								else
								{
									myPushSkelSave(joints, nTime, tbb::tick_count::now());
								}
							}
							if (m_bViewBody)
							{
								for (int j = 0; j < _countof(joints); ++j)
								{
									BodyToDepthScreen(&(m_2dpScreenCoords[j].x), &(m_2dpScreenCoords[j].y), joints[j].Position, width, height);
								}
								DrawBody(joints, m_2dpScreenCoords);
							}
						}
					}
				}
			}


		}


	}

	static void UpdateBody()
	{

		if (!m_pBodyFrameReader)
		{
			return;
		}

		IBodyFrame* pBodyFrame = NULL;

		HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);

		if (SUCCEEDED(hr))
		{
			INT64 nTime = 0;

			hr = pBodyFrame->get_RelativeTime(&nTime);

			IBody* ppBodies[BODY_COUNT] = { 0 };

			if (SUCCEEDED(hr))
			{
				hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
			}

			if (SUCCEEDED(hr))
			{
				if (!m_bViewDepth && m_bViewBody)
					*imgDepthDisplay = Mat::zeros(cDepthHeight, cDepthWidth, CV_8UC4);
				ProcessBodyOriginal(nTime, BODY_COUNT, ppBodies);

				if (m_bViewBody)
				{
					myPushDepthDisplay(imgDepthDisplay);
				}
			}

			for (int i = 0; i < _countof(ppBodies); ++i)
			{
				SafeRelease(ppBodies[i]);
			}
		}

		SafeRelease(pBodyFrame);

	}

	static void UpdateDepth()
	{
		if (!m_pDepthFrameReader)
		{
			strcpy(m_cpDebugStr, "No Depth reader");
			return;
		}

		IDepthFrame* pDepthFrame = NULL;

		HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

		if (SUCCEEDED(hr))
		{

			INT64 nTime = 0;
			IFrameDescription* pFrameDescription = NULL;
			int nWidth = 0;
			int nHeight = 0;
			USHORT nDepthMinReliableDistance = 0;
			USHORT nDepthMaxReliableDistance = 0;
			UINT nBufferSize = 0;
			UINT16 *pBuffer = NULL;

			hr = pDepthFrame->get_RelativeTime(&nTime);

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
			}

			if (SUCCEEDED(hr))
			{
				hr = pFrameDescription->get_Width(&nWidth);
			}

			if (SUCCEEDED(hr))
			{
				hr = pFrameDescription->get_Height(&nHeight);
			}

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
			}

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxReliableDistance);
			}

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
			}

			if (SUCCEEDED(hr))
			{

				ProcessDepth(nTime, pBuffer, nWidth, nHeight, nDepthMinReliableDistance, nDepthMaxReliableDistance);
			}

			SafeRelease(pFrameDescription);
		}
		else
		{
		}

		SafeRelease(pDepthFrame);
	}

	static void UpdateColour()
	{
		if (!m_pColorFrameReader)
		{
			cout << "No Depth reader\n";
			return;
		}

		IColorFrame* pColorFrame = NULL;

		HRESULT hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);

		if (SUCCEEDED(hr))
		{
			INT64 nTime = 0;
			IFrameDescription* pFrameDescription = NULL;
			int nWidth = 0;
			int nHeight = 0;
			ColorImageFormat imageFormat = ColorImageFormat_None;
			UINT nBufferSize = 0;
			RGBQUAD *pBuffer = NULL;

			hr = pColorFrame->get_RelativeTime(&nTime);

			if (SUCCEEDED(hr))
			{
				hr = pColorFrame->get_FrameDescription(&pFrameDescription);
			}

			if (SUCCEEDED(hr))
			{
				hr = pFrameDescription->get_Width(&nWidth);
			}

			if (SUCCEEDED(hr))
			{
				hr = pFrameDescription->get_Height(&nHeight);
			}

			if (SUCCEEDED(hr))
			{
				hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
			}

			if (SUCCEEDED(hr))
			{
				if (imageFormat == ColorImageFormat_Bgra)
				{
					hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&pBuffer));
				}
				else if (m_pColorRGBX)
				{
					pBuffer = m_pColorRGBX;
					nBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
					hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);
				}

				else
				{
					hr = E_FAIL;
				}
			}

			if (SUCCEEDED(hr))
			{
				int dataSize = imgColour->rows*imgColour->cols*((int)imgColour->elemSize());

				imgColour->data = (uchar*)pBuffer;

				if (m_bSaveRGB)
				{
					ProcessColor(nTime, pBuffer, nWidth, nHeight);
				}
				if (m_bViewRGB)
				{
					resize(*imgColour, *imgColourDisplay, cv::Size(imgColourDisplay->cols, imgColourDisplay->rows));
					myPushRGBDisplay(imgColourDisplay);
				}


			}

			SafeRelease(pFrameDescription);
		}

		SafeRelease(pColorFrame);
	}

	static void safeClose()
	{

		//Close files
		CloseSavePaths();

		// done with color frame reader
		SafeRelease(m_pColorFrameReader);

		// done with depth frame reader
		SafeRelease(m_pDepthFrameReader);

		// done with body frame reader
		SafeRelease(m_pBodyFrameReader);


		// done with coordinate mapper
		SafeRelease(m_pCoordinateMapper);

		// close the Kinect Sensor
		if (m_pKinectSensor)
		{
			m_pKinectSensor->Close();
		}

		SafeRelease(m_pKinectSensor);

		CloseSavePaths();
	}

	static void ProcessColor(INT64 nTime, RGBQUAD* pBuffer, int nWidth, int nHeight)
	{

		// Make sure we've received valid data
		if (pBuffer && (nWidth == cColorWidth) && (nHeight == cColorHeight))
		{
			myPushRGBSave(imgColour, nTime, tbb::tick_count::now());
		}
	}


	static void ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
	{
		// Make sure we've received valid data
		if (pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight))
		{

			//write the image
			if (m_bSaveDepth)
			{
				//GetFilenamePNG();		
				imgDepth->data = (uchar*)pBuffer;
				//imwrite(cpFilenamePNG, *imgDepth);
				myPushDepthSave(imgDepth, nTime, tbb::tick_count::now());
			}

			if (m_pDepthRGBX && m_bViewDepth)
			{
				RGBQUAD* pRGBX = m_pDepthRGBX;

				// end pixel is start + width*height - 1
				const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

				while (pBuffer < pBufferEnd)
				{
					USHORT depth = *pBuffer;

					// To convert to a byte, we're discarding the most-significant
					// rather than least-significant bits.
					// We're preserving detail, although the intensity will "wrap."
					// Values outside the reliable depth range are mapped to 0 (black).

					// Note: Using conditionals in this loop could degrade performance.
					// Consider using a lookup table instead when writing production code.
					BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth % 256) : 0);

					pRGBX->rgbRed = intensity;
					pRGBX->rgbGreen = intensity;
					pRGBX->rgbBlue = intensity;

					++pRGBX;
					++pBuffer;
				}
				imgDepthDisplay->data = (uchar*)m_pDepthRGBX;
				if (!m_bViewBody)
					myPushDepthDisplay(imgDepthDisplay);

			}
		}
	}

	static void CreateRGBFolder()
	{
		sprintf(cpFullDirectPathRGB, "%s\\RGB", cpFullDirectPath);
		wchar_t wtext[1000];
		mbstowcs(wtext, cpFullDirectPathRGB, strlen(cpFullDirectPathRGB) + 1);//Plus null
		LPWSTR ptr = wtext;

		if (CreateDirectory(ptr, NULL) ||
			ERROR_ALREADY_EXISTS == GetLastError())
		{
			// CopyFile(...)
		}
		else
		{
			// Failed to create directory.
		}
	}

	static void CreateDepthFolder()
	{

		sprintf(cpFullDirectPathDepth, "%s\\DEPTH", cpFullDirectPath);
		wchar_t wtext[1000];
		mbstowcs(wtext, cpFullDirectPathDepth, strlen(cpFullDirectPathDepth) + 1);//Plus null
		LPWSTR ptr = wtext;

		if (CreateDirectory(ptr, NULL) ||
			ERROR_ALREADY_EXISTS == GetLastError())
		{
			// CopyFile(...)
		}
		else
		{
			// Failed to create directory.
		}
	}

	static void CreateFolder()
	{
		GetFolderName();

		sprintf(cpFullDirectPath, "%s\\%s", cpRootDirect, cpSubdirectoryName);
		wchar_t wtext[1000];
		mbstowcs(wtext, cpFullDirectPath, strlen(cpFullDirectPath) + 1);//Plus null
		LPWSTR ptr = wtext;
		cout << ptr << endl;
		cout << wtext << endl;
		cout << CreateDirectory(ptr, NULL) << endl;
		cout << "current file name " << cpCurrFileName << endl;

		if (CreateDirectory(ptr, NULL) ||
			ERROR_ALREADY_EXISTS == GetLastError())
		{
			// CopyFile(...)
		}
		else
		{

			cout << "failed to create directory named: " <<cpFullDirectPath<< endl;
			
		}
	}


	static void GetFolderName()
	{
		SYSTEMTIME st;
		GetSystemTime(&st);

		sprintf(cpSubdirectoryName, "%d-%s%d-%s%d-%s%d_%s%d-%s%d", st.wYear,
			(st.wMonth < 10) ? "0" : "", st.wMonth,
			(st.wDay < 10) ? "0" : "", st.wDay,
			(st.wHour < 10) ? "0" : "", st.wHour,
			(st.wMinute < 10) ? "0" : "", st.wMinute,
			(st.wSecond < 10) ? "0" : "", st.wSecond);
	}

	static void GetFilenamePNG()
	{
		SYSTEMTIME st;
		GetSystemTime(&st);

		sprintf(cpFilenamePNG, "depth_%s%d-%s%d_%s%d-%s%d-%s%d.png",

			(st.wDay < 10) ? "0" : "", st.wDay,
			(st.wHour < 10) ? "0" : "", st.wHour,
			(st.wMinute < 10) ? "0" : "", st.wMinute,
			(st.wSecond < 10) ? "0" : "", st.wSecond,
			(st.wMilliseconds < 10) ? "000" : (st.wMilliseconds < 100) ? "00" : (st.wMilliseconds < 1000) ? "0" : "", st.wMilliseconds);
	}

	static void GetFilenameJPG()
	{
		SYSTEMTIME st;
		GetSystemTime(&st);

		sprintf(cpFilenameJPG, "colour_%s%d-%s%d_%s%d-%s%d-%s%d.jpg",

			(st.wDay < 10) ? "0" : "", st.wDay,
			(st.wHour < 10) ? "0" : "", st.wHour,
			(st.wMinute < 10) ? "0" : "", st.wMinute,
			(st.wSecond < 10) ? "0" : "", st.wSecond,
			(st.wMilliseconds < 10) ? "000" : (st.wMilliseconds < 100) ? "00" : (st.wMilliseconds < 1000) ? "0" : "", st.wMilliseconds);

	}

	static void DrawBone(const Joint* pJoints, cv::Point pJointPoints[], int iStart, int iEnd)
	{
		cv::line(*imgDepthDisplay, pJointPoints[iStart], pJointPoints[iEnd], cv::Scalar(255, 0, 0), 3);
	}

	static void DrawBody(const Joint* pJoints, cv::Point pJointPoints[])
	{
		//Text

		// Draw the bones
		// Torso
		DrawBone(pJoints, pJointPoints, KIN_JointType_Head, KIN_JointType_Neck);
		DrawBone(pJoints, pJointPoints, KIN_JointType_Neck, KIN_JointType_SpineShoulder);
		DrawBone(pJoints, pJointPoints, KIN_JointType_SpineShoulder, KIN_JointType_SpineMid);
		DrawBone(pJoints, pJointPoints, KIN_JointType_SpineMid, KIN_JointType_SpineBase);
		DrawBone(pJoints, pJointPoints, KIN_JointType_SpineShoulder, KIN_JointType_ShoulderRight);
		DrawBone(pJoints, pJointPoints, KIN_JointType_SpineShoulder, KIN_JointType_ShoulderLeft);
		DrawBone(pJoints, pJointPoints, KIN_JointType_SpineBase, KIN_JointType_HipRight);
		DrawBone(pJoints, pJointPoints, KIN_JointType_SpineBase, KIN_JointType_HipLeft);

		// Left Arm
		DrawBone(pJoints, pJointPoints, KIN_JointType_ShoulderLeft, KIN_JointType_ElbowLeft);
		DrawBone(pJoints, pJointPoints, KIN_JointType_ElbowLeft, KIN_JointType_WristLeft);
		DrawBone(pJoints, pJointPoints, KIN_JointType_WristLeft, KIN_JointType_HandLeft);
		DrawBone(pJoints, pJointPoints, KIN_JointType_HandLeft, KIN_JointType_HandTipLeft);
		DrawBone(pJoints, pJointPoints, KIN_JointType_WristLeft, KIN_JointType_ThumbLeft);

		// Right Arm
		DrawBone(pJoints, pJointPoints, KIN_JointType_ShoulderRight, KIN_JointType_ElbowRight);
		DrawBone(pJoints, pJointPoints, KIN_JointType_ElbowRight, KIN_JointType_WristRight);
		DrawBone(pJoints, pJointPoints, KIN_JointType_WristRight, KIN_JointType_HandRight);
		DrawBone(pJoints, pJointPoints, KIN_JointType_HandRight, KIN_JointType_HandTipRight);
		DrawBone(pJoints, pJointPoints, KIN_JointType_WristRight, KIN_JointType_ThumbRight);

		// Right Leg
		DrawBone(pJoints, pJointPoints, KIN_JointType_HipRight, KIN_JointType_KneeRight);
		DrawBone(pJoints, pJointPoints, KIN_JointType_KneeRight, KIN_JointType_AnkleRight);
		DrawBone(pJoints, pJointPoints, KIN_JointType_AnkleRight, KIN_JointType_FootRight);

		// Left Leg
		DrawBone(pJoints, pJointPoints, KIN_JointType_HipLeft, KIN_JointType_KneeLeft);
		DrawBone(pJoints, pJointPoints, KIN_JointType_KneeLeft, KIN_JointType_AnkleLeft);
		DrawBone(pJoints, pJointPoints, KIN_JointType_AnkleLeft, KIN_JointType_FootLeft);

		// Draw the joints
		for (int i = 0; i < JointType_Count; ++i)
		{
			if (pJoints[i].TrackingState == TrackingState_Inferred)
			{
				cv::circle(*imgDepthDisplay, pJointPoints[i], 3, cv::Scalar(0, 0, 255), 2);
			}
			else if (pJoints[i].TrackingState == TrackingState_Tracked)
			{
				cv::circle(*imgDepthDisplay, pJointPoints[i], 3, cv::Scalar(0, 255, 0), 2);
			}
		}
	}
};

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////                          University of Bristol                          ////////////////
//////////////                       Computer Science Department                       ////////////////
//===================================================================================================//
///////                            This is an open source code for:                             ///////
////////           "3D Data Acquisition and Registration using Two Opposing Kinects"         //////////
////////////////      V. Soleimani, M. Mirmehdi, D. Damen, S. Hannuna, M. Camplani    /////////////////
////////////////         International Conference on 3D Vision, Stanford, 2016        /////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
