/* +---------------------------------------------------------------------------+
|                     Mobile Robot Programming Toolkit (MRPT)               |
|                          http://www.mrpt.org/                             |
|                                                                           |
| Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
| See: http://www.mrpt.org/Authors - All rights reserved.                   |
| Released under BSD License. See details in http://www.mrpt.org/License    |
+---------------------------------------------------------------------------+ */

/*
Example  : kinect-online-offline-demo
Web page : http://www.mrpt.org/Switching_between_reading_live_Kinect_RGBD_dataset_for_debugging

Purpose  : Demonstrate how to programatically switch between online Kinect
grabbing and offline parsing a Rawlog dataset. Refer to the launch
of the grabbing thread in Test_KinectOnlineOffline()
*/

#include <mrpt/hwdrivers/CKinect.h>
#include <mrpt/gui.h>
#include <mrpt/slam/CColouredPointsMap.h>
#include <mrpt/synch/CThreadSafeVariable.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/slam/CRawlog.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/opengl/CFrustum.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/vision.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <memory> // for std::auto_ptr, unique_ptr


#include "BackProjectionSearch.h"
using namespace BackProjectionICP;

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::gui;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace std;


// ------------------------------------------------------
//				Test_KinectOnlineOffline
// ------------------------------------------------------
void Test_KinectOnlineOffline(bool is_online, const string &rawlog_file = string())
{
	CFileGZInputStream  dataset;

	// Offline:
	// ---------------------
	if (!dataset.open(rawlog_file))
		throw std::runtime_error("Couldn't open rawlog dataset file for input...");

	CImage::IMAGES_PATH_BASE = CRawlog::detectImagesDirectory(rawlog_file);

	const int width = 640;
	const int height = 480;

	const int xOffset = 0;
	const int yOffset = 0;

	const float fx = 520.9f;
	const float fy = 521.5f;
	const float cx = 325.6f;
	const float cy = 249.7f;

	int radiusSearch_pixel = 20;
	int pixelStepSize = 2;

	BackProjectionSearch bpSearch(fx, fy, cx, cy, pixelStepSize, radiusSearch_pixel, width, height);

	
	CObservationPtr obs;
	do
	{
		try {
			dataset >> obs;
		}
		catch (std::exception &e) {
			throw std::runtime_error(string("\nError reading from dataset file (EOF?):\n") + string(e.what()));
		}
		ASSERT_(obs.present())
	} while (!IS_CLASS(obs, CObservation3DRangeScan));

	CObservation3DRangeScanPtr lastObs = CObservation3DRangeScanPtr(obs);
	lastObs->load(); // *Important* This is needed to load the range image if stored as a separate file.

	std::ofstream transformationResultFile;
	transformationResultFile.open("transformResult.txt");

	int index = 1;
	bool end = false;

	while (true)
	{
		CObservationPtr obs;
		do
		{
			try {
				dataset >> obs;
			}
			catch (std::exception &e) {
				end = true;
				break;
				//throw std::runtime_error(string("\nError reading from dataset file (EOF?):\n") + string(e.what()));
			}
			ASSERT_(obs.present())
		} while (!IS_CLASS(obs, CObservation3DRangeScan));

		if (end)
			break;

		CObservation3DRangeScanPtr newObs = CObservation3DRangeScanPtr(obs);
		newObs->load(); // *Important* This is needed to load the range image if stored as a separate file.

		//if (index < 20)
		//{
		//	index++;
		//	lastObs = newObs;
		//	continue;
		//}

		std::cout << "---------------------------- " << index << " ----------------------------" << std::endl;;
		Eigen::Matrix4f transformation = bpSearch.doIcp(lastObs->rangeImage.block(yOffset, xOffset, height, width), newObs->rangeImage.block(yOffset, xOffset, height, width), transformationResultFile);
		std::cout << "transformation: " << std::endl << transformation << std::endl << std::endl;
		std::cout << std::endl;

		if (index % 10 == 0)
			transformationResultFile.flush();

		//cin.get();

		lastObs = newObs;
		index++;
	}

	cin.get();
}


int main(int argc, char **argv)
{
	try
	{
		// Online
		//Test_KinectOnlineOffline(true);

		// Offline:
		//cout << "Using offline operation" << endl;
		Test_KinectOnlineOffline(false, "rgbd_dataset_freiburg2_pioneer_slam.rawlog");

		mrpt::system::sleep(50);
		return 0;

	}
	catch (std::exception &e)
	{
		std::cout << "EXCEPCION: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Another exception!!");
		return -1;
	}

}
