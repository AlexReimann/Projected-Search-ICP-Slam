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
#include <Eigen\Dense>

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

	CColouredPointsMap  globalPtsMap;
	globalPtsMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;

	mrpt::gui::CDisplayWindow3D  win3D("Back Projection ICP Slam", 800, 600);

	win3D.setCameraAzimuthDeg(140);
	win3D.setCameraElevationDeg(20);
	win3D.setCameraZoom(8.0);
	win3D.setFOV(90);
	win3D.setCameraPointingToPoint(2.5, 0, 0);

	mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
	gl_points->setPointSize(2.5);

	mrpt::opengl::CPointCloudColouredPtr gl_points_map = mrpt::opengl::CPointCloudColoured::Create();
	gl_points_map->setPointSize(2.0);

	const double aspect_ratio = 480.0 / 640.0;

	mrpt::opengl::CSetOfObjectsPtr gl_cur_cam_corner = mrpt::opengl::stock_objects::CornerXYZSimple(0.4, 4);

	opengl::COpenGLViewportPtr viewInt;
	{
		mrpt::opengl::COpenGLScenePtr &scene = win3D.get3DSceneAndLock();

		// Create the Opengl object for the point cloud:
		scene->insert(gl_points_map);
		scene->insert(gl_points);
		scene->insert(mrpt::opengl::CGridPlaneXY::Create());

		scene->insert(gl_cur_cam_corner);

		const int VW_WIDTH = 350;	// Size of the viewport into the window, in pixel units.
		const int VW_HEIGHT = aspect_ratio*VW_WIDTH;

		// Create the Opengl objects for the planar images each in a separate viewport:
		viewInt = scene->createViewport("view2d_int");
		viewInt->setViewportPosition(2, 2, VW_WIDTH, VW_HEIGHT);
		viewInt->setTransparent(true);

		win3D.unlockAccess3DScene();
		win3D.repaint();
	}

	CPose3D currentCamPose_wrt_last;
	std::vector<TPose3D> camera_key_frames_path;  // The 6D path of the Kinect camera.
	camera_key_frames_path.push_back(TPose3D(0, 0, 0, 0, 0, 0));

	const int width = 640;
	const int height = 480;

	const int xOffset = 0;
	const int yOffset = 0;

	const float fx = 520.9f;
	const float fy = 521.5f;
	const float cx = 325.6f;
	const float cy = 249.7f;

	int radiusSearch_pixel = 10;
	int pixelStepSize = 4;

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

	globalPtsMap.clear();
	const CPose3D tempStartPose(0,0,0,0,0,0);
	lastObs->project3DPointsFromDepthImageInto(globalPtsMap, &tempStartPose);

	win3D.get3DSceneAndLock();
	gl_points_map->loadFromPointsMap(&globalPtsMap);
	win3D.unlockAccess3DScene();

	std::ofstream transformationResultFile;
	transformationResultFile.open("transformResult.csv");

	std::ofstream imageResultFile1;
	imageResultFile1.open("Image1.csv");

	std::ofstream imageResultFile2;
	imageResultFile2.open("Image2.csv");

	int index = 1;
	bool end = false;

	while (win3D.isOpen())
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

				if (index < 30)
		{
			index++;
			lastObs = newObs;
			continue;
		}

		win3D.get3DSceneAndLock();
		viewInt->setImageView(newObs->intensityImage);
		win3D.unlockAccess3DScene();

		std::cout << "---------------------------- " << index << " ----------------------------" << std::endl;;

		mrpt::math::CMatrixDouble44 transformation = bpSearch.doIcp(lastObs->rangeImage.block(yOffset, xOffset, height, width), newObs->rangeImage.block(yOffset, xOffset, height, width), transformationResultFile);
		std::cout << "transformation: " << std::endl << transformation << std::endl << std::endl;
		std::cout << std::endl;

		//Eigen::IOFormat matlabFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", "\n", "", "", "", "\n");

		//if (imageResultFile1.is_open())
		//{
		//	imageResultFile1 << lastObs->rangeImage.block(yOffset, xOffset, height, width).format(matlabFormat);
		//}

		//if (imageResultFile2.is_open())
		//{
		//	imageResultFile2 << newObs->rangeImage.block(yOffset, xOffset, height, width).format(matlabFormat);
		//}

		//if (index % 10 == 0)
		//	transformationResultFile.flush();

		CPose3D relativePose(transformation);
		const CPose3D new_keyframe_global = CPose3D(camera_key_frames_path.back()) + relativePose;
		camera_key_frames_path.push_back(TPose3D(new_keyframe_global));

		newObs->project3DPointsFromDepthImage();
		globalPtsMap.insertObservation(newObs.pointer());

		win3D.get3DSceneAndLock();
		gl_points_map->loadFromPointsMap(&globalPtsMap);
		win3D.unlockAccess3DScene();

		win3D.repaint();

		CColouredPointsMap localPntsMap;
		localPntsMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;
		newObs->project3DPointsFromDepthImageInto(localPntsMap, false);

		win3D.get3DSceneAndLock();

		gl_points->loadFromPointsMap(&localPntsMap);
		gl_points->setPose(new_keyframe_global);
		gl_cur_cam_corner->setPose(new_keyframe_global);

		win3D.unlockAccess3DScene();
		win3D.repaint();

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
