#include <iostream>
#include <fstream>

#include "ConverterPlaneFromTo3d.h"

namespace BackProjectionICP
{

	ConverterPlaneFromTo3d::ConverterPlaneFromTo3d(float fx, float fy, float cx, float cy, int height, int width)
	{
		Eigen::VectorXf colIndicies = Eigen::VectorXf::LinSpaced(Eigen::Sequential, width, 1, (float)width);
		Eigen::VectorXf rowIndicies = Eigen::VectorXf::LinSpaced(Eigen::Sequential, height, 1, (float)height);
		Eigen::VectorXf onesColSize = Eigen::VectorXf::Ones(width, 1);
		Eigen::VectorXf onesRowSize = Eigen::VectorXf::Ones(height, 1);
		Eigen::MatrixXf indiciesMatrixAxisX = onesRowSize * colIndicies.transpose(); //row = 1, 2, 3, 4, ..
		Eigen::MatrixXf indiciesMatrixAxisY = rowIndicies * onesColSize.transpose();

		xAdjustment_ = (indiciesMatrixAxisX.array() - cx) / fx;
		yAdjustment_ = (indiciesMatrixAxisY.array() - cy) / fy;


		Eigen::IOFormat matlabFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n", "", "", "", "");
		std::ostringstream saveFileNameString0;
		saveFileNameString0 << "indiciesMatrixAxisX.csv";

		std::ofstream matrixFile;
		matrixFile.open(saveFileNameString0.str());

		if (matrixFile.is_open())
		{
			matrixFile << indiciesMatrixAxisX.format(matlabFormat);
		}

		std::ostringstream saveFileNameString;
		saveFileNameString << "xAdjustment.csv";

		std::ofstream matrixFile1;
		matrixFile1.open(saveFileNameString.str());

		if (matrixFile1.is_open())
		{
			matrixFile1 << xAdjustment_.format(matlabFormat);
		}
	}

	PointCoordinateMatrices	ConverterPlaneFromTo3d::projectTo3d(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> image)
	{
		PointCoordinateMatrices reshapedCoordinates;

		//std::cout << "image: " << std::endl << image << std::endl << std::endl;

		//As defined for the example data sets to get the distance in meters Z = depth / 5000 TODO change for kinect data (maybe is the same...)
		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> z = image.array() / 1;// 5000;
		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> x = z.cwiseProduct(xAdjustment_);
		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> y = z.cwiseProduct(yAdjustment_);

		//std::cout << "x: " << std::endl << x << std::endl << std::endl;
		//std::cout << "y: " << std::endl << y << std::endl << std::endl;
		//std::cout << "z: " << std::endl << z << std::endl << std::endl;

		reshapedCoordinates.x = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(x.data(), 1, image.size());
		reshapedCoordinates.y = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(y.data(), 1, image.size());
		reshapedCoordinates.z = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(z.data(), 1, image.size());

		return reshapedCoordinates;
	}

}//end namespace BackProjectionICP