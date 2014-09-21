#pragma once

#include <list>
#include <iostream>
#include <fstream>

#include "BackPropPixelOffsetWalker.h"
#include "ConverterPlaneFromTo3d.h"

namespace BackProjectionICP
{

	class BackProjectionSearch
	{
	public:
		BackProjectionSearch(float fx, float fy, float cx, float cy, int pixelStepSize, int radiusSearch_pixel, int width, int height);
		~BackProjectionSearch();

		Eigen::Matrix4f doIcp(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& oldFrame,
			const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& newFrame, std::ofstream& resultOutputFile);

	private:
		void initialize();
		Eigen::MatrixXf runSearch(const PointCoordinateMatrices& oldPoints3d,
			const PointCoordinateMatrices& newPoints3d, std::list<int>& searchIndiciesOffsetsFiltered);
		int calculateBackProjectionImageLinearIndex(float x, float y, float z);
		Eigen::Matrix4f estimateTransformation(const Eigen::MatrixXf& matches);
		double calcAverageSquaredDistance(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& a,
			const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& b);
		void saveMatrix(const Eigen::MatrixXi& matrix, std::string fileName);
		void saveMatrixf(const Eigen::MatrixXf& matrix, std::string fileName);

	private:
		const int width_;
		const int height_;

		int xBorder_;
		int yBorder_;

		const float fx_;
		const float fy_;
		const float cx_;
		const float cy_;

		int radiusSearch_pixel_;
		int pixelStepSize_;

		std::list<int> searchIndiciesOffsets_;

		BackPropPixelOffsetWalker pixelWalker_;
		ConverterPlaneFromTo3d coordinateConverter_;
	};

}//end namespace BackProjectionICP