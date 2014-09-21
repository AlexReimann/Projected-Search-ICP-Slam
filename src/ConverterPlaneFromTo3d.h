#pragma once

#include <Eigen\Dense>

namespace BackProjectionICP
{
	struct PointCoordinateMatrices
	{
		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> x;
		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> y;
		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> z;
	};

	class ConverterPlaneFromTo3d
	{
	public:
		ConverterPlaneFromTo3d(float fx, float fy, float cx, float cy, int height, int width);

		PointCoordinateMatrices	projectTo3d(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> image);

	private:
		Eigen::MatrixXf xAdjustment_;
		Eigen::MatrixXf yAdjustment_;
	};

}//end namespace BackProjectionICP