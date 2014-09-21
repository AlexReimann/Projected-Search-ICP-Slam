#include "BackPropPixelOffsetWalker.h"

#include <iostream>
#include <vector>
#include <Eigen\Dense>

using namespace std;

namespace BackProjectionICP
{

	BackPropPixelOffsetWalker::BackPropPixelOffsetWalker(int radius_pixel, int imageWidth, float fx, float fy)
	{
		minDistanceSquaredLinearSorted_ = calculatMinDistanceSquaredEstimationSorted(radius_pixel, imageWidth, fx, fy);
		reset();
	}

	std::vector<indexValuePair> BackPropPixelOffsetWalker::calculatMinDistanceSquaredEstimationSorted(int radius_pixel, int imageWidth, float fx, float fy)
	{
		int diameter = 2 * radius_pixel + 1;
		float fx2_ = 1.0f / (fx * fx);
		float fy2_ = 1.0f / (fy * fy);

		Eigen::VectorXf indicies(diameter);
		Eigen::VectorXf indiciesPart = Eigen::VectorXf::LinSpaced(Eigen::Sequential, radius_pixel, 1, (float)radius_pixel);
		indicies << (indiciesPart.reverse() * -1), 0.0f, indiciesPart;

		Eigen::VectorXf ones = Eigen::VectorXf::Ones(diameter, 1);
		Eigen::MatrixXf indiciesMatrixAxisX = ones * indicies.transpose(); //row = 1, 2, 3, 4, ..
		Eigen::MatrixXf indiciesMatrixAxisXSquared = indiciesMatrixAxisX.cwiseProduct(indiciesMatrixAxisX);

		// indiciesMatrixAxisXSquared.transpose() = squared indicies of y-axis
		Eigen::MatrixXf minSquared = fx2_ * indiciesMatrixAxisXSquared + fy2_ * indiciesMatrixAxisXSquared.transpose();

		//cout << "minSquared:" << endl;
		//cout << minSquared << endl;

		std::vector<float> minSquaredLinear(diameter*diameter);
		//Has to be row major, because the used mobile robot programming framework (mrpt.org) uses eigen row major matricies
		//(and this is because most algorithms are explained / implemented going row wise)
		Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(minSquaredLinear.data(), diameter, diameter) = minSquared;

		std::vector<indexValuePair> minSquaredLinearSortedWithIndicies = sortWithIndicies(minSquaredLinear);

		int matrixIndex;
		int imageOffsetIndexX;
		int ImageOffsetIndexY;
		Eigen::MatrixXf indiciesMatrixAxisY = indiciesMatrixAxisX.transpose();
		int linearImageIndex;

		std::vector<indexValuePair>::iterator pairIterator = minSquaredLinearSortedWithIndicies.begin();

		for (std::vector<indexValuePair>::iterator pairIterator = minSquaredLinearSortedWithIndicies.begin();
			pairIterator != minSquaredLinearSortedWithIndicies.end();
			++pairIterator)
		{
			//cout << "pair original: " << pairIterator->first << " " << pairIterator->second << endl;

			matrixIndex = pairIterator->first;
			imageOffsetIndexX = (int)indiciesMatrixAxisX(matrixIndex);
			ImageOffsetIndexY = (int)indiciesMatrixAxisY(matrixIndex);

			linearImageIndex = ImageOffsetIndexY * imageWidth + imageOffsetIndexX;
			pairIterator->first = linearImageIndex;
		}

		return minSquaredLinearSortedWithIndicies;
	}

	static bool pairComperator(const indexValuePair& left, const indexValuePair& right)
	{
		return left.second < right.second;
	}

	std::vector<indexValuePair> BackPropPixelOffsetWalker::sortWithIndicies(const std::vector<float>& values)
	{
		std::vector<indexValuePair> indexValuePairs;

		for (unsigned int i = 0; i < values.size(); i++)
		{
			indexValuePairs.push_back(indexValuePair(i, values[i]));
		}

		std::sort(indexValuePairs.begin(), indexValuePairs.end(), pairComperator);

		return indexValuePairs;
	}

	void BackPropPixelOffsetWalker::reset()
	{
		pairIterator_ = minDistanceSquaredLinearSorted_.begin();
	}

	std::vector<indexValuePair>::iterator BackPropPixelOffsetWalker::get()
	{
		return pairIterator_;
	}

	bool BackPropPixelOffsetWalker::next()
	{
		pairIterator_++;

		if (pairIterator_ != minDistanceSquaredLinearSorted_.end())
			return true;

		return false;
	}

}//end namespace BackProjectionICP