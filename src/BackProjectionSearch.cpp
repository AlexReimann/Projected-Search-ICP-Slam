#include "BackProjectionSearch.h"

#include <boost\chrono.hpp>

namespace BackProjectionICP
{

	BackProjectionSearch::BackProjectionSearch(float fx, float fy, float cx, float cy, int pixelStepSize = 2, int radiusSearch_pixel = 20, int width = 640, int height = 480) :
		fx_(fx), fy_(fy), cx_(cx), cy_(cy), pixelStepSize_(pixelStepSize), radiusSearch_pixel_(radiusSearch_pixel), width_(width),
		height_(height), pixelWalker_(radiusSearch_pixel, width, fx, fy), coordinateConverter_(fx, fy, cx, cy, height, width)
	{
		initialize();
	}


	BackProjectionSearch::~BackProjectionSearch()
	{
	}

	void BackProjectionSearch::initialize()
	{
		xBorder_ = width_ - radiusSearch_pixel_;
		yBorder_ = height_ - radiusSearch_pixel_;

		int linearOffsetY = 0;

		int maxNumberOfMatches = ((yBorder_ - radiusSearch_pixel_) / pixelStepSize_) * ((xBorder_ - radiusSearch_pixel_) / pixelStepSize_);

		//calculate indicies
		for (int y = radiusSearch_pixel_; y < yBorder_; y = y + pixelStepSize_)
		{
			linearOffsetY = y * width_;

			for (int x = radiusSearch_pixel_; x < xBorder_; x = x + pixelStepSize_)
			{
				searchIndiciesOffsets_.push_back(linearOffsetY + x);
			}
		}
	}

	Eigen::Matrix4f BackProjectionSearch::doIcp(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& oldFrame,
		const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& newFrame, std::ofstream& resultOutputFile)
	{
		boost::chrono::thread_clock::time_point startOfIcp = boost::chrono::thread_clock::now();

		PointCoordinateMatrices oldPoints3d = coordinateConverter_.projectTo3d(oldFrame);
		PointCoordinateMatrices newPoints3d = coordinateConverter_.projectTo3d(newFrame);

		Eigen::IOFormat matlabFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", "\n", "", "", "", "\n");

		//std::ostringstream saveFileNameString;
		//saveFileNameString << "old.csv";
		//saveMatrixf(oldFrame, saveFileNameString.str());

		//std::ostringstream saveFileNameString1;
		//saveFileNameString1 << "old3dx.csv";
		//saveMatrixf(oldPoints3d.x, saveFileNameString1.str());

		//std::ostringstream saveFileNameString3;
		//saveFileNameString3 << "old3dy.csv";
		//saveMatrixf(oldPoints3d.y, saveFileNameString3.str());

		//std::ostringstream saveFileNameString4;
		//saveFileNameString4 << "old3dz.csv";
		//saveMatrixf(oldPoints3d.z, saveFileNameString4.str());

		//std::ostringstream saveFileNameString2;
		//saveFileNameString2 << "new.csv";
		//saveMatrixf(newFrame, saveFileNameString2.str());

		std::list<int> searchIndiciesOffsetsFiltered = std::list<int>(searchIndiciesOffsets_);

		//std::cout << "oldPoints3d x " << std::endl << oldPoints3d.x << std::endl;
		//std::cout << "oldPoints3d y " << std::endl << oldPoints3d.y << std::endl;
		//std::cout << "oldPoints3d z " << std::endl << oldPoints3d.z << std::endl << std::endl;
		//
		//std::cout << "newPoints3d x " << std::endl << newPoints3d.x << std::endl;
		//std::cout << "newPoints3d y " << std::endl << newPoints3d.y << std::endl;
		//std::cout << "newPoints3d z " << std::endl << newPoints3d.z << std::endl << std::endl;

		//TODO hard coded number of iterations for now
		int numberOfIterations = 10;
		Eigen::MatrixXf matches;
		Eigen::Matrix4f transformation;
		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> oldPoints3dMatrix = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Ones(4, width_ * height_);
		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> transformedOldPoints;

		Eigen::Matrix4f fullTransformation = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f fullTransformationTransposed = Eigen::Matrix4f::Identity();

		for (int i = 0; i < numberOfIterations; ++i)
		{
			//std::cout << "searchIndiciesOffsetsFiltered start: " << searchIndiciesOffsetsFiltered.size() << std::endl;
			//std::cout << "-----------------------Start of iteration " << i << " -----------------------" << std::endl;
			matches = runSearch(oldPoints3d, newPoints3d, searchIndiciesOffsetsFiltered);


			//std::cout << "searchIndiciesOffsetsFiltered: " << searchIndiciesOffsetsFiltered.size() << std::endl;
			//Just to show "goodness", should decrease
			std::cout << "distance matches: " << calcAverageSquaredDistance(matches.topRows(3), matches.bottomRows(3)) << std::endl;
			transformation = estimateTransformation(matches);
			oldPoints3dMatrix.row(0) = oldPoints3d.x;
			oldPoints3dMatrix.row(1) = oldPoints3d.y;
			oldPoints3dMatrix.row(2) = oldPoints3d.z;
			transformedOldPoints = transformation * oldPoints3dMatrix;
			fullTransformation = fullTransformation * transformation;
			oldPoints3d.x = transformedOldPoints.row(0);
			oldPoints3d.y = transformedOldPoints.row(1);
			oldPoints3d.z = transformedOldPoints.row(2);

			//std::cin.get();
			//std::cout << "oldPoints3d x transformed" << std::endl << oldPoints3d.x << std::endl;
			//std::cout << "oldPoints3d y transformed" << std::endl << oldPoints3d.y << std::endl;
			//std::cout << "oldPoints3d z transformed" << std::endl << oldPoints3d.z << std::endl << std::endl;
			//std::cout << "-------------------------End of iteration-------------------------" << std::endl;
		}
		
		//if (resultOutputFile.is_open())
		//{
		//	resultOutputFile << fullTransformation.format(matlabFormat);
		//}

		boost::chrono::milliseconds icpTime = boost::chrono::duration_cast<boost::chrono::milliseconds>(boost::chrono::thread_clock::now() - startOfIcp);

		//Transposing transformation matrix:
		//   R   | -R'*t
		// ------ -------
		// 0 0 0 |   1

		fullTransformationTransposed.block<3, 3>(0, 0) = fullTransformation.block<3, 3>(0, 0).transpose();
		fullTransformationTransposed.block<3, 1>(0, 3) = -fullTransformation.block<3, 3>(0, 0).transpose() * fullTransformation.block<3, 1>(0, 3);

		return fullTransformation;
	}

	Eigen::MatrixXf BackProjectionSearch::runSearch(const PointCoordinateMatrices& oldPoints3d, const PointCoordinateMatrices& newPoints3d, std::list<int>& searchIndiciesOffsetsFiltered)
	{
		float oldX = 0.0f;
		float oldY = 0.0f;
		float dx = 0.0f;
		float dy = 0.0f;

		int linearOffset = 0;
		int linearOffsetBackProjection = 0;
		int linearOffsetMinSearch = 0;

		std::vector<indexValuePair>::iterator minDistanceIterator;

		float currentZ = 0.0f;
		float diffZ = 0.0f;
		float absMinZDiff = 0.0f;
		float zDiff = 0.0f;

		float currentMinDistance = 0.0f;
		float tempDistance = 0.0f;
		int linOffsetToClostestPoint = 0;

		int breakCount = 0;
		int zeroPixelCount = 0;
		int countInnerLoop = 0;

		int maxNumberOfMatches = ((yBorder_ - radiusSearch_pixel_) / pixelStepSize_) * ((xBorder_ - radiusSearch_pixel_) / pixelStepSize_);
		int matchIndex = 0;

		boost::chrono::thread_clock::time_point iterationStart = boost::chrono::thread_clock::now();

		const float* dataPointerOld = oldPoints3d.z.data();
		const float* dataPointerOldX = oldPoints3d.x.data();
		const float* dataPointerOldY = oldPoints3d.y.data();

		const float* dataPointerSearch = newPoints3d.z.data();
		const float* dataPointerSearchX = newPoints3d.x.data();
		const float* dataPointerSearchY = newPoints3d.y.data();

		const float* currentDataPointer = dataPointerOld;

		Eigen::Matrix2Xi matchIndicies = Eigen::Matrix2Xi::Constant(2, maxNumberOfMatches, -1);
		Eigen::MatrixXf matches = Eigen::MatrixXf::Constant(6, maxNumberOfMatches, -1);

		std::list<int>::iterator offsetIterator = searchIndiciesOffsetsFiltered.begin();

		int k = 0;

		//for (int i = 0, size = searchImage.size(); i < size; ++i)
		//only iterator around the inner part, so we don't get border issues
		//also the outer part could be new, so we can't map it anyways
		while (offsetIterator != searchIndiciesOffsetsFiltered.end())
		{
			linearOffset = (*offsetIterator);
			currentDataPointer = dataPointerOld + linearOffset;
			currentZ = *currentDataPointer;

			k++;

			if (currentZ == 0)
			{
				searchIndiciesOffsetsFiltered.erase(offsetIterator++);
				zeroPixelCount++;
				continue;
			}
					

			pixelWalker_.reset();
			minDistanceIterator = pixelWalker_.get();
			

			//calculate for the first time to have a start value
			oldX = *(dataPointerOldX + linearOffset);
			oldY = *(dataPointerOldY + linearOffset);

			linearOffsetBackProjection = calculateBackProjectionImageLinearIndex(oldX, oldY, currentZ);

			//std::cout << "x, y, z: " << oldX << ", " << oldY << ", " << currentZ << std::endl;
			//std::cout << "linearOffsetBackProjection x linearOffset: " << linearOffsetBackProjection << " x " << linearOffset << std::endl;

			linearOffsetMinSearch = linearOffsetBackProjection + minDistanceIterator->first;
			diffZ = *(dataPointerSearch + linearOffsetMinSearch);
			linOffsetToClostestPoint = linearOffsetBackProjection + minDistanceIterator->first;;

			dx = oldX - *(dataPointerSearchX + linearOffsetMinSearch);
			dy = oldY - *(dataPointerSearchY + linearOffsetMinSearch);

			absMinZDiff = std::abs(currentZ - diffZ);
			currentMinDistance = dx*dx + dy*dy + absMinZDiff*absMinZDiff;

			while (pixelWalker_.next())
			{
				minDistanceIterator = pixelWalker_.get();
				linearOffsetMinSearch = linearOffsetBackProjection + minDistanceIterator->first;
				diffZ = *(dataPointerSearch + linearOffsetMinSearch);

				//TODO maybe make a matrix with squared values before, could be faster..
				//but maybe not, probably takes the same time to iterate the matrix / catch the value
				//if (currentMinDistance <= minDistanceIterator->second * diffZ * diffZ + (currentZ - diffZ) * (currentZ - diffZ))
				if (currentMinDistance <= minDistanceIterator->second * currentZ * currentZ)
				{
					breakCount++;
					break;
				}

				if (diffZ == 0)
					continue;

				zDiff = abs(currentZ - diffZ);

				if (absMinZDiff < zDiff)
					continue;

				countInnerLoop++;

				dx = oldX - *(dataPointerSearchX + linearOffsetMinSearch);
				dy = oldY - *(dataPointerSearchY + linearOffsetMinSearch);

				tempDistance = dx*dx + dy*dy + zDiff*zDiff;

				if (tempDistance < currentMinDistance)
				{
					currentMinDistance = tempDistance;
					linOffsetToClostestPoint = linearOffsetMinSearch;
					absMinZDiff = zDiff;
				}
			}

			if (diffZ == 0)
			{
				//the closest matching point has zero distance = invalid sensor value
				//there is no valid match around the point (= can't be matched)
				//probably noise or fluctuation / reflection sth.., so drop it
				searchIndiciesOffsetsFiltered.erase(offsetIterator++);
				zeroPixelCount++;
				continue;
			}

			matches(0, matchIndex) = oldX;
			matches(1, matchIndex) = oldY;
			matches(2, matchIndex) = currentZ;
			matches(3, matchIndex) = *(dataPointerSearchX + linOffsetToClostestPoint);
			matches(4, matchIndex) = *(dataPointerSearchY + linOffsetToClostestPoint);
			matches(5, matchIndex) = *(dataPointerSearch + linOffsetToClostestPoint);

			matchIndicies(0, matchIndex) = linearOffset;
			matchIndicies(1, matchIndex) = linOffsetToClostestPoint;
			matchIndex++;
			offsetIterator++;
		}

		matchIndicies.conservativeResize(2, matchIndex);
		matches.conservativeResize(6, matchIndex);

		//std::cout << "matchIndicies last row: " << std::endl << matchIndicies.col(matchIndicies.cols() - 1) << std::endl << std::endl;
		//std::cout << "matches last row: " << std::endl << matches.col(matches.cols() - 1) << std::endl << std::endl;
		//std::cout << "searchIndiciesOffsetsFiltered: " << searchIndiciesOffsetsFiltered.size() << std::endl;
		

		//std::ostringstream saveFileNameString;
		//saveFileNameString << "matches.csv";
		//saveMatrix(matchIndicies, saveFileNameString.str());

		boost::chrono::milliseconds iterationTime = boost::chrono::duration_cast<boost::chrono::milliseconds>(boost::chrono::thread_clock::now() - iterationStart);

		//std::cout << matchIndicies.cols() << ", ";
		//std::cout << zeroPixelCount << ", ";
		//std::cout << breakCount << ", ";
		//std::cout << countInnerLoop << ", ";
		//std::cout << iterationTime.count() << std::endl;

		//std::cout << "Search finished after " << iterationTime.count() << " ms" << std::endl;
		//std::cout << "Break count: " << breakCount << std::endl;
		//std::cout << "Innter loop count: " << countInnerLoop << std::endl;
		//std::cout << "Number of matches: " << matches.cols() << std::endl;

		//return matchIndicies;
		return matches;
	}

	int BackProjectionSearch::calculateBackProjectionImageLinearIndex(float x, float y, float z)
	{
		int xFramePosition = std::min(xBorder_, std::max(radiusSearch_pixel_, (int)std::roundf(x * (fx_ / z) + cx_ - 1)));
		int yFramePosition = std::min(yBorder_, std::max(radiusSearch_pixel_, (int)std::roundf(y * (fy_ / z) + cy_ - 1))); 

		return width_ * yFramePosition + xFramePosition;
	}

	Eigen::Matrix4f BackProjectionSearch::estimateTransformation(const Eigen::MatrixXf& matches)
	{
		int numberOfMatches = matches.cols();

		Eigen::Vector3f centerPointOld = matches.topRows(3).rowwise().mean();
		Eigen::Vector3f centerPointNew = matches.bottomRows(3).rowwise().mean();

		float meanNormalizer = 1.0f / numberOfMatches;

		//std::cout << "size, meanNormalizer: " << numberOfMatches  << "," << meanNormalizer << std::endl;


		//std::cout << "centerPointsOld: " << std::endl << centerPointOld << std::endl << std::endl;
		//std::cout << "centerPointsNew: " << std::endl << centerPointNew << std::endl << std::endl;

		//std::cout << "oldPoints3d matches" << std::endl << matches.topRows(3) << std::endl;
		//std::cout << "newPoints3d matches" << std::endl << matches.bottomRows(3) << std::endl;

		Eigen::Matrix3Xf distanceToCenterOld = matches.topRows(3).colwise() - centerPointOld;
		//std::cout << "distanceToCenterOld" << std::endl << distanceToCenterOld << std::endl;
		//distanceToCenterOld = distanceToCenterOld.array() * meanNormalizer;
		//std::cout << "distanceToCenterOld normalized" << std::endl << distanceToCenterOld << std::endl;
		Eigen::Matrix3Xf distanceToCenterNew = matches.bottomRows(3).colwise() - centerPointNew;
		//std::cout << "distanceToCenterNew" << std::endl << distanceToCenterNew << std::endl;
		//distanceToCenterNew = distanceToCenterNew.array() * meanNormalizer;
		//std::cout << "distanceToCenterNew normalized" << std::endl << distanceToCenterNew << std::endl;

		//TODO (later) weighting should go in here
		
		Eigen::MatrixXf correlationMatrix = distanceToCenterNew * distanceToCenterOld.transpose();

		//std::cout << "correlationMatrix: " << std::endl << correlationMatrix << std::endl << std::endl;

		Eigen::JacobiSVD<Eigen::MatrixXf> svd(correlationMatrix, Eigen::ComputeThinU | Eigen::ComputeThinV);

		Eigen::Matrix3f helperRotation = Eigen::Matrix3f::Identity();
		helperRotation(2, 2) = (svd.matrixU() * svd.matrixV().transpose()).determinant();
		Eigen::Matrix3f rotation = svd.matrixV() * helperRotation * svd.matrixU().transpose();
		Eigen::Vector3f translation = centerPointOld - rotation * centerPointNew;

		Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

		transformation.block<3, 3>(0, 0) = rotation.transpose();
		transformation.block<3, 1>(0, 3) = -rotation.transpose() * translation;

		//std::cout << "rotation: " << std::endl << rotation << std::endl << std::endl;
		//std::cout << "translation: " << std::endl << translation << std::endl << std::endl;

		return transformation;
	}

	double BackProjectionSearch::calcAverageSquaredDistance(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& a,
		const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& b)
	{
		double sum = 0;
		for (int i = 0; i < a.cols(); ++i)
		{/*
			std::cout << "a: " << std::endl << a.col(i) << std::endl << std::endl;
			std::cout << "b: " << std::endl << b.col(i) << std::endl << std::endl;*/
			sum += (a(0, i) - b(0, i)) * (a(0, i) - b(0, i));
			sum += (a(1, i) - b(1, i)) * (a(1, i) - b(1, i));
			sum += (a(2, 1) - b(2, 1)) * (a(2, 1) - b(2, 1));
		}

		return sum;
	}

	void BackProjectionSearch::saveMatrix(const Eigen::MatrixXi& matrix, std::string fileName)
	{
		Eigen::IOFormat matlabFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n", "", "", "", "");

		std::ofstream matrixFile;
		matrixFile.open(fileName);

		if (matrixFile.is_open())
		{
			matrixFile << matrix.format(matlabFormat);
		}
	}

	void BackProjectionSearch::saveMatrixf(const Eigen::MatrixXf& matrix, std::string fileName)
	{
		Eigen::IOFormat matlabFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n", "", "", "", "\n");

		std::ofstream matrixFile;
		matrixFile.open(fileName);

		if (matrixFile.is_open())
		{
			matrixFile << matrix.format(matlabFormat);
		}
	}
}//end namespace BackProjectionICP
