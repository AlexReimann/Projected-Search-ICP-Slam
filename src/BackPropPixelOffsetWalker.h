#pragma once

#include <vector>

namespace BackProjectionICP
{
	typedef std::pair<int, float> indexValuePair;

	class BackPropPixelOffsetWalker
	{
	public:
		BackPropPixelOffsetWalker(int radius_pixel, int imageWidth, float fx, float fy);

		void reset();
		std::vector<indexValuePair>::iterator get();
		bool next();

	private:
		std::vector<indexValuePair>::iterator pairIterator_;
		std::vector<indexValuePair> minDistanceSquaredLinearSorted_;

	private:
		std::vector<indexValuePair> calculatMinDistanceSquaredEstimationSorted(int radius_pixel, int imageWidth, float fx, float fy);
		std::vector<indexValuePair> sortWithIndicies(const std::vector<float>& values);
	};
}//end namespace BackProjectionICP