#ifndef GENERAL_HEAD_H

#define GENERAL_HEAD_H

#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;

typedef tuple<pair<int, int>, int> PointInfoTYPE;
// tuple(pair(row, col), value)
typedef vector<vector<int>> BandSummitsTYPE;
//vector(pointsummits, valuesummits)
typedef tuple<int, int, int, BandSummitsTYPE> LaneInfoTYPE;
//tuple(leftborder, rightborder, Bandcol, BandSummitsTYPE)
typedef tuple<int, int, int, int> BandAreaTYPE;
//tuple(top, bottom, left, right)

namespace General
{
	int GetMaxDistance(const cv::Mat& source, int distance = 5);

	int ContainingIndex(int value, const vector<int>& testVec);

	int ContainingIndex(int value, const vector<int>& testVec, int MAXlimit);

	int ContainingIndex(int value, const vector<double>& testVec, int MAXlimit);

	int GetStandardBandLength(const cv::Mat& source, int THRESHOLDdistance = 0, int MAXvalue = 0);

	cv::Mat Gray16ToBGR(const cv::Mat& source);

	cv::Mat Divide3X5(const cv::Mat& source);

	cv::Mat ExtendBackground(const cv::Mat& source, int mode = 1);

	pair<int, int> GetBandHeight(const cv::Mat& source, int row, int col, int STANDARDvalue);

	pair<int, int> GetBandLength(const cv::Mat& source, int row, int col, int STANDARDvalue,
		int leftLIMIT, int rightLIMIT);

	vector<int> GetPointDifferenceVec(const cv::Mat& source, int distance);

	PointInfoTYPE GetMaxPointInfo(const cv::Mat& source);
}

#endif

