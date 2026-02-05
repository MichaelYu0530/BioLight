#ifndef PLAN_A_HEAD_H

#define PLAN_A_HEAD_H

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

void SolutionPlanA(const cv::Mat& img, const string& filenum);

namespace PlanA
{
	bool IsBackground(const cv::Mat& source, int col);

	bool IsSummit(const vector<int>& Difference, int index);

	bool IsInRectArea(int testrow, int testcol, BandAreaTYPE Bandarea);

	bool IsInRectArea(cv::Point testpoint, cv::Point LTcorner, cv::Point RBcorner);

	void InsertPoint(BandSummitsTYPE& summits, int point, int value);

	void OutputLaneInfo(const vector<LaneInfoTYPE>& BandinfoVec);

	void DrawRectInOrigin(const cv::Mat& origin, const BandAreaTYPE& Bandarea);

	void DrawRectInProcessed(const cv::Mat& processed, const BandAreaTYPE& Bandarea);

	LaneInfoTYPE GetLaneInfoSpecial(const cv::Mat& source, int begin, int end);

	PointInfoTYPE GetStandardPointInfo(const cv::Mat& source, int row, int col);

	BandSummitsTYPE FindSummit(const cv::Mat& source, int col);

	BandSummitsTYPE GetSummitUnion(const vector<BandSummitsTYPE>& summitsVec);

	vector<LaneInfoTYPE> GetLaneInfo(const cv::Mat& source, int begin, int end);

	vector<LaneInfoTYPE> CheckAndReviseBandLength(const cv::Mat& source,
		const vector<LaneInfoTYPE>& BandinfoVec);

	vector<LaneInfoTYPE> DeleteSurplusSummits(const cv::Mat& source, const vector<LaneInfoTYPE>& BandinfoVec);

	vector<BandAreaTYPE> LocateBandInSingleLane(const cv::Mat& source, const LaneInfoTYPE& Bandinfo);
}

#endif