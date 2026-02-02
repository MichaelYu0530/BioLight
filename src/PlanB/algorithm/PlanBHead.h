#ifndef PLAN_B_HEAD_H

#define PLAN_B_HEAD_H

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

void SolutionPlanB(const cv::Mat& source, string filenum);

namespace PlanB
{
	void ClearRowArea(cv::Mat& source, int top, int bottom);

	void CheckAdjacentBands(const cv::Mat& source, vector<Band>& BandareaVec,
		int THRESHOLDdistance, int MAXvalue);

	void ReviseBandLength_Primary(const cv::Mat& source, vector<vector<Band>>& BandList,
		int THRESHOLDdistance, int MAXvalue);

	void ReviseBandLength_Advanced(const cv::Mat& source, vector<vector<Band>>& BandList,
		int THRESHOLDdistance, int MAXvalue);

	void GetBandLineOffsets(const cv::Mat& source, Band& THISBand,
		int THRESHOLDdistance, int MAXvalue, cv::Mat& colored_img);

	void DrawBandShape(const cv::Mat& colored_img, Band Bandarea, int thickness);

	bool AnalyzeAllLane(const cv::Mat& source, vector<vector<Band>>& ALLBandareaVec,
		int THRESHOLDdistance, int STANDARDlength, int MAXvalue,
		int leftlimit, int rightlimit);

	bool IsIsolatedPoint(const cv::Mat& source, int row, int col, int THRESHOLDdistance, Direction direction);

	bool IsBackGroundArea(const cv::Mat& source, int THRESHOLDdistance, int leftlimit, int rightlimit);

	bool IsTilted(const cv::Mat& source, Band CURRENTBand, int THRESHOLDdistance, int MAXvalue);

	float GetRate(float ratio);

	float GetSlope(const cv::Mat& source, Band CURRENTBand, int THRESHOLDdistance, int MAXvalue);

	cv::Mat GetMidMat(const cv::Mat& source, int midcol, int toplimit, int bottomlimit);

	pair<int, int> GetBandHeight_Rough(const cv::Mat& source, int row, int col,
		int THRESHOLDdistance, int MAXvalue, int STANDARDvalue = 0);

	pair<int, int> GetBandLength_Rough(const cv::Mat& source, int row, int col,
		int THRESHOLDdistance, int MAXvalue, int STANDARDvalue = 0);

	pair<int, int> GetBandHeight_Accurate(const cv::Mat& source, int row, int col,
		int THRESHOLDdistance, int MAXvalue, int STANDARDvalue);

	pair<int, int> GetBandLength_Accurate(const cv::Mat& source, int row, int col,
		int THRESHOLDdistance, int MAXvalue, int STANDARDvalue);

	pair<int, int> GetReferenceLaneLength(const cv::Mat& source, int STANDARDlength, int THRESHOLDdistance);

	pair<int, int> GetAllLaneLength(const cv::Mat& source, int THRESHOLDdistance);

	pair<int, int> GetAllBandHeight(const cv::Mat& source, int THRESHOLDdistance, int leftlimit, int rightlimit);

	pair<int, int> FindBackgroundOffset(const cv::Mat& midMat, int THRESHOLDdistance, int MAXvalue,
		int Bandtop, int Bandbottom, int toplimit, int bottomlimit);

	Band FindBandByMaxPoint(const cv::Mat& source, const cv::Mat& midMat,
		int THRESHOLDdistance, int STANDARDlength, int MAXvalue,
		int leftlimit, int rightlimit, int toplimit, int bottomlimit);

	vector<Band> SortBand(vector<Band>& BandareaVec);

	vector<Band> AnalyzeSingleLane(const cv::Mat& source,
		int THRESHOLDdistance, int STANDARDlength, int MAXvalue,
		int leftlimit, int rightlimit);

	vector<vector<Band>> SortLane(vector<vector<Band>>& BandList);

	vector<vector<Band>> TransferToOrigin(const vector<vector<Band>>& Processed);
}

#endif