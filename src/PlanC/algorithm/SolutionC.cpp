#include "GeneralHead.h"
#include "ClassHead.h"
#include "PlanAHead.h"
#include "PlanBHead.h"
#include "PlanCHead.h"

#include <math.h>
#include <iostream>

using namespace std;
using namespace cv;
using namespace General;

typedef tuple<pair<int, int>, int> PointInfoTYPE;
//tuple(pair(row, col), value)
typedef vector<vector<int>> BandSummitsTYPE;
//vector(pointsummits, valuesummits)
typedef tuple<int, int, int, BandSummitsTYPE> LaneInfoTYPE;
//tuple(leftborder, rightborder, Bandcol, BandSummitsTYPE)
typedef tuple<int, int, int, int> BandAreaTYPE;
//tuple(top, bottom, left, right)
typedef pair<BandAreaTYPE, int> BandInfoTYPE;
//pair(BandAreaTYPE, Bandlength) 

void SolutionPlanC(const cv::Mat& source, string filenum)
{
	using namespace PlanC;

	const string filepath = "C://Users//Father Yuyue//Desktop//";
	const string filename = filepath + filenum;

	cv::Mat img = ExtendBackground(source, 0);
	cv::Mat Colored_img = Gray16ToBGR(img);//

	int THRESHOLDdistance = GetMaxDistance(img) / 10;
	int MAXvalue = get<1>(GetMaxPointInfo(img));
	int STANDARDlength = GetStandardBandLength(img, THRESHOLDdistance, MAXvalue);

	pair<int, int> allleftANDright = GetAllLaneLength_Rough(img, THRESHOLDdistance);
	int leftBORDER = allleftANDright.first;
	int rightBORDER = allleftANDright.second;

	vector<vector<Band>> BandList;

	AnalyzeAllLane(img, BandList,
		THRESHOLDdistance, STANDARDlength, MAXvalue,
		5, img.cols - 6);

	BandList = SortLane(BandList);

	for (int i = 0; i < BandList.size(); i++)
		if (BandList[i].size() > 0)
			for (int j = 0; j < BandList[i].size(); j++)
				DrawRect(Colored_img, BandList[i][j], 2);

	imwrite(filepath + "origin result" + filenum, Colored_img);
}