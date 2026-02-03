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

void SolutionPlanB(const cv::Mat& source, string filenum)
{
	using namespace PlanB;

	const string filepath = "C://Users//Father Yuyue//Desktop//";
	const string filename = filepath + filenum;

	cv::Mat Processed_img = Divide3X5(source);//
	Processed_img = ExtendBackground(Processed_img);//
	cv::Mat Colored_Origin_img = Gray16ToBGR(source);//
	cv::Mat Colored_Processed_img = Gray16ToBGR(Processed_img);
	imwrite(filepath + "extendedresult" + filenum, Processed_img);

	int THRESHOLDdistance = GetMaxDistance(Processed_img) / 10;
	int MAXvalue = get<1>(GetMaxPointInfo(Processed_img));
	int STANDARDlength = GetStandardBandLength(Processed_img, THRESHOLDdistance, MAXvalue);

	pair<int, int> allleftANDright = GetAllLaneLength(Processed_img, THRESHOLDdistance);
	int leftBORDER = allleftANDright.first;
	int rightBORDER = allleftANDright.second;

	vector<vector<Band>> Processed_BandList;

	AnalyzeAllLane(Processed_img, Processed_BandList,
		THRESHOLDdistance, STANDARDlength, MAXvalue,
		5, Processed_img.cols - 6);

	vector<vector<Band>> Origin_BandList = TransferToOrigin(Processed_BandList);

	ReviseBandLength_Primary(source, Origin_BandList, THRESHOLDdistance, MAXvalue);

	ReviseBandLength_Advanced(source, Origin_BandList, THRESHOLDdistance, MAXvalue);

	for (int i = 0; i < Origin_BandList.size(); i++)
		for (int j = 0; j < Origin_BandList[i].size(); j++)
		{
			Band& THISBand = Origin_BandList[i][j];

			float slope = GetSlope(source, THISBand, THRESHOLDdistance, MAXvalue);
			THISBand.slope = slope;
			GetBandLineOffsets(source, THISBand, THRESHOLDdistance, MAXvalue, Colored_Origin_img);


		}

	imwrite(filepath + "processed result" + filenum, Colored_Processed_img);
	imwrite(filepath + "origin result" + filenum, Colored_Origin_img);
}