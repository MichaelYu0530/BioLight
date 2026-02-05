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

void SolutionPlanA(const cv::Mat& img, const string& filenum)
{
	using namespace PlanA;

	const string filepath = "C://Users//Father Yuyue//Desktop//";
	const string filename = filepath + filenum;

	Mat Processed_img = Divide3X5(img);//
	Mat Colored_Origin_img = Gray16ToBGR(img);//
	Processed_img = ExtendBackground(Processed_img);//
	Mat Colored_Processed_img = Gray16ToBGR(Processed_img);
	//cv::imwrite(filepath + "extendedresult" + filenum, Processed_img);

	vector<LaneInfoTYPE> laneinfoVec = GetLaneInfo(Processed_img, 5, Processed_img.cols - 5);
	//

	vector<LaneInfoTYPE> REVISEDlaneinfoVec = CheckAndReviseBandLength(Processed_img, laneinfoVec);

	vector<LaneInfoTYPE> IMPROVEDlaneinfoVec = DeleteSurplusSummits(Processed_img, REVISEDlaneinfoVec);

	for (int i = 0; i < IMPROVEDlaneinfoVec.size(); i++)
	{
		const LaneInfoTYPE& IMPROVEDlaneinfo = IMPROVEDlaneinfoVec[i];

		const vector<BandAreaTYPE>& BandareaVec = LocateBandInSingleLane(Processed_img, IMPROVEDlaneinfo);

		for (int j = 0; j < BandareaVec.size(); j++)
		{
			DrawRectInProcessed(Colored_Processed_img, BandareaVec[j]);
			DrawRectInOrigin(Colored_Origin_img, BandareaVec[j]);
		}
	}

	cv::imwrite(filepath + "origin final" + filenum, Colored_Origin_img);
	cv::imwrite(filepath + "processed final" + filenum, Colored_Processed_img);
}