#include "GeneralHead.h"
#include "ClassHead.h"
#include "PlanBHead.h"

#include <math.h>
#include <iostream>

using namespace std;
using namespace PlanB;
using namespace General;

typedef tuple<pair<int, int>, int> PointInfoTYPE;
// tuple(pair(row, col), value)
typedef vector<vector<int>> BandSummitsTYPE;
//vector(pointsummits, valuesummits)
typedef tuple<int, int, int, BandSummitsTYPE> LaneInfoTYPE;
//tuple(leftborder, rightborder, Bandcol, BandSummitsTYPE)
typedef tuple<int, int, int, int> BandAreaTYPE;
//tuple(top, bottom, left, right)

void PlanB::ClearRowArea(cv::Mat& source, int top, int bottom)
{
	for (int r = top; r <= bottom; r++)
		for (int c = 0; c < source.cols; c++)
			source.at<ushort>(r, c) = 0;
}
//一行的灰度值设为0

void PlanB::CheckAdjacentBands(const cv::Mat& midMat, vector<Band>& BandareaVec,
	int THRESHOLDdistance, int MAXvalue)
{
	if (BandareaVec.size() == 0)
		return;

	for (vector<Band>::iterator iter = BandareaVec.begin(); iter < BandareaVec.end() - 1; )
	{
		int CURRENTtop = iter->top;
		int CURRENTbottom = iter->bottom;
		int NEXTtop = (iter + 1)->top;
		int NEXTbottom = (iter + 1)->bottom;
		int CURRENTleft = iter->left;
		int CURRENTright = iter->right;
		int NEXTleft = (iter + 1)->left;
		int NEXTright = (iter + 1)->right;

		int CURRENTvalue = iter->midmaxvalue;
		int NEXTvalue = (iter + 1)->midmaxvalue;

		if (NEXTtop - CURRENTbottom > 4)//距离远，不合并
			iter++;
		else//NEXTtop - CURRENTbottom <= 4，说明相邻
		{
			if (NEXTtop - CURRENTbottom <= 2)//非常接近，合并且取最大值
				if (CURRENTvalue - NEXTvalue <=
					THRESHOLDdistance * GetRate(min(CURRENTvalue, NEXTvalue) * 1.0 / MAXvalue))//差值不大，合并
				{
					iter = BandareaVec.erase(iter);
					iter = BandareaVec.erase(iter);

					Band MERGERDBandarea(CURRENTtop, NEXTbottom,
						max(CURRENTleft, NEXTleft), max(CURRENTright, NEXTright), max(CURRENTvalue, NEXTvalue));
					iter = BandareaVec.insert(iter, MERGERDBandarea);
				}
				else//差值较大	
					if (CURRENTvalue > NEXTvalue)//删除后一个
					{
						iter++;
						iter = BandareaVec.erase(iter);
						iter--;
					}
					else//删除前一个
						iter = BandareaVec.erase(iter);
			else//不非常接近，但差值不大则跳过
			{
				if (CURRENTvalue - NEXTvalue <=
					THRESHOLDdistance * GetRate(min(CURRENTvalue, NEXTvalue) * 1.0 / MAXvalue))//差值不大，跳过
					iter++;
				else//差值较大，进一步判断是否为同一区域	
				{
					int count = 0;
					for (int c = 0; c < 3; c++)
					{
						int STANDARDvalue = min(midMat.at<ushort>(CURRENTbottom, c), midMat.at<ushort>(NEXTtop, c));
						for (int r = CURRENTbottom + 1; r < NEXTtop; r++)
							if (STANDARDvalue - midMat.at<ushort>(r, c) - STANDARDvalue >=
								THRESHOLDdistance * GetRate(STANDARDvalue * 1.0 / MAXvalue))
								count++;
					}

					if (count >= 2 * (NEXTtop - CURRENTbottom - 1))//多数点满足，视为同一区域
						iter++;
					else
						if (CURRENTvalue > NEXTvalue)//删除后一个
						{
							iter++;
							iter = BandareaVec.erase(iter);
							iter--;
						}
						else//删除前一个
							iter = BandareaVec.erase(iter);
				}
			}
		}
	}
}
//两两检查并合并，实时修改BandareaVec

void PlanB::ReviseBandLength_Primary(const cv::Mat& source, vector<vector<Band>>& BandList,
	int THRESHOLDdistance, int MAXvalue)
{
	for (int i = 0; i < BandList.size(); i++)
		for (int j = 0; j < BandList[i].size(); j++)
		{
			Band& THISBand = BandList[i][j];
			int midrow = (THISBand.top + THISBand.bottom) / 2;
			int midcol = (THISBand.left + THISBand.right) / 2;
			int value = THISBand.midmaxvalue;

			int count;
			int NEWleft = THISBand.left;
			for (; NEWleft >= max(THISBand.left - 20, 0); NEWleft--)
			{
				count = 0;
				for (int r = midrow - 1; r <= midrow + 1; r++)
				{
					int CURRENTvalue = source.at<ushort>(r, NEWleft);
					if (value - CURRENTvalue < THRESHOLDdistance * GetRate(value * 1.0 / MAXvalue))
						count++;
				}
				if (count == 0)
					break;
			}

			int NEWright = THISBand.right;
			for (; NEWright <= min(THISBand.right + 20, source.cols - 1); NEWright++)
			{
				count = 0;
				for (int r = midrow - 1; r <= midrow + 1; r++)
				{
					int CURRENTvalue = source.at<ushort>(r, NEWright);
					if (value - CURRENTvalue < THRESHOLDdistance * GetRate(value * 1.0 / MAXvalue))
						count++;
				}
				if (count == 0)
					break;
			}

			THISBand.left = NEWleft;
			THISBand.right = NEWright;
		}
}
//左右边界修正，每条单独处理

void PlanB::ReviseBandLength_Advanced(const cv::Mat& source, vector<vector<Band>>& BandList,
	int THRESHOLDdistance, int MAXvalue)
{
	for (int i = 0; i < BandList.size(); i++)
		for (int j = 0; j < BandList[i].size(); j++)
		{
			Band& THISBand = BandList[i][j];

			int ORIGINtop = THISBand.top;
			int ORIGINbottom = THISBand.bottom;
			int ORIGINleft = THISBand.left;
			int ORIGINright = THISBand.right;
			int MIDrow = (ORIGINtop + ORIGINbottom) / 2;
			int STANDARDvalue = THISBand.midmaxvalue;

			//左边界
			pair<int, int> leftTESTheight =
				GetBandHeight_Accurate(source, MIDrow, ORIGINleft, THRESHOLDdistance, MAXvalue, STANDARDvalue);

			int REVISEDleft = ORIGINleft;
			for (int r = leftTESTheight.first; r <= leftTESTheight.second; r++)
			{
				int TESTleft =
					GetBandLength_Accurate(source, r, ORIGINleft, THRESHOLDdistance, MAXvalue, STANDARDvalue).first;
				REVISEDleft = min(REVISEDleft, TESTleft);
			}

			//右边界
			pair<int, int> rightTESTheight =
				GetBandHeight_Accurate(source, MIDrow, ORIGINright, THRESHOLDdistance, MAXvalue, STANDARDvalue);

			int REVISEDright = ORIGINright;
			for (int r = rightTESTheight.first; r <= rightTESTheight.second; r++)
			{
				int TESTright =
					GetBandLength_Accurate(source, r, ORIGINright, THRESHOLDdistance, MAXvalue, STANDARDvalue).second;
				REVISEDright = max(REVISEDright, TESTright);
			}

			THISBand.left = REVISEDleft;
			THISBand.right = REVISEDright;
		}
}
//左右边界修正，只针对斜条进行

void PlanB::GetBandLineOffsets(const cv::Mat& source, Band& THISBand,
	int THRESHOLDdistance, int MAXvalue, cv::Mat& colored_img)
{
	int top = THISBand.top;
	int bottom = THISBand.bottom;
	int left = THISBand.left;
	int right = THISBand.right;
	int MIDrow = (top + bottom) / 2;
	int MIDcol = (left + right) / 2;

	int value = THISBand.midmaxvalue;
	float slope = THISBand.slope;
	float COMPAREvalue = THRESHOLDdistance * GetRate(value * 1.0 / MAXvalue);

	int row = 0;
	if (value - source.at<ushort>(MIDrow, MIDcol) < COMPAREvalue)
		row = MIDrow;
	else
		if (value - source.at<ushort>(top, MIDcol) < COMPAREvalue)
			row = top;
		else
			row = bottom;

	pair<int, int> MIDheight = GetBandHeight_Rough(source, row, MIDcol, THRESHOLDdistance, MAXvalue, value);
	int MIDtop = MIDheight.first;
	int MIDbottom = MIDheight.second;

	//y = slope * x + offset <==> y = kx + b
	float top_line_offset = MIDtop - THISBand.slope * MIDcol;
	float bottom_line_offset = MIDbottom - THISBand.slope * MIDcol;
	THISBand.top_line_offset = top_line_offset;
	THISBand.bottom_line_offset = bottom_line_offset;

	int rightcol = (4 * right + left) / 5;
	int leftcol = (4 * left + right) / 5;
	cv::Point point1(rightcol, round(slope * rightcol + top_line_offset - 2));
	cv::Point point2(rightcol, round(slope * rightcol + bottom_line_offset + 2));
	cv::Point point3(leftcol, round(slope * leftcol + top_line_offset - 2));
	cv::Point point4(leftcol, round(slope * leftcol + bottom_line_offset + 2));
	cv::line(colored_img, point1, point3, cv::Scalar(0, 0, 255), 2);
	cv::line(colored_img, point2, point4, cv::Scalar(0, 0, 255), 2);
}
//获取左右边界线的截距，并绘制红色直线标记到Band上

void PlanB::DrawBandShape(const cv::Mat& colored_img, Band Bandarea, int thickness)
{
	int top = Bandarea.top - 1;
	int bottom = Bandarea.bottom + 1;
	int left = Bandarea.left - 1;
	int right = Bandarea.right + 1;//平移的画图原点(加1只为观察，实际无影响)
	//在24位图上画图
	cv::Point p1(left, top);
	cv::Point p2(right, bottom);
	//在图上画矩形框

	cv::rectangle(colored_img, p1, p2, cv::Scalar(0, 0, 255), thickness);
}
//24位图上画矩形框

bool PlanB::AnalyzeAllLane(const cv::Mat& source, vector<vector<Band>>& ALLBandareaVec,
	int THRESHOLDdistance, int STANDARDlength, int MAXvalue,
	int leftlimit, int rightlimit)
{
	cv::Mat leftclone = source.clone();
	cv::Mat rightclone = source.clone();//复制Mat图像

	pair<int, int> referenceleftANDright = GetReferenceLaneLength(source, STANDARDlength, THRESHOLDdistance);
	int REFERENCEleft = referenceleftANDright.first;
	int REFERENCEright = referenceleftANDright.second;
	int REFERENCEmidcol = (REFERENCEleft + REFERENCEleft) / 2; //此处算法有误，应为参考位置

	vector<Band> REFERENCEBandareaVec =
		AnalyzeSingleLane(source, THRESHOLDdistance, STANDARDlength, MAXvalue, REFERENCEleft, REFERENCEright);

	if (REFERENCEBandareaVec.size() == 0)
		return false;//若未找到任何车道，IsBackGroundArea中对应车道的vector为空

	vector<Band> SORTEDBandareaVec;
	if (REFERENCEBandareaVec.size() == 1)
		SORTEDBandareaVec = REFERENCEBandareaVec;
	else
	{
		SORTEDBandareaVec = SortBand(REFERENCEBandareaVec);

		int ALLBandtop = SORTEDBandareaVec.front().top;
		int ALLBandbottom = SORTEDBandareaVec.back().bottom;
		cv::Mat REFERENCEmidMat = GetMidMat(source, REFERENCEmidcol, ALLBandtop, ALLBandbottom);

		CheckAdjacentBands(REFERENCEmidMat, SORTEDBandareaVec, THRESHOLDdistance, MAXvalue);
	}

	ALLBandareaVec.push_back(SORTEDBandareaVec);

	//以参考原Mat图像分割，leftclone和rightclone分别为左右两侧的Mat图像
	for (int r = 0; r < leftclone.rows; r++)
		for (int c = REFERENCEleft - 3; c < leftclone.cols; c++)//-3为水平方向染色，确保实际识别时偏移小
			leftclone.at<ushort>(r, c) = 0;

	int leftFLAG = !IsBackGroundArea(leftclone, THRESHOLDdistance, leftlimit, REFERENCEleft - 1);
	if (leftFLAG)//左侧还有车道
		leftFLAG = AnalyzeAllLane(leftclone, ALLBandareaVec,
			THRESHOLDdistance, STANDARDlength, MAXvalue,
			leftlimit, REFERENCEleft - 1);
	//递归调用AnalyzeAllLane，若未识别到车道时leftFLAG转为false

	for (int r = 0; r < rightclone.rows; r++)
		for (int c = 0; c <= REFERENCEright + 3; c++)//+3为水平方向染色，确保实际识别时偏移小
			rightclone.at<ushort>(r, c) = 0;

	int rightFLAG = !IsBackGroundArea(rightclone, THRESHOLDdistance, REFERENCEright + 1, rightlimit);
	if (rightFLAG)//右侧还有车道
		rightFLAG = AnalyzeAllLane(rightclone, ALLBandareaVec,
			THRESHOLDdistance, STANDARDlength, MAXvalue,
			REFERENCEright + 1, rightlimit);
	//递归调用AnalyzeAllLane，若未识别到车道时rightFLAG转为false

	if (!leftFLAG && !rightFLAG)//左右均未识别到，返回false结束递归
		return false;
}
//车道分治识别

bool PlanB::IsIsolatedPoint(const cv::Mat& source, int row, int col, int THRESHOLDdistance, Direction direction)
{
	int count = 0;

	if (direction == Direction::Left_To_Right)
	{
		ushort basevalue = source.at<ushort>(row, col - 5);
		for (int r = row; r <= row + 1; r++)
			for (int c = col; c <= col + 4; c++)
			{
				if (source.at<ushort>(r, c) - basevalue > THRESHOLDdistance * 0.5)
					count++;
			}
		if (count >= 6)
			return false;

		count = 0;
		for (int r = row - 1; r <= row; r++)
			for (int c = col; c <= col + 4; c++)
			{
				if (source.at<ushort>(r, c) - basevalue > THRESHOLDdistance * 0.5)
					count++;
			}
		if (count >= 6)
			return false;
	}
	if (direction == Direction::Right_To_Left)
	{
		ushort basevalue = source.at<ushort>(row, col + 5);
		for (int r = row; r <= row + 1; r++)
			for (int c = col; c >= col - 4; c--)
			{
				if (source.at<ushort>(r, c) - basevalue > THRESHOLDdistance * 0.5)
					count++;
			}
		if (count >= 6)
			return false;

		count = 0;
		for (int r = row - 1; r <= row; r++)
			for (int c = col; c >= col - 4; c--)
			{
				if (source.at<ushort>(r, c) - basevalue > THRESHOLDdistance * 0.5)
					count++;
			}
		if (count >= 6)
			return false;
	}
	if (direction == Direction::Top_To_Bottom)
	{
		ushort basevalue = source.at<ushort>(row - 5, col);
		for (int r = row; r <= row + 1; r++)
			for (int c = col; c >= col - 4; c--)
			{
				if (source.at<ushort>(r, c) - basevalue > THRESHOLDdistance * 0.5)
					count++;
			}
		if (count >= 6)
			return false;

		count = 0;
		for (int r = row; r <= row + 1; r++)
			for (int c = col - 2; c <= col + 2; c++)
			{
				if (source.at<ushort>(r, c) - basevalue > THRESHOLDdistance * 0.5)
					count++;
			}
		if (count >= 6)
			return false;

		count = 0;
		for (int r = row; r <= row + 1; r++)
			for (int c = col; c <= col + 4; c++)
			{
				if (source.at<ushort>(r, c) - basevalue > THRESHOLDdistance * 0.5)
					count++;
			}
		if (count >= 6)
			return false;
	}
	if (direction == Direction::Bottom_To_Top)
	{
		ushort basevalue = source.at<ushort>(row + 5, col);
		for (int r = row - 1; r <= row; r++)
			for (int c = col; c >= col - 4; c--)
			{
				if (source.at<ushort>(r, c) - basevalue > THRESHOLDdistance * 0.5)
					count++;
			}
		if (count >= 6)
			return false;

		count = 0;
		for (int r = row - 1; r <= row; r++)
			for (int c = col - 2; c <= col + 2; c++)
			{
				if (source.at<ushort>(r, c) - basevalue > THRESHOLDdistance * 0.5)
					count++;
			}
		if (count >= 6)
			return false;

		count = 0;
		for (int r = row - 1; r <= row; r++)
			for (int c = col; c <= col + 4; c++)
			{
				if (source.at<ushort>(r, c) - basevalue > THRESHOLDdistance * 0.5)
					count++;
			}
		if (count >= 6)
			return false;
	}

	return true;
}
//GetAllLaneLength/GetAllLaneHeight中判断某点是否为孤立点，用于排除噪声

bool PlanB::IsBackGroundArea(const cv::Mat& source, int THRESHOLDdistance, int leftlimit, int rightlimit)
{
	if (rightlimit - 5 <= 5)
		return true;
	if ((source.rows - 6) - leftlimit <= 5)
		return true;
	//图像边缘太小时直接判断为背景

	if (rightlimit - leftlimit <= 3)
		return true;
	//区域太窄时直接判断为背景

	pair<int, int> topANDbottom = GetAllBandHeight(source, THRESHOLDdistance, leftlimit, rightlimit);
	int top = topANDbottom.first;
	int bottom = topANDbottom.second;

	if (top == source.rows - 5 || bottom == 5)
		return true;
	return false;
}
//判断一个预选区域是否实际为背景，即无效区域

bool PlanB::IsTilted(const cv::Mat& source, Band CURRENTBand, int THRESHOLDdistance, int MAXvalue)
{
	int top = CURRENTBand.top;
	int bottom = CURRENTBand.bottom;
	int midrow = (top + bottom) / 2;
	int left = CURRENTBand.left;
	int right = CURRENTBand.right;
	int value = CURRENTBand.midmaxvalue;

	int testcol1 = (3 * left + right) / 4;
	int testbottom1 = GetBandHeight_Rough(source, midrow, testcol1, THRESHOLDdistance, MAXvalue, value).second;

	int testcol2 = (left + 3 * right) / 4;
	int testbottom2 = GetBandHeight_Rough(source, midrow, testcol2, THRESHOLDdistance, MAXvalue, value).second;

	if (abs(testcol1 - testcol2) < 3)
		return false;
	else
		return true;
}
//判断一条是否倾斜

float PlanB::GetRate(float ratio)
{
	float rate = (15 * ratio + 1) / 8;
	return rate;
}
//GetBandLength/Height中使用，一个灰度值与Mat图像最大值的比例换算系数

float PlanB::GetSlope(const cv::Mat& source, Band CURRENTBand, int THRESHOLDdistance, int MAXvalue)
{
	int top = CURRENTBand.top;
	int bottom = CURRENTBand.bottom;
	int left = CURRENTBand.left;
	int right = CURRENTBand.right;
	int value = CURRENTBand.midmaxvalue;

	vector<int> TESTcols;
	for (int i = 2; i <= 8; i++)
		TESTcols.push_back((left * i + right * (8 - i)) / 8);//取5个点以确保准确值

	int midrow = (top + bottom) / 2;
	float COMPAREvalue = THRESHOLDdistance * GetRate(value * 1.0 / MAXvalue);
	vector<int> TESTrows;
	for (int i = 0; i < 5; i++)
		if (value - source.at<ushort>(midrow, TESTcols[i]) < COMPAREvalue)
			TESTrows.push_back(midrow);
		else
			if (value - source.at<ushort>(top, TESTcols[i]) < COMPAREvalue)
				TESTrows.push_back(top);
			else
				TESTrows.push_back(bottom);

	vector<int> TESTbottom;//上边界可能不够平滑，故只取下边界，因为下边界更易受倾斜影响
	for (int i = 0; i < 5; i++)
		TESTbottom.push_back(
			GetBandHeight_Accurate(source, TESTrows[i], TESTcols[i], THRESHOLDdistance, MAXvalue, value).second);

	int xy = 0;
	for (int i = 0; i < 5; i++)
		xy += TESTcols[i] * TESTbottom[i];

	int xx = 0;
	for (int i = 0; i < 5; i++)
		xx += TESTcols[i] * TESTcols[i];

	int nxy, nxx;
	int colsum = 0, bottomsum = 0;
	for (int i = 0; i < 5; i++)
	{
		colsum += TESTcols[i];
		bottomsum += TESTbottom[i];
	}
	nxy = colsum * bottomsum / 5;
	nxx = colsum * colsum / 5;

	float slope = (xy - nxy) * 1.0 / (xx - nxx);

	return slope;
}
//计算一条的下边界倾斜斜率

cv::Mat PlanB::GetMidMat(const cv::Mat& source, int midcol, int toplimit, int bottomlimit)
{
	cv::Mat newMat = cv::Mat::zeros(source.rows, 3, CV_16UC1);

	for (int r = toplimit; r <= bottomlimit; r++)
		for (int c = midcol - 1; c <= midcol + 1; c++)
			newMat.at<ushort>(r, c - midcol + 1) = source.at<ushort>(r, c);
	//注意newMat相当于在source中的对应平移(midcol - 1)列

	return newMat;
}
//取一个区域中的3列，以midcol为中心的3列组成的Mat图像

pair<int, int> PlanB::GetBandHeight_Rough(const cv::Mat& source, int row, int col,
	int THRESHOLDdistance, int MAXvalue, int STANDARDvalue)
{
	if (STANDARDvalue == 0)
		STANDARDvalue = source.at<ushort>(row, col);
	float ratio = STANDARDvalue * 1.0 / MAXvalue;
	float RATE = GetRate(ratio);

	if (source.cols == 3)//midMat图像
	{
		int topsum = 0, bottomsum = 0;

		int top;
		for (int c = 0; c < 3; c++)
		{
			for (top = row - 1; top >= 5; top--)
				if (STANDARDvalue - source.at<ushort>(top, c) > THRESHOLDdistance * RATE)
					break;
			topsum += top + 1;//逆向循环时top为终止值，+1
		}

		int bottom;
		for (int c = 0; c < 3; c++)
		{
			for (bottom = row + 1; bottom < source.rows - 5; bottom++)
				if (STANDARDvalue - source.at<ushort>(bottom, c) > THRESHOLDdistance * RATE)
					break;
			bottomsum += bottom - 1;//正向循环时bottom为终止值，-1
		}

		int topresult = topsum / 3;
		int bottomresult = bottomsum / 3;
		int height = bottomresult - topresult + 1;

		if (height < 3)//若高度较小，实际高度需扩展以避免遗漏
		{
			if (topresult == row && bottomresult == row)
			{
				topresult--;
				bottomresult++;
			}
			if (topresult == row && bottomresult > row)
				topresult--;
			if (topresult < row && bottomresult == row)
				bottomresult++;
		}

		if (topresult == -1)//防止越界
			topresult = 0;
		if (bottomresult == source.rows)//防止越界
			bottomresult = source.rows - 1;

		return pair<int, int>(topresult, bottomresult);
	}

	else//普通Mat图像
	{
		int topsum = 0, bottomsum = 0;

		vector<int> testcols;
		for (int offset = -2; offset <= 2; offset++)
			if (abs(source.at<ushort>(row, col + offset) - STANDARDvalue <= THRESHOLDdistance * RATE))
				testcols.push_back(col + offset);

		int top;
		for (int i = 0; i < testcols.size(); i++)
		{
			for (top = row - 1; top >= 5; top--)
				if (STANDARDvalue - source.at<ushort>(top, testcols[i]) > THRESHOLDdistance * RATE)
					break;
			topsum += top + 1;//逆向循环时top为终止值，+1
		}

		int bottom;
		for (int i = 0; i < testcols.size(); i++)
		{
			for (bottom = row + 1; bottom < source.rows - 5; bottom++)
				if (STANDARDvalue - source.at<ushort>(bottom, testcols[i]) > THRESHOLDdistance * RATE)
					break;
			bottomsum += bottom - 1;//正向循环时bottom为终止值，-1
		}

		int topresult = round(topsum / testcols.size());
		int bottomresult = round(bottomsum / testcols.size());
		int height = bottomresult - topresult + 1;

		if (height < 3)//若高度较小，实际高度需扩展以避免遗漏
		{
			if (topresult == row && bottomresult == row)
			{
				topresult--;
				bottomresult++;
			}
			if (topresult == row && bottomresult > row)
				topresult--;
			if (topresult < row && bottomresult == row)
				bottomresult++;
		}

		if (topresult == -1)//防止越界
			topresult = 0;
		if (bottomresult == source.rows)//防止越界
			bottomresult = source.rows - 1;

		return pair<int, int>(topresult, bottomresult);
	}
}
//从一个点出发，获取上下边界，取的是上下边界的平均值

pair<int, int> PlanB::GetBandLength_Rough(const cv::Mat& source, int row, int col,
	int THRESHOLDdistance, int MAXvalue, int STANDARDvalue)
{
	if (STANDARDvalue == 0)
		STANDARDvalue = source.at<ushort>(row, col);
	float ratio = STANDARDvalue * 1.0 / MAXvalue;
	float RATE = GetRate(ratio);

	vector<int> testrows;
	for (int offset = -2; offset <= 2; offset++)
		if (STANDARDvalue - source.at<ushort>(row + offset, col) <= THRESHOLDdistance * RATE)
			testrows.push_back(row + offset);

	vector<int> leftVec, rightVec;
	for (int i = 0; i < testrows.size(); i++)
	{
		int THISvalue = source.at<ushort>(testrows[i], col);
		float THISrate = GetRate(THISvalue * 1.0 / MAXvalue);

		int left = col - 1;
		for (; left >= 5; left--)
			if (THISvalue - source.at<ushort>(testrows[i], left) > THRESHOLDdistance * THISrate)
				break;
		leftVec.push_back(left + 1);

		int right = col + 1;
		for (; right <= source.rows - 6; right++)
			if (THISvalue - source.at<ushort>(testrows[i], right) > THRESHOLDdistance * THISrate)
				break;
		rightVec.push_back(right - 1);
	}

	int RESULTleft = 65535, RESULTright = 0;
	for (int j = 0; j < testrows.size(); j++)
	{
		if (leftVec[j] < RESULTleft)
			RESULTleft = leftVec[j];
		if (rightVec[j] > RESULTright)
			RESULTright = rightVec[j];
	}

	return pair<int, int>(RESULTleft, RESULTright);
}
//从一个点出发，获取左右边界，取的是左右边界的极值

pair<int, int> PlanB::GetBandHeight_Accurate(const cv::Mat& source, int row, int col,
	int THRESHOLDdistance, int MAXvalue, int STANDARDvalue)
{
	int COMPAREvalue = THRESHOLDdistance * GetRate(STANDARDvalue * 1.0 / MAXvalue);

	int top = row;
	for (; top >= 2; top--)
		if (STANDARDvalue - source.at<ushort>(top - 1, col) > COMPAREvalue &&
			STANDARDvalue - source.at<ushort>(top - 2, col) > COMPAREvalue)
			break;

	int bottom = row;
	for (; bottom < source.rows - 2; bottom++)
		if (STANDARDvalue - source.at<ushort>(bottom + 1, col) > COMPAREvalue &&
			STANDARDvalue - source.at<ushort>(bottom + 2, col) > COMPAREvalue)
			break;

	return pair<int, int>(top, bottom);
}
//从一个点出发，获取上下边界，只取的是上下边界的确切值

pair<int, int> PlanB::GetBandLength_Accurate(const cv::Mat& source, int row, int col,
	int THRESHOLDdistance, int MAXvalue, int STANDARDvalue)
{
	int COMPAREvalue = THRESHOLDdistance * GetRate(STANDARDvalue * 1.0 / MAXvalue);

	int left = col;
	for (; left >= 2; left--)
		if (STANDARDvalue - source.at<ushort>(row, left - 1) > COMPAREvalue &&
			STANDARDvalue - source.at<ushort>(row, left - 2) > COMPAREvalue)
			break;

	int right = col;
	for (; right < source.cols - 2; right++)
		if (STANDARDvalue - source.at<ushort>(row, right + 1) > COMPAREvalue &&
			STANDARDvalue - source.at<ushort>(row, right + 2) > COMPAREvalue)
			break;

	return pair<int, int>(left, right);
}
//从一个点出发，获取左右边界，只取的是左右边界的确切值

pair<int, int> PlanB::GetReferenceLaneLength(const cv::Mat& source, int STANDARDlength, int THRESHOLDdistance)
{
	int count = 0;
	vector<int> testlengthVec;
	vector<pair<int, int>> testleftAndrightVec;

	int MAXvalue = 0;
	cv::Mat clone = source.clone();

	for (int r = 0; r < 5; r++)
		for (int c = 0; c < clone.cols; c++)
			clone.at<ushort>(r, c) = 0;
	for (int r = clone.rows - 5; r < clone.rows; r++)
		for (int c = 0; c < clone.cols; c++)
			clone.at<ushort>(r, c) = 0;
	for (int r = 0; r < clone.rows; r++)
		for (int c = 0; c < 5; c++)
			clone.at<ushort>(r, c) = 0;
	for (int r = 0; r < clone.rows; r++)
		for (int c = clone.cols - 5; c < clone.cols; c++)
			clone.at<ushort>(r, c) = 0;
	//置零以防止边缘定位原Mat图像展开

	int flag = 0;
	while (1)
	{
		double maxv;
		double* maxptr = &maxv;
		cv::Point maxpoint;
		cv::Point* maxpointptr = &maxpoint;

		cv::minMaxLoc(clone, nullptr, maxptr, nullptr, maxpointptr);

		int row = maxpointptr->y;
		int col = maxpointptr->x;
		int value = *maxptr;

		if (value < THRESHOLDdistance)
		{
			flag = 1;
			break;
		}

		if (count == 0 && MAXvalue == 0)
			MAXvalue = value;

		pair<int, int> topAndbottom = PlanB::GetBandHeight_Rough(clone, row, col, THRESHOLDdistance, MAXvalue);
		pair<int, int> leftAndright = General::GetBandLength(clone, row, col, source.at<ushort>(row, col), 5, clone.cols - 6);
		//此处PlanA思路，但GetBandLength为PlanB思路，混合使用
		int top = topAndbottom.first;
		int bottom = topAndbottom.second;
		int left = leftAndright.first;
		int right = leftAndright.second;

		int testlength = right - left + 1;

		if (testlength <= STANDARDlength + 3 && testlength >= STANDARDlength - 3)
			return leftAndright;

		if (testlength <= STANDARDlength / 2 || testlength >= STANDARDlength * 3 / 2)
			count--;//比第二条车道短
		else
		{
			testlengthVec.push_back(testlength);
			testleftAndrightVec.push_back(leftAndright);
		}//STANDARDlength相近的第二条车道时存入vector

		for (int r = top; r <= bottom; r++)
			for (int c = left; c <= right; c++)
				clone.at<ushort>(r, c) = 0;
		//已找到的不要求其灰度值恒转为0，以防一味追求最大值而忽略次大值

		count++;
		if (count == 5)
		{
			flag = 1;
			break;
		}
	}

	if (flag)
	{
		int MINdistance = 65535;
		pair<int, int> MINleftAndright;
		for (int i = 0; i < testlengthVec.size(); i++)
			if (abs(testlengthVec[i] - STANDARDlength) < MINdistance)
				MINleftAndright = testleftAndrightVec[i];

		return MINleftAndright;
	}
}
//首次获取参考左右边界，参考车道为最接近标准长度的车道

pair<int, int> PlanB::GetAllLaneLength(const cv::Mat& source, int THRESHOLDdistance)
{
	int leftBORDER = 5;
	int leftFLAG = 0;
	for (; leftBORDER < source.cols - 5; leftBORDER++)
	{
		for (int r = 0 + 1; r < source.rows - 1; r++)//+1为确保IsIsolatedPoint中Mat图像值不越界
		{
			int distance = source.at<ushort>(r, leftBORDER) - source.at<ushort>(r, leftBORDER - 5);
			if (distance > THRESHOLDdistance)
				if (!IsIsolatedPoint(source, r, leftBORDER, THRESHOLDdistance, Direction::Left_To_Right))
				{
					leftFLAG = 1;
					break;
				}
		}
		if (leftFLAG)
			break;
	}

	int rightBORDER = source.cols - 6;
	int rightFLAG = 0;
	for (; rightBORDER >= 5; rightBORDER--)
	{
		for (int r = 0 + 1; r < source.rows - 1; r++)//+1为确保IsIsolatedPoint中Mat图像值不越界
		{
			int distance = source.at<ushort>(r, rightBORDER) - source.at<ushort>(r, rightBORDER + 5);
			if (distance > THRESHOLDdistance)
				if (!IsIsolatedPoint(source, r, rightBORDER, THRESHOLDdistance, Direction::Right_To_Left))
				{
					rightFLAG = 1;
					break;
				}
		}
		if (rightFLAG)
			break;
	}

	return pair<int, int>(leftBORDER, rightBORDER);
}
//获取整个图像左边界和右边界

pair<int, int> PlanB::GetAllBandHeight(const cv::Mat& source, int THRESHOLDdistance, int leftlimit, int rightlimit)
{
	int topBORDER = 5;
	int topFLAG = 0;
	for (; topBORDER < source.rows - 5; topBORDER++)
	{
		for (int c = leftlimit; c <= rightlimit; c++)//leftlimit和rightlimit由GetAllLaneLength确定边界
		{
			int distance = source.at<ushort>(topBORDER, c) - source.at<ushort>(topBORDER - 5, c);
			if (distance > THRESHOLDdistance)
				if (!IsIsolatedPoint(source, topBORDER, c, THRESHOLDdistance, Direction::Top_To_Bottom))
				{
					topFLAG = 1;
					break;
				}
		}
		if (topFLAG == 1)
			break;
	}

	int bottomBORDER = source.rows - 6;
	int bottomFLAG = 0;
	for (; bottomBORDER > 5; bottomBORDER--)
	{
		for (int c = leftlimit; c <= rightlimit; c++)//leftlimit和rightlimit由GetAllLaneLength确定边界
		{
			int distance = source.at<ushort>(bottomBORDER, c) - source.at<ushort>(bottomBORDER + 5, c);
			if (distance > THRESHOLDdistance)
				if (!IsIsolatedPoint(source, bottomBORDER, c, THRESHOLDdistance, Direction::Bottom_To_Top))
				{
					bottomFLAG = 1;
					break;
				}
		}
		if (bottomFLAG == 1)
			break;
	}

	return pair<int, int>(topBORDER, bottomBORDER);
}
//获取整个图像上边界和下边界

pair<int, int> PlanB::FindBackgroundOffset(const cv::Mat& midMat, int THRESHOLDdistance, int MAXvalue,
	int Bandtop, int Bandbottom, int toplimit, int bottomlimit)
{
	int topoffset, bottomoffset;

	for (topoffset = 0; topoffset <= Bandtop - toplimit; topoffset++)
	{
		if (topoffset > 0)
		{
			if (!midMat.at<ushort>(Bandtop - topoffset + 1, 1) && midMat.at<ushort>(Bandtop - topoffset, 1))
				break;

			int count = 0;
			for (int i = 0; i < 3; i++)
				if (midMat.at<ushort>(Bandtop - topoffset, i) - midMat.at<ushort>(Bandtop - topoffset + 1, i)
					>=
					THRESHOLDdistance * GetRate(midMat.at<ushort>(Bandtop - topoffset + 1, i) * 1.0 / MAXvalue))
					count++;
			if (count >= 2)
				break;

			if (Bandbottom - (Bandtop - topoffset) >= 3)
			{
				count = 0;
				for (int i = 0; i < 3; i++)
					if (midMat.at<ushort>(Bandtop - topoffset, i) - midMat.at<ushort>(Bandtop - topoffset + 3, i)
						>=
						THRESHOLDdistance * GetRate(midMat.at<ushort>(Bandtop - topoffset + 3, i) * 1.0 / MAXvalue))
						count++;
				if (count >= 2)
					break;
			}

			if (Bandbottom - (Bandtop - topoffset) >= 5)
			{
				count = 0;
				for (int i = 0; i < 3; i++)
					if (midMat.at<ushort>(Bandtop - topoffset, i) - midMat.at<ushort>(Bandtop - topoffset + 5, i)
						>=
						THRESHOLDdistance * GetRate(midMat.at<ushort>(Bandtop - topoffset + 5, i) * 1.0 / MAXvalue))
						count++;
				if (count >= 2)
					break;
			}

		}
	}

	for (bottomoffset = 0; bottomoffset <= bottomlimit - Bandbottom; bottomoffset++)
	{
		if (bottomoffset > 0)
		{
			if (!midMat.at<ushort>(Bandbottom + bottomoffset - 1, 1) && midMat.at<ushort>(Bandbottom + bottomoffset, 1))
				break;

			int count = 0;
			for (int i = 0; i < 3; i++)
				if (midMat.at<ushort>(Bandbottom + bottomoffset, i) - midMat.at<ushort>(Bandbottom + bottomoffset - 1, i)
					>=
					THRESHOLDdistance * GetRate(midMat.at<ushort>(Bandbottom + bottomoffset - 1, i) * 1.0 / MAXvalue))
					count++;
			if (count >= 2)
				break;

			if ((Bandbottom + bottomoffset) - Bandtop >= 3)
			{
				count = 0;
				for (int i = 0; i < 3; i++)
					if (midMat.at<ushort>(Bandbottom + bottomoffset, i) - midMat.at<ushort>(Bandbottom + bottomoffset - 3, i)
						>=
						THRESHOLDdistance * GetRate(midMat.at<ushort>(Bandbottom + bottomoffset - 3, i) * 1.0 / MAXvalue))
						count++;
				if (count >= 2)
					break;
			}

			if ((Bandbottom + bottomoffset) - Bandtop >= 5)
			{
				count = 0;
				for (int i = 0; i < 3; i++)
					if (midMat.at<ushort>(Bandbottom + bottomoffset, i) - midMat.at<ushort>(Bandbottom + bottomoffset - 5, i)
						>=
						THRESHOLDdistance * GetRate(midMat.at<ushort>(Bandbottom + bottomoffset - 5, i) * 1.0 / MAXvalue))
						count++;
				if (count >= 2)
					break;
			}
		}
	}

	return pair<int, int>(topoffset, bottomoffset);
}
//在已找到条带基础上寻找上下背景的偏移量

Band PlanB::FindBandByMaxPoint(const cv::Mat& source, const cv::Mat& midMat,
	int THRESHOLDdistance, int STANDARDlength, int MAXvalue,
	int leftlimit, int rightlimit, int toplimit, int bottomlimit)
{
	int midcol = (leftlimit + rightlimit) / 2;

	PointInfoTYPE MAXpointinfo = GetMaxPointInfo(midMat);

	int STANDARDvalue = get<1>(MAXpointinfo);
	int MAXcol = get<0>(MAXpointinfo).first + midcol - 1;//注意GetMaxPointInfo的返回图为子图，故此处MAXcol需映射回原图
	int MAXrow = get<0>(MAXpointinfo).second;

	if (STANDARDvalue < MAXvalue / 8)//识别值已经小于一定阈值，说明剩余全为背景，停止
		return Band(0, 0, 0, 0, 0);

	pair<int, int> BandtopANDbottom =
		PlanB::GetBandHeight_Rough(midMat, MAXrow, 1, THRESHOLDdistance, MAXvalue);
	pair<int, int> BandleftANDright =
		General::GetBandLength(source, MAXrow, MAXcol, source.at<ushort>(MAXrow, MAXcol), leftlimit, rightlimit);

	int Bandleft = BandleftANDright.first;
	int Bandright = BandleftANDright.second;
	int Bandtop = BandtopANDbottom.first;
	int Bandbottom = BandtopANDbottom.second;

	int length = Bandright - Bandleft + 1;
	int height = Bandbottom - Bandtop + 1;
	int area = length * height;
	if (area <= (STANDARDlength * 0.75) * 2 && area <= (STANDARDlength - 3) * 2)
		return Band(Bandtop, Bandbottom, -1, -1, 0);//识别太小，判断为不要
	if (length <= STANDARDlength / 2 || length >= STANDARDlength * 3 / 2)
		return Band(Bandtop, Bandbottom, -2, -2, 0);//识别太长或太短，判断为不要

	return Band(Bandtop, Bandbottom, Bandleft, Bandright, STANDARDvalue);
}
//在给定区域内，找到第一条

vector<Band> PlanB::SortBand(vector<Band>& BandareaVec)
//此函数会改变BandareaVec本身，慎用
{
	vector<Band> SORTEDBandareaVec;

	if (BandareaVec.size() > 1)//只有一元素的BandareaVec无需排序
	{
		vector<int> topVec;
		for (int i = 0; i < BandareaVec.size(); i++)
			topVec.push_back(BandareaVec[i].top);

		int index = 0;
		int topMIN = 65535;
		while (topVec.size() > 1)
		{
			index = 0;
			topMIN = 65535;
			for (int i = 0; i < topVec.size(); i++)
				if (topMIN > topVec[i])
				{
					topMIN = topVec[i];
					index = i;
				}

			SORTEDBandareaVec.push_back(BandareaVec[index]);
			BandareaVec.erase(BandareaVec.begin() + index);
			topVec.erase(topVec.begin() + index);
		}
		SORTEDBandareaVec.push_back(BandareaVec.back());//此时BandareaVec只剩一个元素
	}

	return SORTEDBandareaVec;
}
//对BandareaVec按从上到下排序

vector<Band> PlanB::AnalyzeSingleLane(const cv::Mat& source,
	int THRESHOLDdistance, int STANDARDlength, int MAXvalue,
	int leftlimit, int rightlimit)
{
	pair<int, int> topANDbottom = GetAllBandHeight(source, THRESHOLDdistance, leftlimit, rightlimit);
	int toplimit = topANDbottom.first;
	int bottomlimit = topANDbottom.second;

	cv::Mat midMatclone = GetMidMat(source, (leftlimit + rightlimit) / 2, toplimit, bottomlimit);//取中间3列的Mat图像

	vector<Band> BandareaVec;
	while (1)
	{
		Band TRUEBandarea = FindBandByMaxPoint(source, midMatclone,
			THRESHOLDdistance, STANDARDlength, MAXvalue,
			leftlimit, rightlimit, toplimit, bottomlimit);

		int Bandtop = TRUEBandarea.top;
		int Bandbottom = TRUEBandarea.bottom;
		int Bandleft = TRUEBandarea.left;
		int Bandright = TRUEBandarea.right;

		if (Bandtop == 0 && Bandbottom == 0 && Bandleft == 0 && Bandright == 0)
			break;//此时区域元素全为0，FindBandByMaxPoint返回全0

		if ((Bandleft == -1 && Bandright == -1) || (Bandleft == -2 && Bandright == -2))//太小或长短异常之条
		{
			for (int r = Bandtop; r <= Bandbottom; r++)
				for (int c = 0; c < 3; c++)
					midMatclone.at<ushort>(r, c) =
					(midMatclone.at<ushort>(Bandtop - 1, c) + midMatclone.at<ushort>(Bandbottom + 1, c)) / 2;
			continue;
		}

		BandareaVec.push_back(TRUEBandarea);

		pair<int, int> backgroundoffsets =
			FindBackgroundOffset(midMatclone, THRESHOLDdistance, MAXvalue, Bandtop, Bandbottom, toplimit, bottomlimit);
		int topBGoffset = backgroundoffsets.first;
		int bottomBGoffset = backgroundoffsets.second;

		int totaltop = Bandtop - topBGoffset;
		int totalbottom = Bandbottom + bottomBGoffset;
		if (totaltop == toplimit && bottomlimit == totalbottom)
			break;

		ClearRowArea(midMatclone, totaltop, totalbottom);
	}

	return BandareaVec;
}
//在给定区域内循环查找条带

vector<vector<Band>> PlanB::SortLane(vector<vector<Band>>& BandList)
{
	vector<vector<Band>> SORTEDBandList;

	vector<int> leftVec;
	for (int i = 0; i < BandList.size(); i++)
		leftVec.push_back(BandList[i].front().left);

	int index = 0;
	int leftMIN = 65535;
	while (leftVec.size() > 1)
	{
		index = 0;
		leftMIN = 65535;
		for (int i = 0; i < leftVec.size(); i++)
			if (leftMIN > leftVec[i])
			{
				leftMIN = leftVec[i];
				index = i;
			}

		SORTEDBandList.push_back(BandList[index]);
		BandList.erase(BandList.begin() + index);
		leftVec.erase(leftVec.begin() + index);
	}
	SORTEDBandList.push_back(BandList.front());//此时BandList只剩一个车道
	return SORTEDBandList;
}
//为多个车道按从左到右排序

vector<vector<Band>> PlanB::TransferToOrigin(const vector<vector<Band>>& ProcessedList)
{
	vector<vector<Band>> OriginList;

	for (int i = 0; i < ProcessedList.size(); i++)
	{
		vector<Band> OriginVec;

		for (int j = 0; j < ProcessedList[i].size(); j++)
		{
			int processedtop = ProcessedList[i][j].top;
			int processedbottom = ProcessedList[i][j].bottom;
			int processedleft = ProcessedList[i][j].left;
			int processedright = ProcessedList[i][j].right;
			int processedvalue = ProcessedList[i][j].midmaxvalue;

			int origintop = (processedtop - 5) * 3 + 1;
			int originbottom = (processedbottom - 5) * 3 + 1;
			int originleft = (processedleft - 5) * 5 + 2;
			int originright = (processedright - 5) * 5 + 2;
			int originvalue = processedvalue;

			OriginVec.push_back(Band(origintop, originbottom, originleft, originright, originvalue));
		}

		OriginList.push_back(OriginVec);
	}

	return OriginList;
}
//3X5缩放图中获得的信息转换回原图信息