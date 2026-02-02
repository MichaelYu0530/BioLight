#include "GeneralHead.h"
#include "ClassHead.h"
#include "PlanBHead.h"
#include "PlanCHead.h"

#include <math.h>
#include <iostream>

using namespace std;
using namespace PlanC;
using namespace General;

typedef tuple<pair<int, int>, int> PointInfoTYPE;
// tuple(坐标(row, col), 灰度值)
typedef vector<vector<int>> BandSummitsTYPE;
// vector(峰值点行坐标列表, 对应灰度差值列表)
typedef tuple<int, int, int, BandSummitsTYPE> LaneInfoTYPE;
// tuple(左边界, 右边界, 条带中心列, 峰值信息)
typedef tuple<int, int, int, int> BandAreaTYPE;
// tuple(上边界, 下边界, 左边界, 右边界)

void PlanC::ClearRowArea(cv::Mat& source, int top, int bottom)
{
	for (int r = top; r <= bottom; r++)
		for (int c = 0; c < source.cols; c++)
			source.at<ushort>(r, c) = 0;
}
// 将指定行区间[top, bottom]内的所有像素置为0（用于抹除已处理区域）

void PlanC::CheckAdjacentBands(const cv::Mat& midMat, vector<Band>& BandareaVec,
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

		if (NEXTtop - CURRENTbottom > 12) // 两区域垂直间距过大，视为独立条带
			iter++;
		else // NEXTtop - CURRENTbottom <= 12，需进一步判断是否合并
		{
			if (NEXTtop - CURRENTbottom <= 6) // 间距很小，优先考虑合并
				if (abs(CURRENTvalue - NEXTvalue) <=
					THRESHOLDdistance * GetRate(min(CURRENTvalue, NEXTvalue) * 1.0 / MAXvalue)) // 灰度差异在容差范围内
				{
					// 合并两个相邻条带
					iter = BandareaVec.erase(iter);
					iter = BandareaVec.erase(iter);

					Band MERGERDBandarea(CURRENTtop, NEXTbottom,
						min(CURRENTleft, NEXTleft), max(CURRENTright, NEXTright), max(CURRENTvalue, NEXTvalue));
					iter = BandareaVec.insert(iter, MERGERDBandarea);
				}
				else // 灰度差异过大，保留更强的一个
					if (CURRENTvalue > NEXTvalue) // 保留当前条带
						BandareaVec.erase(iter + 1);
					else // 保留下一个条带
						iter = BandareaVec.erase(iter);
			else // 间距中等（7～12行），需检查中间是否存在连续显著响应
				if (abs(CURRENTvalue - NEXTvalue) <=
					THRESHOLDdistance * GetRate(min(CURRENTvalue, NEXTvalue) * 1.0 / MAXvalue))
					iter++; // 差异小，暂不合并，继续检查
				else
				{
					// 检查中间区域是否存在足够多的显著像素（非孤立噪声）
					int count = 0;
					for (int c = 0; c < 5; c++) // 在midMat的5列范围内检查
					{
						int STANDARDvalue = min(midMat.at<ushort>(CURRENTbottom, c), midMat.at<ushort>(NEXTtop, c));
						for (int r = CURRENTbottom + 1; r < NEXTtop; r++)
							if (abs(STANDARDvalue - midMat.at<ushort>(r, c)) >
								THRESHOLDdistance * GetRate(STANDARDvalue * 1.0 / MAXvalue))
								count++;
					}

					// 若显著像素数量超过阈值（占中间区域的1/3以上），则视为连续条带，保留两者
					if (count >= 5 * (NEXTtop - CURRENTbottom - 1) / 3)
						iter++;
					else
						if (CURRENTvalue > NEXTvalue) // 否则保留更强者
							BandareaVec.erase(iter + 1);
						else
							iter = BandareaVec.erase(iter);
				}
		}
	}
}
// 合并或剔除相邻的垂直条带区域：基于间距、灰度相似性和中间区域连续性进行决策

void PlanC::DrawRect(const cv::Mat& colored_img, Band Bandarea, int thickness)
{
	int top = Bandarea.top - 1;
	int bottom = Bandarea.bottom + 1;
	int left = Bandarea.left - 1;
	int right = Bandarea.right + 1; // 绘图时略微扩大边界（±1仅为视觉效果，实际区域无偏移）
	// 直接在传入的Mat图像上绘制矩形框
	cv::Point p1(left, top);
	cv::Point p2(right, bottom);
	// 构造矩形对角点

	cv::rectangle(colored_img, p1, p2, cv::Scalar(0, 0, 255), thickness);
}
// 在24位彩色图像上绘制指定条带区域的矩形框（红色）

bool PlanC::AnalyzeAllLane(const cv::Mat& source, vector<vector<Band>>& ALLBandareaVec,
	int THRESHOLDdistance, int STANDARDlength, int MAXvalue,
	int leftlimit, int rightlimit)
{
	cv::Mat leftclone = source.clone();
	cv::Mat rightclone = source.clone(); // 创建左右两份副本，用于递归分割

	pair<int, int> referenceleftANDright = GetReferenceLaneLength(source, STANDARDlength, THRESHOLDdistance);
	int REFERENCEleft = referenceleftANDright.first;
	int REFERENCEright = referenceleftANDright.second;
	int REFERENCEmidcol = (REFERENCEleft + REFERENCEright) / 2; // 计算参考条带的中心列（原代码此处有笔误，已修正）

	vector<Band> REFERENCEBandareaVec =
		AnalyzeSingleLane(source, THRESHOLDdistance, STANDARDlength, MAXvalue, REFERENCEleft, REFERENCEright);

	if (REFERENCEBandareaVec.size() == 0)
		return false; // 未检测到有效参考条带，可能整个区域为背景

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

	// 清除参考条带右侧区域，准备分析左侧
	for (int r = 0; r < leftclone.rows; r++)
		for (int c = REFERENCEleft; c < leftclone.cols; c++)
			leftclone.at<ushort>(r, c) = 0;

	int leftFLAG = !IsBackGroundArea(leftclone, THRESHOLDdistance, leftlimit, REFERENCEleft - 1);
	if (leftFLAG) // 左侧存在非背景区域
		leftFLAG = AnalyzeAllLane(leftclone, ALLBandareaVec,
			THRESHOLDdistance, STANDARDlength, MAXvalue,
			leftlimit, REFERENCEleft - 1);
	// 递归分析左侧区域；若递归失败，leftFLAG将为false

	// 清除参考条带左侧区域，准备分析右侧
	for (int r = 0; r < rightclone.rows; r++)
		for (int c = 0; c <= REFERENCEright; c++)
			rightclone.at<ushort>(r, c) = 0;

	int rightFLAG = !IsBackGroundArea(rightclone, THRESHOLDdistance, REFERENCEright + 1, rightlimit);
	if (rightFLAG) // 右侧存在非背景区域
		rightFLAG = AnalyzeAllLane(rightclone, ALLBandareaVec,
			THRESHOLDdistance, STANDARDlength, MAXvalue,
			REFERENCEright + 1, rightlimit);
	// 递归分析右侧区域；若递归失败，rightFLAG将为false

	if (!leftFLAG && !rightFLAG) // 左右均无有效条带，但当前参考条带有效，整体返回true
		return true; // 注意：此处原注释有误，应返回true表示当前层级成功
	else
		return true; // 实际上只要找到参考条带就应返回true
}
// 递归地分析整幅图像中的所有条带区域：先找参考条带，再分别递归处理左右两侧

bool PlanC::IsIsolatedPoint(const cv::Mat& source, int row, int col, int THRESHOLDdistance, Direction direction)
{
	int count;
	int MAXvalue = get<1>(GetMaxPointInfo(source));

	if (direction == Direction::Left_To_Right)
	{
		ushort BASEvalue = source.at<ushort>(row, col - 5);

		count = 0;
		for (int r = row - 2; r <= row; r++)
			for (int c = col; c <= col + 4; c++)
			{
				if (source.at<ushort>(r, c) - BASEvalue > THRESHOLDdistance * GetRate(BASEvalue * 1.0 / MAXvalue))
					count++;
			}
		if (count >= 10)
			return false;

		count = 0;
		for (int r = row - 1; r <= row + 1; r++)
			for (int c = col; c <= col + 4; c++)
			{
				if (source.at<ushort>(r, c) - BASEvalue > THRESHOLDdistance * GetRate(BASEvalue * 1.0 / MAXvalue))
					count++;
			}
		if (count >= 10)
			return false;

		count = 0;
		for (int r = row; r <= row + 2; r++)
			for (int c = col; c <= col + 4; c++)
			{
				if (source.at<ushort>(r, c) - BASEvalue > THRESHOLDdistance * GetRate(BASEvalue * 1.0 / MAXvalue))
					count++;
			}
		if (count >= 10)
			return false;
	}

	if (direction == Direction::Right_To_Left)
	{
		ushort BASEvalue = source.at<ushort>(row, col + 5);

		count = 0;
		for (int r = row - 2; r <= row; r++)
			for (int c = col; c >= col - 4; c--)
			{
				if (source.at<ushort>(r, c) - BASEvalue > THRESHOLDdistance * GetRate(BASEvalue * 1.0 / MAXvalue))
					count++;
			}
		if (count >= 10)
			return false;

		count = 0;
		for (int r = row - 1; r <= row + 1; r++)
			for (int c = col; c >= col - 4; c--)
			{
				if (source.at<ushort>(r, c) - BASEvalue > THRESHOLDdistance * GetRate(BASEvalue * 1.0 / MAXvalue))
					count++;
			}
		if (count >= 10)
			return false;

		count = 0;
		for (int r = row; r <= row + 2; r++)
			for (int c = col; c >= col - 4; c--)
			{
				if (source.at<ushort>(r, c) - BASEvalue > THRESHOLDdistance * GetRate(BASEvalue * 1.0 / MAXvalue))
					count++;
			}
		if (count >= 10)
			return false;
	}

	if (direction == Direction::Top_To_Bottom)
	{
		ushort BASEvalue = source.at<ushort>(row - 5, col);

		count = 0;
		for (int r = row - 2; r <= row; r++)
			for (int c = col; c >= col - 4; c--)
			{
				if (source.at<ushort>(r, c) - BASEvalue > THRESHOLDdistance * GetRate(BASEvalue * 1.0 / MAXvalue))
					count++;
			}
		if (count >= 10)
			return false;

		count = 0;
		for (int r = row - 1; r <= row + 1; r++)
			for (int c = col - 2; c <= col + 2; c++)
			{
				if (source.at<ushort>(r, c) - BASEvalue > THRESHOLDdistance * GetRate(BASEvalue * 1.0 / MAXvalue))
					count++;
			}
		if (count >= 10)
			return false;

		count = 0;
		for (int r = row; r <= row + 2; r++)
			for (int c = col; c <= col + 4; c++)
			{
				if (source.at<ushort>(r, c) - BASEvalue > THRESHOLDdistance * GetRate(BASEvalue * 1.0 / MAXvalue))
					count++;
			}
		if (count >= 10)
			return false;
	}

	if (direction == Direction::Bottom_To_Top)
	{
		ushort BASEvalue = source.at<ushort>(row + 5, col);

		count = 0;
		for (int r = row - 2; r <= row; r++)
			for (int c = col; c >= col - 4; c--)
			{
				if (source.at<ushort>(r, c) - BASEvalue > THRESHOLDdistance * GetRate(BASEvalue * 1.0 / MAXvalue))
					count++;
			}
		if (count >= 10)
			return false;

		count = 0;
		for (int r = row - 1; r <= row + 1; r++)
			for (int c = col - 2; c <= col + 2; c++)
			{
				if (source.at<ushort>(r, c) - BASEvalue > THRESHOLDdistance * GetRate(BASEvalue * 1.0 / MAXvalue))
					count++;
			}
		if (count >= 10)
			return false;

		count = 0;
		for (int r = row - 2; r <= row; r++)
			for (int c = col; c <= col + 4; c++)
			{
				if (source.at<ushort>(r, c) - BASEvalue > THRESHOLDdistance * GetRate(BASEvalue * 1.0 / MAXvalue))
					count++;
			}
		if (count >= 10)
			return false;
	}

	return true;
}
// 判断某点是否为孤立噪声点（用于GetAllLaneLength/GetAllLaneHeight中排除虚假边缘）
// 通过在指定方向的邻域内统计显著响应像素数量，若不足则视为孤立点

bool PlanC::IsBackGroundArea(const cv::Mat& source, int THRESHOLDdistance, int leftlimit, int rightlimit)
{
	if (rightlimit - leftlimit <= 10)
		return true;
	// 区域宽度太小，视为无效

	pair<int, int> topANDbottom = GetAllBandHeight(source, THRESHOLDdistance, leftlimit, rightlimit);
	int top = topANDbottom.first;
	int bottom = topANDbottom.second;

	if (top == source.rows - 5 || bottom == 4)
		return true; // 未检测到有效垂直范围，视为背景
	return false;
}
// 判断指定列区间[leftlimit, rightlimit]内是否为背景区域（即无显著条带）

float PlanC::GetRate(float ratio)
{
	float rate = (15 * ratio + 1) / 8;
	return rate;
}
// 动态调整阈值比例因子：灰度值越高，允许的绝对差值越大（适应高动态范围图像）

cv::Mat PlanC::GetMidMat(const cv::Mat& source, int midcol, int toplimit, int bottomlimit)
{
	cv::Mat newMat = cv::Mat::zeros(source.rows, 5, CV_16UC1);

	for (int r = toplimit; r <= bottomlimit; r++)
		for (int c = midcol - 2; c <= midcol + 2; c++)
			newMat.at<ushort>(r, c - midcol + 2) = source.at<ushort>(r, c);
	// 提取以midcol为中心、宽5列的子图像，用于精细分析

	return newMat;
}
// 从原图中提取以midcol为中心的5列区域（左右各2列），生成新的Mat用于局部处理

pair<int, int> PlanC::GetBandHeight(const cv::Mat& source, int row, int col, int THRESHOLDdistance, int MAXvalue)
{
	if (source.cols == 5) // 处理由GetMidMat生成的5列子图
	{
		int STANDARDvalue = (source.at<ushort>(row, 0) + source.at<ushort>(row, 1) +
			source.at<ushort>(row, 2) + source.at<ushort>(row, 3) + source.at<ushort>(row, 4)) / 5;
		int CURRENTvalue;
		float ratio = STANDARDvalue * 1.0 / MAXvalue;
		float RATE = GetRate(ratio);

		int top = row - 1;
		for (; top >= 5; top--)
		{
			CURRENTvalue = (source.at<ushort>(top, 0) + source.at<ushort>(top, 1) +
				source.at<ushort>(top, 2) + source.at<ushort>(top, 3) + source.at<ushort>(top, 4)) / 5;
			if (abs(STANDARDvalue - CURRENTvalue) > THRESHOLDdistance * RATE)
				break;
		}
		top++;

		int bottom = row + 1;
		for (; bottom < source.rows - 5; bottom++)
		{
			CURRENTvalue = (source.at<ushort>(bottom, 0) + source.at<ushort>(bottom, 1) +
				source.at<ushort>(bottom, 2) + source.at<ushort>(bottom, 3) + source.at<ushort>(bottom, 4)) / 5;
			if (abs(STANDARDvalue - CURRENTvalue) > THRESHOLDdistance * RATE)
				break;
		}
		bottom--;

		if (top < 0) // 边界保护
			top = 0;
		if (bottom >= source.rows)
			bottom = source.rows - 1;

		return pair<int, int>(top, bottom);
	}

	else // 处理原始宽图
	{
		int topsum = 0, bottomsum = 0;
		int STANDARDvalue = source.at<ushort>(row, col);
		float ratio = STANDARDvalue * 1.0 / MAXvalue;
		float RATE = GetRate(ratio);

		vector<int> testcols;
		for (int offset = -5; offset <= 5; offset++)
			if (col + offset >= 0 && col + offset < source.cols &&
				abs(source.at<ushort>(row, col + offset) - STANDARDvalue) <= THRESHOLDdistance * RATE)
				testcols.push_back(col + offset);

		int top;
		for (int i = 0; i < testcols.size(); i++)
		{
			for (top = row - 1; top >= 5; top--)
				if (STANDARDvalue - source.at<ushort>(top, testcols[i]) > THRESHOLDdistance * RATE)
					break;
			topsum += top + 1; // 补偿break后的偏移
		}

		int bottom;
		for (int i = 0; i < testcols.size(); i++)
		{
			for (bottom = row + 1; bottom < source.rows - 5; bottom++)
				if (STANDARDvalue - source.at<ushort>(bottom, testcols[i]) > THRESHOLDdistance * RATE)
					break;
			bottomsum += bottom - 1;
		}

		int topresult = round(topsum * 1.0 / testcols.size());
		int bottomresult = round(bottomsum * 1.0 / testcols.size());

		if (topresult < 0)
			topresult = 0;
		if (bottomresult >= source.rows)
			bottomresult = source.rows - 1;

		return pair<int, int>(topresult, bottomresult);
	}
}
// 根据中心点(row,col)和峰值，向上/下搜索确定条带的垂直范围（高度）

pair<int, int> PlanC::GetBandLength(const cv::Mat& source, int row, int col, int THRESHOLDdistance, int MAXvalue)
{
	int STANDARDvalue = source.at<ushort>(row, col);
	float ratio = STANDARDvalue * 1.0 / MAXvalue;
	float RATE = GetRate(ratio);

	vector<int> testrows;
	for (int offset = -2; offset <= 2; offset++)
		if (row + offset >= 0 && row + offset < source.rows &&
			STANDARDvalue - source.at<ushort>(row + offset, col) <= THRESHOLDdistance * RATE)
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
		for (; right <= source.cols - 6; right++) // 修复：原代码误用source.rows
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
// 根据中心点(row,col)和峰值，在多行上向左/右搜索确定条带的水平范围（长度）

pair<int, int> PlanC::GetReferenceLaneLength(const cv::Mat& source, int STANDARDlength, int THRESHOLDdistance)
{
	int count = 0;
	vector<int> testlengthVec;
	vector<pair<int, int>> testleftAndrightVec;

	int MAXvalue = 0;
	cv::Mat clone = source.clone();

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

		pair<int, int> topAndbottom = PlanC::GetBandHeight(clone, row, col, THRESHOLDdistance, MAXvalue);
		pair<int, int> leftAndright = PlanC::GetBandLength(clone, row, col, THRESHOLDdistance, MAXvalue);
		// 注意：此处调用的是PlanC自身的函数，而非PlanA或PlanB
		int top = topAndbottom.first;
		int bottom = topAndbottom.second;
		int left = leftAndright.first;
		int right = leftAndright.second;

		int testlength = right - left + 1;

		if (testlength <= STANDARDlength + 6 && testlength >= STANDARDlength - 6)
			return leftAndright; // 找到符合标准长度的条带，直接返回

		if (testlength <= STANDARDlength / 2 || testlength >= STANDARDlength * 3 / 2)
			count--; // 过短或过长，不计入有效样本
		else
		{
			testlengthVec.push_back(testlength);
			testleftAndrightVec.push_back(leftAndright);
		} // 仅记录接近标准长度的候选

		for (int r = top; r <= bottom; r++)
			for (int c = left; c <= right; c++)
				clone.at<ushort>(r, c) = 0;
		// 抹除已处理区域，避免重复检测

		count++;
		if (count == 5)
		{
			flag = 1;
			break;
		}
	}

	if (flag == 1)
	{
		int MINdistance = 65535;
		pair<int, int> MINleftAndright;
		for (int i = 0; i < testlengthVec.size(); i++)
			if (abs(testlengthVec[i] - STANDARDlength) < MINdistance)
				MINleftAndright = testleftAndrightVec[i];

		return MINleftAndright;
	}
}
// 寻找一个符合标准长度的参考条带，用于后续递归分割；若找不到，则返回最接近的候选

pair<int, int> PlanC::GetAllLaneLength_Rough(const cv::Mat& source, int THRESHOLDdistance)
{
	int leftBORDER = 5;
	int leftFLAG = 0;
	for (; leftBORDER <= source.cols - 6; leftBORDER++)
	{
		for (int r = 1; r < source.rows - 1; r++) // 跳过首尾行，避免边界效应
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
		for (int r = 1; r < source.rows - 1; r++)
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
// 粗略估计整幅图像中所有条带的左右边界（全局范围）

pair<int, int> PlanC::GetAllBandHeight(const cv::Mat& source, int THRESHOLDdistance, int leftlimit, int rightlimit)
{
	int topBORDER = 5;
	int topFLAG = 0;
	for (; topBORDER <= source.rows - 6; topBORDER++)
	{
		for (int c = leftlimit; c <= rightlimit; c++)
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
	for (; bottomBORDER >= 5; bottomBORDER--)
	{
		for (int c = leftlimit; c <= rightlimit; c++)
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
// 在指定列区间[leftlimit, rightlimit]内，粗略估计所有条带的上下边界

pair<int, int> PlanC::FindBackgroundOffset(const cv::Mat& midMat, int THRESHOLDdistance, int MAXvalue,
	int Bandtop, int Bandbottom, int toplimit, int bottomlimit)
{
	int topoffset;
	if (Bandtop - toplimit == 0)
		topoffset = 0;
	else
		for (topoffset = 1; topoffset <= Bandtop - toplimit; topoffset++)
		{
			int FRONTvalue = (midMat.at<ushort>(Bandtop - topoffset - 3, 0) + midMat.at<ushort>(Bandtop - topoffset - 3, 1) +
				midMat.at<ushort>(Bandtop - topoffset - 3, 2) + midMat.at<ushort>(Bandtop - topoffset - 3, 3) +
				midMat.at<ushort>(Bandtop - topoffset - 3, 4)) / 5;
			int BACKvalue = (midMat.at<ushort>(Bandtop - topoffset - 0, 0) + midMat.at<ushort>(Bandtop - topoffset - 0, 1) +
				midMat.at<ushort>(Bandtop - topoffset - 0, 2) + midMat.at<ushort>(Bandtop - topoffset - 0, 3) +
				midMat.at<ushort>(Bandtop - topoffset - 0, 4)) / 5;

			if (FRONTvalue == 0)
			{
				for (; midMat.at<ushort>(Bandtop - topoffset, 0) > 0; topoffset++) {}
				topoffset--;
				break;
			}

			if (FRONTvalue - BACKvalue > THRESHOLDdistance * GetRate(BACKvalue * 1.0 / MAXvalue))
				break;
		}

	int bottomoffset;
	if (bottomlimit - Bandbottom == 0)
		bottomoffset = 0;
	else
		for (bottomoffset = 1; bottomoffset <= bottomlimit - Bandbottom; bottomoffset++)
		{
			int FRONTvalue = (midMat.at<ushort>(Bandbottom - bottomoffset + 3, 0) + midMat.at<ushort>(Bandbottom - bottomoffset + 3, 1) +
				midMat.at<ushort>(Bandbottom - bottomoffset + 3, 2) + midMat.at<ushort>(Bandbottom - bottomoffset + 3, 3) +
				midMat.at<ushort>(Bandbottom - bottomoffset + 3, 4)) / 5;
			int BACKvalue = (midMat.at<ushort>(Bandbottom - bottomoffset - 0, 0) + midMat.at<ushort>(Bandbottom - bottomoffset - 0, 1) +
				midMat.at<ushort>(Bandbottom - bottomoffset - 0, 2) + midMat.at<ushort>(Bandbottom - bottomoffset - 0, 3) +
				midMat.at<ushort>(Bandbottom - bottomoffset - 0, 4)) / 5;

			if (BACKvalue == 0)
			{
				for (; midMat.at<ushort>(Bandbottom - bottomoffset, 0) > 0; bottomoffset++) {}
				bottomoffset--;
				break;
			}

			if (BACKvalue - FRONTvalue > THRESHOLDdistance * GetRate(FRONTvalue * 1.0 / MAXvalue))
				break;
		}

	return pair<int, int>(topoffset, bottomoffset);
}
// 在条带上下方寻找背景过渡区域的偏移量，用于确定需清除的完整行范围

Band PlanC::FindBandByMaxPoint(const cv::Mat& source, const cv::Mat& midMat,
	int THRESHOLDdistance, int STANDARDlength, int MAXvalue,
	int leftlimit, int rightlimit, int toplimit, int bottomlimit)
{
	int midcol = (leftlimit + rightlimit) / 2;

	PointInfoTYPE MAXpointinfo = GetMaxPointInfo(midMat);

	int STANDARDvalue = get<1>(MAXpointinfo);
	int MAXcol = get<0>(MAXpointinfo).first + midcol - 2; // 修正：GetMaxPointInfo返回的是midMat内的坐标，需映射回原图
	int MAXrow = get<0>(MAXpointinfo).second;

	if (STANDARDvalue < THRESHOLDdistance) // 峰值过低，视为噪声
		return Band(0, 0, 0, 0, 0);

	pair<int, int> BandtopANDbottom =
		PlanC::GetBandHeight(midMat, MAXrow, 2, THRESHOLDdistance, MAXvalue); // 在5列子图中计算高度
	pair<int, int> BandleftANDright =
		PlanC::GetBandLength(source, MAXrow, MAXcol, THRESHOLDdistance, MAXvalue); // 在原图中计算宽度

	int Bandleft = BandleftANDright.first;
	int Bandright = BandleftANDright.second;
	int Bandtop = BandtopANDbottom.first;
	int Bandbottom = BandtopANDbottom.second;

	int length = Bandright - Bandleft + 1;
	int height = Bandbottom - Bandtop + 1;
	int area = length * height;
	if (area <= (STANDARDlength * 0.75) * 2 && area <= (STANDARDlength - 3) * 2)
		return Band(Bandtop, Bandbottom, -1, -1, 0); // 面积过小，标记为无效
	if (length <= STANDARDlength / 2 || length >= STANDARDlength * 3 / 2)
		return Band(Bandtop, Bandbottom, -2, -2, 0); // 长度过短或过长，标记为无效

	return Band(Bandtop, Bandbottom, Bandleft, Bandright, STANDARDvalue);
}
// 基于局部最大值点，提取一个完整的条带区域（Band对象）

vector<Band> PlanC::SortBand(vector<Band>& BandareaVec)
// 对BandareaVec中的条带按垂直位置（top）进行升序排序
{
	vector<Band> SORTEDBandareaVec;

	if (BandareaVec.size() > 1)
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
		SORTEDBandareaVec.push_back(BandareaVec.back());
	}

	return SORTEDBandareaVec;
}
// 将条带列表按其顶部行坐标（top）从小到大排序

vector<Band> PlanC::AnalyzeSingleLane(const cv::Mat& source,
	int THRESHOLDdistance, int STANDARDlength, int MAXvalue,
	int leftlimit, int rightlimit)
{
	pair<int, int> topANDbottom = GetAllBandHeight(source, THRESHOLDdistance, leftlimit, rightlimit);
	int toplimit = topANDbottom.first;
	int bottomlimit = topANDbottom.second;

	cv::Mat midMatclone = GetMidMat(source, (leftlimit + rightlimit) / 2, toplimit, bottomlimit); // 提取中心5列

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
			break; // 无有效峰值，退出循环

		if ((Bandleft == -1 && Bandright == -1) || (Bandleft == -2 && Bandright == -2)) // 无效条带
		{
			// 用上下背景均值填充该区域，避免干扰后续检测
			for (int r = Bandtop; r <= Bandbottom; r++)
				for (int c = 0; c < 5; c++)
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
		if (totaltop <= toplimit && totalbottom >= bottomlimit)
			break; // 已覆盖整个区域

		ClearRowArea(midMatclone, totaltop, totalbottom); // 清除已处理区域
	}

	return BandareaVec;
}
// 分析单个条带列区间内的所有局部条带（垂直堆叠的多个斑点）

vector<vector<Band>> PlanC::SortLane(vector<vector<Band>>& BandList)
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
	SORTEDBandList.push_back(BandList.front());

	return SORTEDBandList;
}
// 将多个条带列（lane）按其最左边界（left）从小到大排序