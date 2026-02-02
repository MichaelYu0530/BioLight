#include "GeneralHead.h"
#include "PlanAHead.h"

#include <math.h>
#include <iostream>

using namespace std;
using namespace PlanA;
using namespace General;

typedef tuple<pair<int, int>, int> PointInfoTYPE;
// tuple(坐标(row, col), 灰度值)
typedef vector<vector<int>> BandSummitsTYPE;
// vector(峰值点行坐标列表, 对应灰度差值列表)
typedef tuple<int, int, int, BandSummitsTYPE> LaneInfoTYPE;
// tuple(左边界, 右边界, 条带中心列, 峰值信息)
typedef tuple<int, int, int, int> BandAreaTYPE;
// tuple(上边界, 下边界, 左边界, 右边界)

bool PlanA::IsBackground(const cv::Mat& source, int col)
{
	cv::Mat colMat = source.col(col);

	int ThresholdRange = 2000; // 灰度动态范围阈值，待测试调整

	double minv = 0.0, maxv = 0.0;
	double* minp = &minv;
	double* maxp = &maxv;

	cv::minMaxIdx(colMat, minp, maxp);

	return (maxv - minv) < ThresholdRange;
}
// 判断该列是否为背景（即整列灰度变化很小，无显著条带）

bool PlanA::IsSummit(const vector<int>& Difference, int index)
{
	float ThresholdRate = 0.5; // 峰值衰减比例阈值，待测试

	if (Difference[index] > 0)
	{
		for (int i = 1; i <= 5; i++)
			if (Difference[index - i] > Difference[index] || Difference[index + i] > Difference[index]) // 非局部最大值
				return false;
		if (Difference[index - 4] < (Difference[index] * ThresholdRate) || Difference[index - 5] < (Difference[index] * ThresholdRate))
			return true;
		else
			return false;
		return true;
	}
	else // 负峰（谷值）
	{
		for (int i = 1; i <= 5; i++)
			if (Difference[index - i] < Difference[index] || Difference[index + i] < Difference[index]) // 非局部最小值
				return false;
		if (Difference[index + 4] > (Difference[index] * ThresholdRate) || Difference[index + 5] > (Difference[index] * ThresholdRate))
			return true;
		else
			return false;
		return true;
	}
}
// 判断某点是否为显著的灰度差值峰值（正峰或负峰）

bool PlanA::IsInRectArea(int testrow, int testcol, BandAreaTYPE Bandarea)
{
	int top = get<0>(Bandarea);
	int bottom = get<1>(Bandarea);
	int left = get<2>(Bandarea);
	int right = get<3>(Bandarea);

	if (testrow < top || testrow > bottom || testcol < left || testcol > right)
		return false;
	else
		return true;
}
// 判断某点(testrow, testcol)是否位于指定矩形区域内

bool PlanA::IsInRectArea(cv::Point testpoint, cv::Point LTcorner, cv::Point RBcorner) // LT == left&top, RB == right&bottom
{
	int top = LTcorner.y;
	int bottom = RBcorner.y;
	int left = LTcorner.x;
	int right = RBcorner.x;
	int testrow = testpoint.y;
	int testcol = testpoint.x;

	if (testrow < top || testrow > bottom || testcol < left || testcol > right)
		return false;
	else
		return true;
}
// 重载版本：使用OpenCV的Point类型判断点是否在矩形内

void PlanA::InsertPoint(BandSummitsTYPE& summits, int point, int value)
{
	vector<int> summitpoints = summits[0], summitvalues = summits[1];
	vector<int>::iterator pointIter = summits[0].begin(), valueIter = summits[1].begin();

	if (point < summitpoints.front()) // point应插入到首位
	{
		summits[0].insert(pointIter, point);
		summits[1].insert(valueIter, value);
	}
	else
		if (point > summitpoints.back()) // point应插入到末位
		{
			summits[0].push_back(point);
			summits[1].push_back(value);
		}
		else
			for (pointIter++, valueIter++; pointIter < summits[0].end(); pointIter++, valueIter++)
				if (point > *(pointIter - 1) && point < *pointIter) // point应插入到中间
				{
					summits[0].insert(pointIter, point);
					summits[1].insert(valueIter, value);
					break;
				}
}
// 将新的峰值点(point)及其值(value)按行坐标顺序插入到summits中

void PlanA::OutputLaneInfo(const vector<LaneInfoTYPE>& laneinfoVec)
{
	for (int i = 0; i < laneinfoVec.size(); i++)
	{
		int left = get<0>(laneinfoVec[i]);
		int right = get<1>(laneinfoVec[i]);
		int Bandcol = get<2>(laneinfoVec[i]);
		BandSummitsTYPE Bandsummits = get<3>(laneinfoVec[i]);
		// Bandsummits: [0]为行坐标, [1]为对应灰度差值

		cout << "Left=" << left << '\t' << "Right=" << right << '\t' << "Length=" << right - left << '\n';
		for (int j = 0; j < Bandsummits[0].size(); j++)
			cout << Bandsummits[0][j] << '\t' << Bandsummits[1][j] << '\n';
		cout << "\n\n";
	}
}
// 打印所有条带（lane）的检测结果信息

void PlanA::DrawRectInOrigin(const cv::Mat& origin, const BandAreaTYPE& Bandarea)
{
	int top = (get<0>(Bandarea) - 5) * 3 + 1 - 1;
	int bottom = (get<1>(Bandarea) - 5) * 3 + 1 + 1;
	int left = (get<2>(Bandarea) - 5) * 5 + 2 - 2;
	int right = (get<3>(Bandarea) - 5) * 5 + 2 + 2; // 因原图经过3x5下采样，此处需反向映射回原始图像坐标（+1/-1仅为视觉效果微调）

	cv::Point p1(left, top);
	cv::Point p2(right, bottom);
	// 在原始高分辨率图像上绘制矩形框

	cv::rectangle(origin, p1, p2, cv::Scalar(0, 0, 255), 2);
}
// 在原始24位BGR图像上绘制检测到的条带区域（红色框）

void PlanA::DrawRectInProcessed(const cv::Mat& processed, const BandAreaTYPE& Bandarea)
{
	int top = get<0>(Bandarea) - 1;
	int bottom = get<1>(Bandarea) + 1;
	int left = get<2>(Bandarea) - 1;
	int right = get<3>(Bandarea) + 1; // 绘图时略微扩大边界（±1仅为便于观察，实际区域无偏移）
	// 直接在当前处理的Mat图像上绘图
	cv::Point p1(left, top);
	cv::Point p2(right, bottom);
	// 在当前图像上绘制矩形框

	cv::rectangle(processed, p1, p2, cv::Scalar(0, 0, 255), 1);
}
// 在处理后的24位BGR图像上绘制检测到的条带区域（红色细框）

PointInfoTYPE PlanA::GetStandardPointInfo(const cv::Mat& source, int row, int col)
{
	int MAXrow, MAXcol, MAX = 0;
	for (int r = 0; r < 7; r++)
		for (int c = 0; c < 7; c++)
			if (source.at<ushort>(r + row - 3, c + col - 2) > MAX)
			{
				MAXrow = r + row - 3;
				MAXcol = c + col - 2;
				MAX = source.at<ushort>(r + row - 3, c + col - 2);
			}

	pair<int, int> coordinate(MAXrow, MAXcol);
	return make_tuple(coordinate, MAX);
}
// 在以(row,col)为中心的7x7邻域内寻找真正的灰度最大值点（亚像素精化）

LaneInfoTYPE PlanA::GetLaneInfoSpecial(const cv::Mat& source, int left, int right) // 此函数专用于已知左右边界的条带区域进行精细搜索
{
	int Bandcol = (left + right) / 2;
	vector<BandSummitsTYPE> summitsVec;

	for (int i = left; i <= right; i++)
	{
		BandSummitsTYPE summits = FindSummit(source, i);
		if (summits.size() > 0)
			summitsVec.push_back(summits);
	}

	BandSummitsTYPE Bandsummits = GetSummitUnion(summitsVec); // 此处仅为初步合并，后续可改进
	summitsVec.clear();

	LaneInfoTYPE laneinfo = make_tuple(left, right, Bandcol, Bandsummits);

	return laneinfo;
}
// 对已知左右边界的条带区域，重新提取并合并峰值信息

BandSummitsTYPE PlanA::GetSummitUnion(const vector<BandSummitsTYPE>& summitsVec)
{
	int begin = 0;
	BandSummitsTYPE summitunion = summitsVec[0]; // GetLaneInfoSpecial中保证summitsVec非空

	for (int i = begin; i < summitsVec.size(); i++)
	{
		vector<int> differentpoints;
		for (int j = 0; j < summitsVec[i][0].size(); j++)
		{
			int point = summitsVec[i][0][j], value = summitsVec[i][1][j];
			int index = ContainingIndex(point, summitunion[0], 5);

			if (index == -1)
				InsertPoint(summitunion, point, value);
			else
			{
				// 保留更高灰度值的点
				summitunion[0][index] = summitunion[1][index] > value ? summitunion[0][index] : point;
				summitunion[1][index] = summitunion[1][index] > value ? summitunion[1][index] : value;
			}
		}
	}

	return summitunion;
}
// 合并多个列的峰值点：相近位置（±5行）的点取灰度值更高的那个

BandSummitsTYPE PlanA::FindSummit(const cv::Mat& source, int col)
{
	BandSummitsTYPE summits;
	vector<int> summitpoints;
	vector<int> summitvalues;

	if (IsBackground(source, col))
		return summits; // 若为背景列，直接返回空结果

	int ThresholdDifference = 750; // 灰度差值显著性阈值，待测试调整
	vector<int> values;
	values = GetPointDifferenceVec(source.col(col), 5); // 计算该列的5行间隔灰度差值序列

	for (int i = 5; i < values.size() - 5; i++)
	{
		if (abs(values[i]) < ThresholdDifference) {}
		else
			if (IsSummit(values, i))
			{
				summitpoints.push_back(i + 5); // 补偿起始偏移
				summitvalues.push_back(values[i]);
			}
	}

	int isfindingpair = 0;
	vector<int> satisfyingpoints;
	// 寻找正负峰对：isfindingpair == 0 表示尚未找到正峰（起始阶段）
	for (int i = 0; i < summitpoints.size(); i++)
	{
		if (isfindingpair == 0)
			if (summitvalues[i] > 0) // 第一个峰应为正峰（上升沿）
				isfindingpair = 1; // 找到正峰后，开始寻找对应的负峰
		if (isfindingpair == 1)
			if (summitvalues[i] <= 0)
			{
				isfindingpair = 0; // 成功配对，重置状态
				satisfyingpoints.push_back(i - 1); // 记录正峰索引（即配对成功的第一个峰）
			}
		// 说明：只保留能形成完整正-负峰对的正峰
	}

	if (satisfyingpoints.size() == 0)
		return summits; // 未成功配对，直接返回空

	vector<int> summitpoints0;
	vector<int> summitvalues0;

	for (int i = 0; i < satisfyingpoints.size(); i++)
	{
		int index = satisfyingpoints[i];
		summitpoints0.push_back(summitpoints[index]);
		summitvalues0.push_back(summitvalues[index]);
	}

	summits.push_back(summitpoints0);
	summits.push_back(summitvalues0);
	return summits;
}
// 在单列中查找有效的灰度差值峰值对（正峰+负峰），若全为背景则返回空的二维vector
// 注意：当summits.size() == 0时，不可访问summits[0]，否则会越界崩溃

vector<LaneInfoTYPE> PlanA::CheckAndReviseBandLength(const cv::Mat& source,
	const vector<LaneInfoTYPE>& laneinfoVec)
{
	int standardSINGLEinterval = 3; // 单个条带之间的标准间隔（同一图像中可能有多条），此处暂定为3列
	int standardSINGLElength = GetStandardBandLength(source); // 获取标准单条带长度

	vector<int> standardlengthVec;
	// 存储所有可能的组合长度：例如1条、2条、3条...拼接后的总长度
	for (int i = 0; i < 20; i++) // 第i个元素表示i+1个条带的总宽度（含间隔）
		standardlengthVec.push_back(i * standardSINGLElength + (i - 1) * standardSINGLEinterval);

	vector<LaneInfoTYPE> revisedlaneinfoVec;
	for (int i = 0; i < laneinfoVec.size(); i++)
	{
		int left = get<0>(laneinfoVec[i]);
		int right = get<1>(laneinfoVec[i]); // 此处的left/right来自初步检测的条带左右边界
		int length = right - left;

		int lanecount = ContainingIndex(length, standardlengthVec);

		if (lanecount == 1) // lanecount == 1 表示识别正确（单条带）
			revisedlaneinfoVec.push_back(laneinfoVec[i]);
		else // lanecount > 1 表示当前区域被误识别为单条带，实际包含多条
		{
			for (int j = 0; j < lanecount; j++)
			{
				LaneInfoTYPE revisedlaneinfo;
				int revisedleft, revisedright;
				if (j == 0)
				{
					revisedleft = left; // -1仅为视觉占位
					revisedright = left + standardSINGLElength + standardSINGLEinterval / 2;
				}
				if (j == lanecount - 1)
				{
					revisedleft = right - standardSINGLElength - standardSINGLEinterval / 2;
					revisedright = right; // +1仅为视觉占位
				}
				if (j > 0 && j < lanecount - 1)
				{
					revisedleft = left + (standardSINGLElength + standardSINGLEinterval) * j;
					revisedright = revisedleft + standardSINGLElength;
				}
				revisedlaneinfo = GetLaneInfoSpecial(source, revisedleft, revisedright);

				revisedlaneinfoVec.push_back(revisedlaneinfo);
			}
		}
	}

	return revisedlaneinfoVec;
}
// 检查并修正条带长度：将被误合并的多条带拆分为多个独立条带

vector<LaneInfoTYPE> PlanA::GetLaneInfo(const cv::Mat& source, int begin, int end)
{
	vector<LaneInfoTYPE> laneinfoVec;
	// 返回的每个元素为tuple(条带左边界, 右边界, 中心列, 峰值信息)

	int MINBandlength = 6; // 最小有效条带宽度（列数），小于该值视为噪声
	int isBandfound = 0;
	int Bandcol = 0;
	int MAXBandleft = 0, MAXBandright = 0;

	vector<BandSummitsTYPE> summitsVec;

	for (int i = begin; i < end; i++)
	{
		BandSummitsTYPE summits = FindSummit(source, i);

		if (summits.size() > 0) // summits.size() == 0 表示该列为背景
		{
			if (isBandfound == 0) // 之前未找到条带时，isBandfound == 0
			{
				isBandfound = 1; // 标记已进入条带区域
				MAXBandleft = i; // 首次检测到非背景列，记录为左边界

				summitsVec.push_back(summits);
			}
			if (isBandfound == 1)
				summitsVec.push_back(summits);
		}
		if ((summits.size() == 0 || (i == end - 1 && summits.size() != 0)) && isBandfound == 1) // isBandfound == 1 且遇到背景列（或到达末尾），表示条带结束
		{
			isBandfound = 0; // 重置状态，准备下一次检测
			if (i == end - 1)
			{
				MAXBandright = i;
				if (summits.size() > 0)
					summitsVec.push_back(summits);
			}
			else
				MAXBandright = i - 1;
			Bandcol = (MAXBandleft + MAXBandright) / 2;
			int Bandlength = MAXBandright - MAXBandleft + 1;

			if (Bandlength <= MINBandlength) // 条带过窄，视为噪声，跳过
			{
				summitsVec.clear();
				continue;
			}

			BandSummitsTYPE Bandsummits = GetSummitUnion(summitsVec); // 合并该条带内所有列的峰值
			summitsVec.clear();

			LaneInfoTYPE laneinfo = make_tuple(MAXBandleft, MAXBandright, Bandcol, Bandsummits);
			laneinfoVec.push_back(laneinfo);
		}
	}

	return laneinfoVec;
}
// 在[begin, end)列范围内检测所有条带，并返回每个条带的左右边界及峰值信息

vector<LaneInfoTYPE> PlanA::DeleteSurplusSummits(const cv::Mat& source, const vector<LaneInfoTYPE>& laneinfoVec)
{
	vector<LaneInfoTYPE> IMPROVEDlaneinfoVec;
	int MAXdifference = GetMaxDistance(source);
	int ThresholdValue = MAXdifference / 10; // 峰值显著性阈值，待测试

	for (int i = 0; i < laneinfoVec.size(); i++)
	{
		int left = get<0>(laneinfoVec[i]);
		int right = get<1>(laneinfoVec[i]);
		int length = right - left;
		int Bandcol = get<2>(laneinfoVec[i]);
		BandSummitsTYPE RAWBandsummits = get<3>(laneinfoVec[i]);
		// RAWBandsummits: [0]为行坐标, [1]为灰度差值

		// 第一步：删除灰度差值过小的峰值点
		vector<int>::iterator pointiter = RAWBandsummits[0].begin();
		vector<int>::iterator valueiter = RAWBandsummits[1].begin();
		for (; pointiter < RAWBandsummits[0].end(); )
		{
			int point = *pointiter, value = *valueiter;
			if (value <= ThresholdValue)
			{
				pointiter = RAWBandsummits[0].erase(pointiter);
				valueiter = RAWBandsummits[1].erase(valueiter);
			}
			else
			{
				pointiter++;
				valueiter++;
			}
		}
		BandSummitsTYPE& PROCESSEDBandsummits = RAWBandsummits; // 为RAWBandsummits取一个别名

		if (PROCESSEDBandsummits[0].size() == 0)
			continue;

		// 第二步：统计每个峰值在条带宽度范围内被多少列支持
		vector<int> countVec(PROCESSEDBandsummits[0].size(), 0);
		for (int j = left; j <= right; j++)
		{
			BandSummitsTYPE Bandsummits = FindSummit(source, j);
			if (Bandsummits.size() == 0)
				continue;
			for (int k = 0; k < Bandsummits[0].size(); k++)
			{
				int index = ContainingIndex(Bandsummits[0][k], PROCESSEDBandsummits[0], 5);
				if (index >= 0 && Bandsummits[1][k] > PROCESSEDBandsummits[1][index] * 0.5)
					countVec[index]++;
			}
		}

		// 第三步：删除支持列数不足的峰值（少于条带宽度的50%）
		vector<int>::iterator count_iter = countVec.begin();
		vector<int>::iterator point_iter = PROCESSEDBandsummits[0].begin();
		vector<int>::iterator value_iter = PROCESSEDBandsummits[1].begin();
		for (; count_iter < countVec.end(); )
			if (*count_iter < length * 0.5)
			{
				count_iter = countVec.erase(count_iter);
				point_iter = PROCESSEDBandsummits[0].erase(point_iter);
				value_iter = PROCESSEDBandsummits[1].erase(value_iter);
			}
			else
			{
				count_iter++;
				point_iter++;
				value_iter++;
			}
		BandSummitsTYPE& IMPROVEDBandsummits = PROCESSEDBandsummits; // 为最终结果取别名

		LaneInfoTYPE IMPROVEDlaneinfo = make_tuple(left, right, Bandcol, IMPROVEDBandsummits);
		IMPROVEDlaneinfoVec.push_back(IMPROVEDlaneinfo);
	}

	return IMPROVEDlaneinfoVec;
}
// 对GetSummitUnion得到的峰值进行二次筛选：去除弱响应和低支持度的虚假峰值

vector<BandAreaTYPE> PlanA::LocateBandInSingleLane(const cv::Mat& source, const LaneInfoTYPE& laneinfo)
{
	int left = get<0>(laneinfo);
	int right = get<1>(laneinfo);
	int Bandcol = get<2>(laneinfo);
	BandSummitsTYPE Bandsummits = get<3>(laneinfo);

	vector<BandAreaTYPE> BandareaVec;
	for (int i = 0; i < Bandsummits[0].size(); i++)
	{
		int point = Bandsummits[0][i];
		PointInfoTYPE STANDARDpointinfo = GetStandardPointInfo(source, point, Bandcol);
		int row = get<0>(STANDARDpointinfo).first, col = get<0>(STANDARDpointinfo).second;
		int STANDARDvalue = get<1>(STANDARDpointinfo);

		pair<int, int> Bandheight = GetBandHeight(source, row, col, STANDARDvalue);
		pair<int, int> Bandlength = GetBandLength(source, row, col, STANDARDvalue, left, right);
		BandAreaTYPE Bandarea = make_tuple(Bandheight.first, Bandheight.second, Bandlength.first, Bandlength.second);
		// Bandarea: [0]=top, [1]=bottom, [2]=left, [3]=right

		BandareaVec.push_back(Bandarea);
	}

	return BandareaVec;
}
// 在单个条带（lane）内，根据每个峰值点精确定位其对应的局部条带区域（矩形框）