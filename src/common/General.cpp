#include "GeneralHead.h"
#include "ClassHead.h"
#include "PlanAHead.h"
#include "PlanBHead.h"

#include <math.h>
#include <iostream>

using namespace std;
using namespace General;

typedef tuple<pair<int, int>, int> PointInfoTYPE;
// tuple(坐标(row, col), 灰度值)
typedef vector<vector<int>> BandSummitsTYPE;
// vector(峰值点坐标列表, 对应灰度值列表)
typedef tuple<int, int, int, BandSummitsTYPE> LaneInfoTYPE;
// tuple(左边界, 右边界, 条带中心列, BandSummitsTYPE)
typedef tuple<int, int, int, int> BandAreaTYPE;
// tuple(上边界, 下边界, 左边界, 右边界)

int General::GetMaxDistance(const cv::Mat& source, int distance)
{
	int MAX = 0;
	for (int c = 5; c < source.cols - 5; c++)
	{
		vector<int> differenceVec = GetPointDifferenceVec(source.col(c), distance);
		for (vector<int>::iterator iter = differenceVec.begin(); iter < differenceVec.end(); iter++)
			if (MAX < abs(*iter))
				MAX = abs(*iter);
	}
	return MAX;
}
// 获取Mat图像中，同列且竖直距离为distance个单位的两像素点灰度值之差的最大绝对值

int General::ContainingIndex(int value, const vector<int>& testVec)
{
	int standardinterval = testVec[0] * -1;
	int standardlength = testVec[1];

	for (int i = 0; i < testVec.size() - 1; i++)
		if (value > testVec[i] && value <= testVec[i + 1])
			if (i == 0)
				return 1;
			else
				return abs(value - testVec[i]) <= abs(value - testVec[i + 1]) ? i : i + 1;
}
// 判断value落在testVec哪个区间内，并返回较近的区间索引（用于分段映射）

int General::ContainingIndex(int value, const vector<int>& testVec, int MAXlimit)
{
	int ThresholdDistance = MAXlimit;

	int flag = 0; int index;
	for (index = 0; index < testVec.size(); index++)
		if (abs(value - testVec[index]) <= ThresholdDistance)
		{
			flag = 1;
			break;
		}
	return flag ? index : -1;
}
// 在testVec中查找与value距离不超过MAXlimit的元素，返回其索引；若无则返回-1

int General::ContainingIndex(int value, const vector<double>& testVec, int MAXlimit)
{
	int ThresholdDistance = MAXlimit;

	int flag = 0; int index;
	for (index = 0; index < testVec.size(); index++)
		if (abs(value - testVec[index]) <= ThresholdDistance)
		{
			flag = 1;
			break;
		}
	return flag ? index : -1;
}
// 重载版本：支持double类型的testVec

int General::GetStandardBandLength(const cv::Mat& source, int THRESHOLDdistance, int MAXvalue)
{
	cv::Mat clone = source.clone();

	vector<double> differentlengthVec;
	vector<int> countVec;

	while (1)
	{
		double maxv;
		double* maxptr = &maxv;
		cv::Point maxpoint;
		cv::Point* maxpointptr = &maxpoint;

		cv::minMaxLoc(clone, nullptr, maxptr, nullptr, maxpointptr);

		int row = maxpointptr->y;
		int col = maxpointptr->x;
		int value = (int)*maxptr;

		pair<int, int> topAndbottom;
		pair<int, int> leftAndright;
		if (THRESHOLDdistance > 0) // THRESHOLDdistance > 0 表示使用PlanB方案
		{
			topAndbottom = PlanB::GetBandHeight_Rough(clone, row, col, THRESHOLDdistance, MAXvalue);
			leftAndright = PlanB::GetBandLength_Rough(clone, row, col, THRESHOLDdistance, MAXvalue);
		}
		else // THRESHOLDdistance为默认0，表示使用PlanA方案
		{
			topAndbottom = General::GetBandHeight(clone, row, col, value);
			leftAndright = General::GetBandLength(clone, row, col, value, 5, source.cols - 6);
		}

		int top = topAndbottom.first;
		int bottom = topAndbottom.second;
		int left = leftAndright.first;
		int right = leftAndright.second;
		int height = bottom - top + 1;
		int length = right - left + 1;
		int area = height * length;

		int MINarea = 20; // 最小有效区域面积，待测试调整
		if (area <= MINarea) // 若区域太小，视为噪声
		{
			for (int r = top; r <= bottom; r++)
				for (int c = left; c <= right; c++)
					clone.at<ushort>(r, c) = 0; // 抹除该区域
			continue;
		}
		else
		{
			if (differentlengthVec.size() == 0) // 首次记录
			{
				differentlengthVec.push_back(length);
				countVec.push_back(1);
			}
			else
			{
				// 检查当前length是否与已有长度相近（容差±2）
				int index = ContainingIndex(length, differentlengthVec, 2);
				if (index >= 0)
				{
					// 聚类合并：根据出现次数加权平均
					if (countVec[index] == 1)
						differentlengthVec[index] = (differentlengthVec[index] + length) / 2;
					if (countVec[index] == 2)
						differentlengthVec[index] = (differentlengthVec[index] * 2 + length) / 3;
					// 合并后取平均值，同时保留原始数据的权重影响
					countVec[index]++;
				}
				else
				{
					differentlengthVec.push_back(length);
					countVec.push_back(1);
				}
			}
			for (int r = top; r <= bottom; r++)
				for (int c = left; c <= right; c++)
					clone.at<ushort>(r, c) = 0; // 抹除已处理区域
		}

		if (countVec.size() > 0) // countVec.size() == differentlengthVec.size()
			for (int i = 0; i < countVec.size(); i++)
				if (countVec[i] >= 3) // 若某长度出现3次及以上，认为是标准条带长度
				{
					int floor = int(differentlengthVec[i]);
					return differentlengthVec[i] - float(floor) >= 0.5 ? floor + 1 : floor; // 四舍五入取整
				}
	}
}
// 通过迭代提取图像中的条带区域，统计其长度，返回出现频率最高的典型长度（四舍五入整数）

cv::Mat General::Gray16ToBGR(const cv::Mat& source) //
{
	int row = source.rows, col = source.cols;
	cv::Mat target8b = cv::Mat::zeros(row, col, CV_8UC1); // 创建8位单通道目标图

	double minv = 0.0, maxv = 0.0;
	double* minp = &minv;
	double* maxp = &maxv;

	cv::minMaxIdx(source, minp, maxp);

	for (int r = 0; r < row; r++)
		for (int c = 0; c < col; c++)
		{
			int value = (source.at<ushort>(r, c) - *minp) / (*maxp - *minp) * 255; // 归一化到0～255
			target8b.at<uchar>(r, c) = value;
		}

	cv::Mat targetBGR = cv::Mat::zeros(row, col, CV_8UC3); // 创建24位三通道BGR图

	std::vector<cv::Mat> channels;
	for (int i = 0; i < 3; i++)
		channels.push_back(target8b);

	cv::merge(channels, targetBGR); // 三通道合并为彩色图（实际为灰度伪彩）

	return targetBGR;
}
// 将16位灰度Mat图像转换为24位BGR格式图像（用于显示）

cv::Mat General::Divide3X5(const cv::Mat& source)
{
	int row = source.rows, col = source.cols;
	int newrow = row / 3, newcol = col / 5;
	cv::Mat target = cv::Mat(newrow, newcol, CV_16UC1);

	for (int r = 0; r < newrow; r++)
		for (int c = 0; c < newcol; c++)
		{
			int sum = source.at<ushort>(r * 3, c * 5 + 0) + source.at<ushort>(r * 3 + 1, c * 5 + 0) + source.at<ushort>(r * 3 + 2, c * 5 + 0) +
				source.at<ushort>(r * 3, c * 5 + 1) + source.at<ushort>(r * 3 + 1, c * 5 + 1) + source.at<ushort>(r * 3 + 2, c * 5 + 1) +
				source.at<ushort>(r * 3, c * 5 + 2) + source.at<ushort>(r * 3 + 1, c * 5 + 2) + source.at<ushort>(r * 3 + 2, c * 5 + 2) +
				source.at<ushort>(r * 3, c * 5 + 3) + source.at<ushort>(r * 3 + 1, c * 5 + 3) + source.at<ushort>(r * 3 + 2, c * 5 + 3) +
				source.at<ushort>(r * 3, c * 5 + 4) + source.at<ushort>(r * 3 + 1, c * 5 + 4) + source.at<ushort>(r * 3 + 2, c * 5 + 4);
			target.at<ushort>(r, c) = sum / 15; // 3x5区域均值下采样
		}

	return target;
}
// 对图像进行3行5列的块均值下采样（降分辨率）

cv::Mat General::ExtendBackground(const cv::Mat& source, int mode)
{
	int row = source.rows, col = source.cols;
	cv::Mat target = cv::Mat::zeros(row + 10, col + 10, CV_16UC1);

	if (mode == 1)
	{
		for (int r = 0; r < row + 10; r++)
			for (int c = 0; c < col + 10; c++)
				if ((r < 5 || r >= row + 5) && (c >= 5 && c < col + 5))
					target.at<ushort>(r, c) = source.at<ushort>(r < 5 ? 0 : row - 1, c - 5); // 上下边缘用首/末行填充
				else
					if ((c < 5 || c >= col + 5) && (r >= 5 && r < row + 5))
						target.at<ushort>(r, c) = source.at<ushort>(r - 5, c < 5 ? 0 : col - 1); // 左右边缘用首/末列填充
					else
						if ((r >= 5 && r < row + 5) && (c >= 5 && c < col + 5))
							target.at<ushort>(r, c) = source.at<ushort>(r - 5, c - 5); // 中心区域原样复制
		// 四个角落用原图四个角的像素值填充
		for (int i = 0; i < 5; i++)
			for (int j = 0; j < 5; j++)
			{
				target.at<ushort>(i, j) = source.at<ushort>(0, 0); // 左上角
				target.at<ushort>(row + 10 - 1 - i, j) = source.at<ushort>(row - 1, 0); // 左下角
				target.at<ushort>(row + 10 - 1 - i, col + 10 - 1 - j) = source.at<ushort>(row - 1, col - 1); // 右下角
				target.at<ushort>(i, col + 10 - 1 - j) = source.at<ushort>(0, col - 1); // 右上角
			}
	}
	else
		// mode != 1 时仅将原图置于新图中心，其余区域保持0（不填充边缘）
		for (int r = 5; r < source.rows - 5; r++)
			for (int c = 5; c < source.cols - 5; c++)
				target.at<ushort>(r, c) = source.at<ushort>(r - 5, c - 5);

	return target;
}
// 扩展图像背景：在四周添加5像素宽的边框，mode=1时用边缘值填充，否则仅居中放置

pair<int, int> General::GetBandHeight(const cv::Mat& source, int row, int col, int STANDARDvalue)
{
	int top = row;
	for (; top >= 5; top--)
	{
		if (source.at<ushort>(top - 0, col) < STANDARDvalue * 0.25)
			break;
		if (source.at<ushort>(top - 0, col) < STANDARDvalue * 0.5 &&
			source.at<ushort>(top - 1, col) < STANDARDvalue * 0.5)
			break;
		if (source.at<ushort>(top - 0, col) < STANDARDvalue * 0.75 &&
			source.at<ushort>(top - 1, col) < STANDARDvalue * 0.75 &&
			source.at<ushort>(top - 2, col) < STANDARDvalue * 0.75)
			break;
		if (source.at<ushort>(top - 0, col) < STANDARDvalue * 0.75 &&
			source.at<ushort>(top - 1, col) < STANDARDvalue * 0.75 &&
			source.at<ushort>(top - 2, col) > STANDARDvalue * 0.75 &&
			source.at<ushort>(top - 3, col) < STANDARDvalue * 0.75)
			break;
	}

	int bottom = row;
	for (; bottom <= source.rows - 5; bottom++)
	{
		if (source.at<ushort>(bottom + 0, col) < STANDARDvalue * 0.25)
			break;
		if (source.at<ushort>(bottom + 0, col) < STANDARDvalue * 0.5 &&
			source.at<ushort>(bottom + 1, col) < STANDARDvalue * 0.5)
			break;
		if (source.at<ushort>(bottom + 0, col) < STANDARDvalue * 0.75 &&
			source.at<ushort>(bottom + 1, col) < STANDARDvalue * 0.75 &&
			source.at<ushort>(bottom + 2, col) < STANDARDvalue * 0.75)
			break;
		if (source.at<ushort>(bottom + 0, col) < STANDARDvalue * 0.75 &&
			source.at<ushort>(bottom + 1, col) < STANDARDvalue * 0.75 &&
			source.at<ushort>(bottom + 2, col) > STANDARDvalue * 0.75 &&
			source.at<ushort>(bottom + 3, col) < STANDARDvalue * 0.75)
			break;
	}

	int height = bottom - top + 1;
	if (height > 3)
		return pair<int, int>(top + 1, bottom - 1); // 去除可能的边界噪声点
	else
	{
		if (height == 1)
			return pair<int, int>(top - 1, bottom + 1);
		if (height == 2)
			if (source.at<ushort>(top - 1, col) > source.at<ushort>(bottom + 1, col))
				return pair<int, int>(top - 1, bottom);
			else
				return pair<int, int>(top, bottom + 1);
		if (height == 3)
			return pair<int, int>(top, bottom);
	}
}
// 根据中心点(row,col)和峰值STANDARDvalue，向上/下搜索确定条带的垂直范围（高度）

pair<int, int> General::GetBandLength(const cv::Mat& source, int row, int col, int STANDARDvalue,
	int leftLIMIT, int rightLIMIT)
{
	vector<int> testrowVec;
	if (source.at<ushort>(row, col) > STANDARDvalue * 0.75)
		testrowVec.push_back(row);

	int offset = 1;
	while (offset <= 4)
	{
		if (row + offset < source.rows)
			if (source.at<ushort>(row + offset, col) > STANDARDvalue * 0.75)
				testrowVec.push_back(row + offset);
		if (row - offset >= 0)
			if (source.at<ushort>(row - offset, col) > STANDARDvalue * 0.75 && offset <= 2)
				testrowVec.push_back(row - offset);
		if (testrowVec.size() == 3)
			break;
		if (testrowVec.size() > 3)
		{
			testrowVec.pop_back();
			break;
		}
		offset++;
	}

	if (testrowVec.size() == 0)
		return pair<int, int>(leftLIMIT, rightLIMIT);

	int left, right, RESULTleft = 65535, RESULTright = 0;
	for (int i = 0; i < testrowVec.size(); i++)
	{
		for (left = col; left >= leftLIMIT - 1; left--) // -1 保证至少包含一个有效点
		{
			if (source.at<ushort>(testrowVec[i], left - 0) <= STANDARDvalue * 0.25)
				break;
			if (source.at<ushort>(testrowVec[i], left - 0) <= STANDARDvalue * 0.4 &&
				source.at<ushort>(testrowVec[i], left - 1) <= STANDARDvalue * 0.4)
				break;
			if (source.at<ushort>(testrowVec[i], left - 0) <= STANDARDvalue * 0.5 &&
				source.at<ushort>(testrowVec[i], left - 1) <= STANDARDvalue * 0.5 &&
				source.at<ushort>(testrowVec[i], left - 2) <= STANDARDvalue * 0.5)
				break;
			if (source.at<ushort>(testrowVec[i], left - 0) <= STANDARDvalue * 0.5 &&
				source.at<ushort>(testrowVec[i], left - 1) <= STANDARDvalue * 0.5 &&
				source.at<ushort>(testrowVec[i], left - 2) >= STANDARDvalue * 0.5 &&
				source.at<ushort>(testrowVec[i], left - 3) <= STANDARDvalue * 0.5)
				break;
		}
		if (left < RESULTleft)
			RESULTleft = left + 1; // 左边界修正

		for (right = col; right <= rightLIMIT + 1; right++) // +1 保证至少包含一个有效点
		{
			if (source.at<ushort>(testrowVec[i], right + 0) <= STANDARDvalue * 0.25)
				break;
			if (source.at<ushort>(testrowVec[i], right + 0) <= STANDARDvalue * 0.4 &&
				source.at<ushort>(testrowVec[i], right + 1) <= STANDARDvalue * 0.4)
				break;
			if (source.at<ushort>(testrowVec[i], right + 0) <= STANDARDvalue * 0.5 &&
				source.at<ushort>(testrowVec[i], right + 1) <= STANDARDvalue * 0.5 &&
				source.at<ushort>(testrowVec[i], right + 2) <= STANDARDvalue * 0.5)
				break;
			if (source.at<ushort>(testrowVec[i], right + 0) <= STANDARDvalue * 0.5 &&
				source.at<ushort>(testrowVec[i], right + 1) <= STANDARDvalue * 0.5 &&
				source.at<ushort>(testrowVec[i], right + 2) >= STANDARDvalue * 0.5 &&
				source.at<ushort>(testrowVec[i], right + 3) <= STANDARDvalue * 0.5)
				break;
		}
		if (right > RESULTright)
			RESULTright = right - 1; // 右边界修正
	}

	return pair<int, int>(RESULTleft, RESULTright);
}
// 根据中心点(row,col)和峰值STANDARDvalue，在多行上向左/右搜索确定条带的水平范围（长度）

vector<int> General::GetPointDifferenceVec(const cv::Mat& source, int distance)
{
	vector<int> DifferenceVec;
	for (int r = 5; r < source.rows - 5; r++)
		DifferenceVec.push_back(source.at<ushort>(r, 0) - source.at<ushort>(r - distance, 0));

	return DifferenceVec;
}
// 计算单列图像中，相距distance行的相邻像素灰度差值序列

PointInfoTYPE General::GetMaxPointInfo(const cv::Mat& source)
{
	double maxvalue = 0;
	double* maxvalueptr = &maxvalue;
	cv::Point maxpoint(0, 0);
	cv::Point* maxpointptr = &maxpoint;

	cv::minMaxLoc(source, nullptr, maxvalueptr, nullptr, maxpointptr);

	int value = *maxvalueptr;
	int MAXcol = maxpointptr->x;
	int MAXrow = maxpointptr->y;

	return make_tuple(pair<int, int>(MAXcol, MAXrow), value);
}
// 获取图像中灰度最大值点的坐标和灰度值