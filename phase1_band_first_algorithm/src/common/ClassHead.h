#ifndef CLASS_HEAD_H

#define CLASS_HEAD_H

#include <opencv2/opencv.hpp>

using namespace std;

enum class Direction
{
	Left_To_Right,
	Right_To_Left,
	Top_To_Bottom,
	Bottom_To_Top
};

class Band
{
public:
	int top;
	int bottom;
	int left;
	int right;

	int midmaxvalue;

	float slope;
	float top_line_offset;
	float bottom_line_offset;

	vector<cv::Point> topborders;
	vector<cv::Point> bottomborders;

	Band(int top, int bottom, int left, int right, int value);

	Band& operator = (const Band& rhs);
};

#endif
