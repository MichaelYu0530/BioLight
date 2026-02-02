#include "ClassHead.h"

Band::Band(int top, int bottom, int left, int right, int value)
{
	this->top = top;
	this->bottom = bottom;
	this->left = left;
	this->right = right;
	this->midmaxvalue = value;
	this->slope = 0;
}

Band& Band::operator = (const Band& rhs)
{
	this->top = rhs.top;
	this->bottom = rhs.bottom;
	this->left = rhs.left;
	this->right = rhs.right;
	this->midmaxvalue = rhs.midmaxvalue;
	this->slope = rhs.slope;

	return *this;
}