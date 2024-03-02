#include "util.h"

#include <math.h>

void initD3F(double* d3X, double* d3Y, double* d3Z, int width, int height, double angle_x, double angle_y)
{
	int l = width * height;

	double THETA_H = M_PI * angle_x / 180.0f;
	double ALPHA_H = (M_PI - THETA_H) / 2;

	double THETA_V = M_PI * angle_y / 180.0f;
	double ALPHA_V = 2 * M_PI - (THETA_V / 2);

	for (int i = 0; i < l; i ++)
	{
		int x = i % width;
		int y = i / width;

		d3Z[i] = fabs(fabs(0.001 * sin(ALPHA_H + (double)x * (THETA_H / width))) * cos(ALPHA_V + (double)y * (THETA_V / height)));

		d3X[i] = (fabs(fabs(0.001 * sin(ALPHA_H + (double)x * (THETA_H / width))) * cos(ALPHA_V + (double)y * (THETA_V / height))) / tan(ALPHA_H + (double)x * (THETA_H / width)));

		d3Y[i] = -1.0 * fabs(fabs(0.001 * sin(ALPHA_H + (double)x * (THETA_H / width))) * cos(ALPHA_V + (double)y * (THETA_V / height))) * tan(ALPHA_V + (double)y * (THETA_V / height));
	}
}

void cal3DXYZ(int i, int width, int height, unsigned int val, double* d3X, double* d3Y, double* d3Z, float &X, float &Y, float &Z)
{
	Z = static_cast<double>(val) * d3Z[i];

	X = static_cast<double>(val) * d3X[i];

	Y = static_cast<double>(val) * d3Y[i];
}

