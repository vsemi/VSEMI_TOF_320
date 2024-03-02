#pragma once

#include <stdint.h>
#include <vector>

struct Color {
	uint8_t r, g, b;
	Color() : r(0), g(0), b(0) {}
	Color(uint8_t r_, uint8_t g_, uint8_t b_) : r(r_), g(g_), b(b_) {}
};
class ImageColorizer {
public:
	ImageColorizer();
	Color getColor(int value);
	uint8_t getGrayscale(int value);
	void setRange(int start, int stop);

private:
	std::vector<Color> colorVector;
	int begin;
	int end;
	int numSteps;
	double indexFactorColor;
	double indexFactorBw;

	double interpolate(double x, double x0, double y0, double x1, double y1);
	void createColorMap(int numSteps, int indx, unsigned char &red, unsigned char &green, unsigned char &blue);
};
