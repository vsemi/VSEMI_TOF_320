#include <color.hpp>
#include <iostream>

#define NUM_COLORS    7500

static const int PIXEL_CODE_OFFSET = 48000;

static const int LOW_AMPLITUDE = 16001 + PIXEL_CODE_OFFSET;
static const int ADC_OVERFLOW  = 16002 + PIXEL_CODE_OFFSET;
static const int SATURATION    = 16003 + PIXEL_CODE_OFFSET;
static const int INTERFERENCE  = 16007 + PIXEL_CODE_OFFSET;
static const int EDGE_DETECTED = 16008 + PIXEL_CODE_OFFSET;

ImageColorizer::ImageColorizer()
{
	numSteps = NUM_COLORS;
	unsigned char red, green, blue;

	for (int i = 0; i< numSteps; i++)
	{
		createColorMap(numSteps, i, red, green, blue);
		colorVector.push_back(Color(red, green, blue));
	}
}
uint8_t ImageColorizer::getGrayscale(int value) {
	uint8_t color = ((float)value) * indexFactorBw;
	return color;
}
Color ImageColorizer::getColor(int value)
{
	
	if (value == SATURATION)
	{
		//std::cout << "                SATURATION:   " << value << std::endl;
		return Color(255, 0, 128);
	}
	else if (value == ADC_OVERFLOW)
	{
		//std::cout << "                ADC_OVERFLOW: " << value << std::endl;
		return Color(169, 14, 255);
	}
	else if (value == INTERFERENCE)
	{
		return Color(255, 255, 255);
	}
	else if (value == EDGE_DETECTED)
	{
		return Color(0, 0, 0);
	}
	else if (value == 0)
	{
		//std::cout << "                Zero value: " << value << std::endl;
		//return colorVector.at(0);
		return Color(0, 0, 0);
	}
	else if (value >=65000)
	{
		return Color(255, 0, 128);
	}

	float value_float = float(value) - float(begin);
    if (value_float < 0 || value_float > end)
    {
        return Color(127, 127, 127);
    }

	int i = value * indexFactorColor - 1;
	//std::cout << "                i: " << i << std::endl;
	//std::cout << "         numSteps: " << numSteps << std::endl;
	i = numSteps - i - 1;
	if (i < 0) i = 0;

	//std::cout << "                i: " << i << std::endl;
	//std::cout << "            value: " << value << std::endl;
	//std::cout << "      colorVector: " << colorVector.size() << std::endl;
	//std::cout << " indexFactorColor: " << indexFactorColor << std::endl;

	return colorVector.at(i);
}
void ImageColorizer::setRange(int start, int stop)
{
	//std::cout << "start: " << start << " stop: " << stop << std::endl;
	begin = start;
	end = stop;

    indexFactorColor = float(NUM_COLORS) / (float(stop) - float(start));
    indexFactorBw = float(255) / (float(stop) - float(start));
}
double ImageColorizer::interpolate(double x, double x0, double y0, double x1, double y1)
{
	if (x1 == x0)
	{
		return y0;
	}
	else
	{
		return ((x - x0)*(y1 - y0) / (x1 - x0) + y0);
	}
}
void ImageColorizer::createColorMap(int numSteps, int indx, unsigned char &red, unsigned char &green, unsigned char &blue) {

	/*double B0 = -0.125;
	double B1 = B0 + 0.25;
	double B2 = B1 + 0.25;
	double B3 = B2 + 0.25;

	double G0 = B1;
	double G1 = G0 + 0.25;
	double G2 = G1 + 0.25;
	double G3 = G2 + 0.25;

	double R0 = B2;
	double R1 = R0 + 0.25;
	double R2 = R1 + 0.25;
	double R3 = R2 + 0.25;*/

	double k = 1;
	double B0 = -0.125 * k - 0.25;
	double B1 = B0 + 0.25 * k;
	double B2 = B1 + 0.25 * k;
	double B3 = B2 + 0.25 * k;

	double G0 = B1;
	double G1 = G0 + 0.25 * k;
	double G2 = G1 + 0.25 * k;
	double G3 = G2 + 0.25 * k + 0.125;

	double R0 = B2;
	double R1 = R0 + 0.25 * k;
	double R2 = R1 + 0.25 * k;
	double R3 = R2 + 0.25 * k + 0.25;

	double i = (double)indx / (double)numSteps - 0.25 * (double)k;

	if (i >= R0 && i < R1) {
		red = interpolate(i, R0, 0, R1, 255);
	}
	else if ((i >= R1) && (i < R2)) {
		red = 255;
	}
	else if ((i >= R2) && (i < R3)) {
		red = interpolate(i, R2, 255, R3, 0);
	}
	else {
		red = 0;
	}

	if (i >= G0 && i < G1) {
		green = interpolate(i, G0, 0, G1, 255);
	}
	else if ((i >= G1) && (i<G2)) {
		green = 255;
	}
	else if ((i >= G2) && (i < G3)) {
		green = interpolate(i, G2, 255, G3, 0);
	}
	else {
		green = 0;
	}
	
	if (i >= B0 && i < B1) {
		blue = interpolate(i, B0, 0, B1, 255);
	}
	else if ((i >= B1) && (i < B2)) {
		blue = 255;
	}
	else if ((i >= B2) && (i < B3)) {
		blue = interpolate(i, B2, 255, B3, 0);
	}
	else {
		blue = 0;
	}
}
