#pragma once

#include <iostream>
#include "CImg.h"
#include "geometry.h"

using namespace std;
using namespace cimg_library;

#define pix pair<int, int>

#define IMAGE_W 2400
#define IMAGE_H 2400
#define GRID 0

static const float RED[] = {1.0f,0.0f,0.0f};
static const float GREEN[] = {0.0f,1.0f,0.0f};
static const float BLUE[] = {0.0f,0.0f,1.0f};
static const float PURPLE[] = {1.0f,0.0f,1.0f};


class Draw {
    CImg<float> img;
    int h, w;
    int xMin, xMax, yMin, yMax;

public:
    Draw(int _h, int _w, _ftype _xMin, _ftype _xMax, _ftype _yMin, _ftype _yMax);
    pix pointToPix(Point p);
    void drawPolygon(Polygon poly, const float* color);
    void drawPoint(Point p, const float* color);
    void drawEdge(Point a, Point b, const float* color);
    void save(string path);
};