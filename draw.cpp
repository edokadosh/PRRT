#include "draw.h"

Draw::Draw(int _h, int _w, _ftype _xMin, _ftype _xMax, _ftype _yMin, _ftype _yMax) : h(_h), w(_w), xMin(_xMin), xMax(_xMax), yMin(_yMin), yMax(_yMax) {
    img = CImg<float>(_h,_w, 1, 3, 0);
}

pix Draw::pointToPix(Point p) {
    return pix(w*(p.x-xMin)/(xMax-xMin), h*(p.y-yMin)/(yMax-yMin));
}

void Draw::drawPolygon(Polygon poly, const float* color) {
    CImg<int> points(poly.pointCnt,2);

    for (int i = 0 ; i < poly.pointCnt ; i++) {
        pix pointPix = pointToPix(poly.points[i]);
        points(i,0) = pointPix.first;
        points(i,1) = pointPix.second;
    }

    img.draw_polygon(points, color);
}

void Draw::drawPoint(Point p, const float* color) {
    pix pointPix = pointToPix(p);
    img.draw_circle(pointPix.first, pointPix.second, 5, color);
}

void Draw::drawEdge(Point a, Point b, const float* color) {
    pix aPix = pointToPix(a);
    pix bPix = pointToPix(b);

    img.draw_line(aPix.first, aPix.second, bPix.first, bPix.second, color);
}

void Draw::save(string path) {
    img.normalize(0, 255);
    img.save(path.c_str());
}
