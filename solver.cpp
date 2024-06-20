#include "solver.h"

Solver::Solver(Scene scene, int xBuckets, int yBuckets) : s(scene) {
	s.initCollisionDetection(xBuckets,yBuckets);
}


void Solver::draw(string path) {
	Draw d(IMAGE_W,IMAGE_H, s.xMin, s.xMax, s.yMin, s.yMax);

	cout << "drawing" << endl;
	for (auto obs : s.obstacles) {
    	d.drawPolygon(obs, RED);
	}

	d.drawPoint(s.start, GREEN);
	d.drawPoint(s.end, BLUE);

	for (int i = 0 ; i < nodes.size() ; i++) {
		d.drawEdge(nodes[i], nodes[parent[i]], GREEN);
	}

	if (GRID) {
        for (int i = s.xMin + 1 ; i < s.xMax ; i++) {
            d.drawEdge(Point(i, s.yMin), Point(i, s.yMax), PURPLE);
        }

        for (int i = s.yMin + 1 ; i < s.yMax ; i++) {
            d.drawEdge(Point(s.xMin, i), Point(s.xMax, i), PURPLE);
        }
    }

    cout << "saving to: " << path << endl;

    d.save(path);
}


