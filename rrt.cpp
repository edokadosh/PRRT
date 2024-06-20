#include "rrt.h"

void RRT::RRTiteration() {
	Point newPoint, nearestPoint, freePoint;
	int nearestIndex = 0; 

	do {
		newPoint = this->s.randomPoint(); 

		// Find nearest point to the newPoint such that the next node 
		// be added in graph in the (nearestPoint, newPoint) while being obstacle free
		nearestPoint = *nodes.begin(); nearestIndex = 0;
		for(int i = 0; i < nodes.size(); i++) {
			auto pnt = nodes[i] ; 

			if((pnt.distance(newPoint) - nearestPoint.distance(newPoint)) <= EPS)
				nearestPoint = pnt, nearestIndex = i ; 
		}
		freePoint = nearestPoint;
		auto step = (newPoint-nearestPoint).normalize() * STEP_SIZE;

		while(freePoint.distance(newPoint) > STEP_SIZE and s.isEdgeObstacleFree(nearestPoint, freePoint+step)) {
			freePoint+=step;
		}
	} while(!(nearestPoint == freePoint) and (!this->s.isEdgeObstacleFree(nearestPoint, freePoint)));

	nodes.push_back(freePoint);
	parent.push_back(nearestIndex);
	cost.push_back(cost[nearestIndex] + distance(nearestPoint, freePoint));

	// check if destination reaced
	if (s.isEdgeObstacleFree(freePoint, s.end)) {
		destinationReached = true;
		nodes.push_back(s.end);
		parent.push_back(nodes.size()-2);
		cost.push_back(cost[nodes.size()-2] + distance(s.end, freePoint));
	}
}

bool RRT::getDestinationReached() {
	return destinationReached;
}

void RRT::solve() {
	int it = 0;

    while (!getDestinationReached()) {
        RRTiteration();
        it++;
		if (it > 1e6) {
    		cout << "Solver stops after " << it << " iterations" << endl;
			return;
		}
    }

    cout << "Solved!" << endl;
    cout << "Solver took " << it << " iterations" << endl;
	cout << nodes.size() << " nodes" << endl;
}