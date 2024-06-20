#include "scene.h"


Scene::Scene(std::string path) {
    std::ifstream scene_file(path, std::ifstream::binary);
    Json::Value scene;
    scene_file >> scene;

    this->start = Point(_ftype(scene["robots"][0]["start"][0].asDouble()), _ftype(scene["robots"][0]["start"][1].asDouble()));
    this->end = Point(_ftype(scene["robots"][0]["end"][0].asDouble()), _ftype(scene["robots"][0]["end"][1].asDouble()));

    for (auto obs : scene["obstacles"]) {
        Polygon newObs = Polygon();
        
        for (auto p : obs["poly"]) {
            newObs.addPoint(Point(p[0].asDouble(),p[1].asDouble()));
        }
        // cout << "Loaded obstacle: [";
        // for (auto p : newObs.points) {
        //     cout << "(" << p.x << "," << p.y << "), ";
        // }
        // cout << "]" << endl;
        this->obstacles.push_back(newObs);
    }
}

Point Scene::randomPoint() {
    return Point((((double)rand())/RAND_MAX)*(xMax-xMin)+xMin, (((double)rand()/RAND_MAX))*(yMax-yMin)+yMin);
}

Point Scene::randomFreePoint() {
    Point p;
    
    do {
        p = Point((((double)rand())/RAND_MAX)*(xMax-xMin)+xMin, (((double)rand()/RAND_MAX))*(yMax-yMin)+yMin);
    } while (!isPointFree(p));
    
    return p;
}



void Scene::initCollisionDetection(int xBuckets, int yBuckets) {
    this->xBuckets = xBuckets;
    this->yBuckets = yBuckets;

    this->collisionDetection = vector<vector<vector<Polygon>>>();
    this->xMin = 0;
    this->xMax = 0;
    this->yMin = 0;
    this->yMax = 0;

    for (Polygon obs : this->obstacles) {
        for (Point p : obs.points) {
            this->xMin = this->xMin < p.x ? this->xMin : p.x;
            this->yMin = this->yMin < p.y ? this->yMin : p.y;
            this->xMax = this->xMax > p.x ? this->xMax : p.x;
            this->yMax = this->yMax > p.y ? this->yMax : p.y;
        }
    }
    this->xMax += 2;
    this->xMin -= 2;
    this->yMax += 2;
    this->yMin -= 2;
    std::cout << "Scene bounds set to: X: " << this->xMax << ", " << this->xMin << "   Y: " << this->yMax << ", " << this->yMin << std::endl;

    this->xJump = double(this->xMax - this->xMin) /xBuckets;
    this->yJump = double(this->yMax - this->yMin) /yBuckets;

    for (int i = 0 ; i < xBuckets ; i++) {
        collisionDetection.push_back(vector<vector<Polygon>>());
        for (int j = 0 ; j < xBuckets ; j++) {
            collisionDetection[i].push_back(vector<Polygon>());
        }
    }

    for (Polygon obs : this->obstacles) {
        _ftype polyXMin = obs.points[0].x;
        _ftype polyXMax = obs.points[0].x;
        _ftype polyYMin = obs.points[0].y;
        _ftype polyYMax = obs.points[0].y;

        for (Point p : obs.points) {
            polyXMin = polyXMin < p.x ? polyXMin : p.x;
            polyXMax = polyXMax > p.x ? polyXMax : p.x;
            polyYMin = polyYMin < p.y ? polyYMin : p.y;
            polyYMax = polyYMax > p.y ? polyYMax : p.y;
        }

        for (int i = int((polyXMin - this->xMin) / this->xJump) ; this->xMin+(i*this->xJump) < polyXMax ; i++) {
            for (int j = int((polyYMin - this->yMin) / this->yJump) ; this->yMin+(j*this->yJump) < polyYMax ; j++) {
                collisionDetection[i][j].push_back(obs);
            }
        }
    }
}

bool Scene::isEdgeObstacleFree(Point a, Point b) {
    if (!(isInBounds(a) && isInBounds(b))) {
        printPoint(a);
        printPoint(b);
        cout << endl;
        throw std::invalid_argument("Error: edge out of bounds");
    }

    if (a.x > b.x) {
        Point temp = a;
        a = b;
        b = temp;
    } // a.x < b.x

    for (int i = xToi(a.x); i <= xToi(b.x); i++) {
        _ftype yStart = yOfXOnLine(a,b,max(i*this->xJump + this->xMin, a.x));
        _ftype yEnd = yOfXOnLine(a,b,min((i+1)*this->xJump + this->xMin, b.x));
        if (yStart > yEnd) {
            _ftype temp = yStart;
            yStart = yEnd;
            yEnd = temp;
        } // yStart < yEnd

        for (int j = yToj(yStart); j <= yToj(yEnd) ; j++) {
            for(auto& poly: this->collisionDetection[i][j]) {
                if(lineSegmentIntersectsPolygon(a, b, poly)) {
                    return false; 
                }
            }
        }
    }
    
    return true; 
}

bool Scene::isPointFree(Point p) {
    if (!isInBounds(p)) {
        printPoint(p);
        cout << endl;
        throw std::invalid_argument("Error: point out of bounds");
    }
    int i = xToi(p.x), j = yToj(p.y);
    for (auto& poly: this->collisionDetection[i][j]) {
        if (poly.pointInside(p)) {
            return false;
        }
    }
    return true;
}
