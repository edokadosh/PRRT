#include <cuda.h>
#include <curand.h>
#include <curand_kernel.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>
#include <iostream>

using namespace std;

#define cuft double 

const cuft _EPS = 1e-6;

#define STEP_SIZE 0.2

#define NUM_NODES_TO_ADD 1000
#define MAX_KERNEL_CYCLES 1e11

struct CudaPoint {
    cuft  x, y;
    __host__ __device__
    CudaPoint() {}
    __host__ __device__
    CudaPoint(cuft  x, cuft  y): x(x), y(y) {}
    __host__ __device__
    CudaPoint(const CudaPoint &p) : x(p.x), y(p.y) {}

    __host__ __device__
    CudaPoint& operator+=(const CudaPoint &t) {
        x += t.x;
        y += t.y;
        return *this;
    }
    __host__ __device__
    CudaPoint& operator-=(const CudaPoint &t) {
        x -= t.x;
        y -= t.y;
        return *this;
    }
    __host__ __device__
    CudaPoint& operator*=(cuft  t) {
        x *= t;
        y *= t;
        return *this;
    }
    __host__ __device__
    CudaPoint& operator/=(cuft  t) {
        x /= t;
        y /= t;
        return *this;
    }
    __host__ __device__
    CudaPoint operator+(const CudaPoint &t) const {
        return CudaPoint(*this) += t;
    }
    __host__ __device__
    CudaPoint operator-(const CudaPoint &t) const {
        return CudaPoint(*this) -= t;
    }
    __host__ __device__
    CudaPoint operator*(cuft  t) const {
        return CudaPoint(*this) *= t;
    }
    __host__ __device__
    CudaPoint operator/(cuft  t) const {
        return CudaPoint(*this) /= t;
    }
    __host__ __device__
    cuft dot(const CudaPoint &t) const {
    	return (x*t.x + y*t.y); 
    }
    __host__ __device__
    cuft cross(const CudaPoint& t) const { 
        return x * t.y - y * t.x;
    }
    __host__ __device__
    cuft cross(const CudaPoint& a, const CudaPoint& b) const {
        return (a - *this).cross(b - *this); 
    }
    __host__ __device__
    cuft distance(const CudaPoint &t) const {
    	const double x_diff = x - t.x, y_diff = y - t.y ; 
    	return sqrt(x_diff * x_diff + y_diff * y_diff);
    }
    __host__ __device__
    CudaPoint steer(const CudaPoint& t, cuft  DELTA) {
	    if(this->distance(t) < DELTA) {
	        return t; 
	    }
	    else {
	        double theta = atan2(t.y - y, t.x - x);
	        return CudaPoint(x + DELTA * cos(theta), y + DELTA * sin(theta));
	    }
	}
    __host__ __device__
    bool operator==(const CudaPoint& rhs) const
    {
        return fabs(x - rhs.x) < _EPS and fabs(y - rhs.y) < _EPS ; // or another approach as above
    }
    __host__ __device__
    CudaPoint normalize() 
    {
        return CudaPoint(*this) / CudaPoint(0,0).distance(*this);
    }
};

__host__ __device__
inline CudaPoint operator*(cuft a, CudaPoint b) {
    return b * a;
}

__host__ __device__
inline cuft distance(CudaPoint& a, CudaPoint &b) {
	const cuft x_diff = a.x - b.x, y_diff = a.y - b.y ; 
	return sqrt(x_diff * x_diff + y_diff * y_diff);
}

__host__ __device__
inline cuft dot(CudaPoint a, CudaPoint b) {
	return (a.x*b.x + a.y*b.y);
}

__host__ __device__
inline cuft cross(CudaPoint a, CudaPoint b) {
    return (a.x*b.y - b.x*a.y); 
}

struct CudaPolygon {
	CudaPoint points[4];  // Assumes clockwise/anti-clockwise points input, maximum of 4 vertices
    int pointCnt = 0 ; 

    __host__ __device__
	CudaPolygon() {
		pointCnt = 0;
	}
    __host__ __device__
	void addPoint(const CudaPoint pnt) {
		points[pointCnt] = pnt; 
        pointCnt++ ;
	}
    __host__ __device__
	bool pointInside(const CudaPoint point) {
	  int i, j, nvert = pointCnt;
	  bool c = false;
	  for(i = 0, j = nvert - 1; i < nvert; j = i++) {
	    if( ( (points[i].y >= point.y ) != (points[j].y >= point.y) ) &&
	        (point.x <= (points[j].x - points[i].x) * (point.y - points[i].y) / (points[j].y - points[i].y) + points[i].x)
	      )
	      c = !c;
	  }
	  return c;
	}
};

__host__ __device__
inline int yOfXOnLine(CudaPoint a, CudaPoint b, cuft x) {
    return a.y + (x-a.x)*((b.y-a.y)/(b.x-a.x));
}

__host__ __device__
inline bool checkCollision(CudaPoint lineFrom, CudaPoint lineTo, CudaPoint location, cuft  radius) {
	location += CudaPoint(radius, radius); // Adjust location from top-left corner to center coordinates 
    cuft  ab2, acab, h2;
    CudaPoint ac = location - lineFrom;
    CudaPoint ab = lineTo - lineFrom;
    ab2 = dot(ab, ab); acab = dot(ac, ab);
    cuft  t = acab / ab2;

    if (t < 0) t = 0;
    else if (t > 1) t = 1;

    CudaPoint h = ((ab * t) + lineFrom) - location;
    h2 = dot(h, h); 
    return (h2 <= (radius * radius));
}

__host__ __device__
inline bool PointInPolygon(CudaPoint point, CudaPolygon polygon) {
  CudaPoint *points = polygon.points;
  int i, j, nvert = polygon.pointCnt;
  bool c = false;

  for(i = 0, j = nvert - 1; i < nvert; j = i++) {
    if( ( (points[i].y >= point.y ) != (points[j].y >= point.y) ) &&
        (point.x <= (points[j].x - points[i].x) * (point.y - points[i].y) / (points[j].y - points[i].y) + points[i].x)
      )
      c = !c;
  }
  return c;
}

__host__ __device__
inline int _sign(const cuft  x) { 
    return x >= 0 ? x ? 1 : 0 : -1; 
}

__host__ __device__
inline bool _intersectOnLine(cuft  a, cuft  b, cuft  c, cuft  d) {
    if ((a - b) > _EPS) {
        cuft temp = a;
        a = b;
        b = temp;
    }
    if ((c - d) > _EPS) {
        cuft temp = c;
        c = d;
        d = temp;
    }
    return fmax(a, c) <= fmin(b, d);
}

__host__ __device__
inline bool check_intersection(const CudaPoint a, const CudaPoint b, const CudaPoint c, const CudaPoint d) {
    // Check if both line segments lie on the same line
    if (c.cross(a, d) == 0 && c.cross(b, d) == 0) {
        return _intersectOnLine(a.x, b.x, c.x, d.x) && _intersectOnLine(a.y, b.y, c.y, d.y);
    }

    // Check if a and b both lie on different side of line segment CD 
    // Similarly check for c and d to lie on different side of line segment AC 
    return _sign(a.cross(b, c)) != _sign(a.cross(b, d)) && _sign(c.cross(d, a)) != _sign(c.cross(d, b));
}

__host__ __device__
inline bool lineSegmentIntersectsPolygon(CudaPoint a, CudaPoint b, CudaPolygon& polygon) {
    // PQ is merely a point not enough distance for it be line segment
    if( a.distance(b) < _EPS ) {
        return PointInPolygon( (a+b)/2.0, polygon); 
    }

    int num = polygon.pointCnt ; 
    CudaPoint *points = polygon.points;
    for(int i = 0; i < num; i++) {
        int nxt = i+1; if(nxt == num) nxt = 0 ;
        if(check_intersection(a, b, points[i], points[nxt])) return true ; 
    }
    return false ; 
}

struct CudaCollisionBucket {
    int* obstacleIndexes;
    int obsCnt;
};

struct CudaNode {
    CudaPoint p;
    int parent;
    cuft  cost;

    CudaNode() : parent(0) {}
    __host__ __device__
    CudaNode(CudaPoint _p, int _parent, cuft  _cost) : p(_p), parent(_parent), cost(_cost) {}
};

struct CudaTree {
    CudaNode nodes[NUM_NODES_TO_ADD];
    int nodeCnt;

    __host__ __device__
    CudaTree() : nodeCnt(0) {}
    __host__ __device__
    void addNode(CudaPoint p, int parent, cuft  cost) {
        nodes[nodeCnt] = CudaNode(p, parent, cost);
        nodeCnt++;
    }
};

struct CudaScene {
    CudaPolygon *obstacles;
    CudaCollisionBucket **collisionDetection;
    cuft  xMin, xMax, yMin, yMax;
    cuft  xJump, yJump;
    CudaPoint start, end;
    curandStateMRG32k3a *randState;

    __host__ __device__
    inline bool isInBounds(CudaPoint a);

    __host__ __device__
    inline int xToi(cuft  x);

    __host__ __device__
    inline int yToj(cuft  y);

    __device__
    CudaPoint randomPoint();

    __host__ __device__
    bool isEdgeObstacleFree(CudaPoint a, CudaPoint b);

};


void runCudaRRT(CudaScene s, CudaTree *re, int numThreads, int blockSizes);

void runInitRandom(curandStateMRG32k3a *state, int seed, int numThreads, int blockSize);

inline void printPoint(CudaPoint p) {
    cout << "(" << p.x << ", " << p.y << ") ";
}