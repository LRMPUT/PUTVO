#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "lcg.h"
#include <deque>

#ifndef QUADTREE_H
#define QUADTREE_H

class Quadtree
{
private:
    int x;
    int y;
    int width;
    int height;
    int area;
    int minArea;

    cv::Point2f point;
    bool hasChildren;
    bool hasPoint;
    cv::Rect boundry;

    // Children
    Quadtree* northWest;
    Quadtree* northEast;
    Quadtree* southWest;
    Quadtree* southEast;

    bool subdivide();
    bool contains(cv::Point2f p);
public:
    Quadtree(int _x, int _y, int _width, int _height,int minimal_area);
    ~Quadtree();

    bool insert(cv::Point2f p);
    bool isLeaf();
    bool containsPoint();
    void draw(cv::Mat& img);
    void resetVisited();
    cv::Point2f getPoint();
    Quadtree* getChidren(int i);

    static void EDfilter(cv::Mat& source_img, cv::Mat& dest_img, std::vector<cv::KeyPoint> &keypoint_arr,
    		Quadtree* tree_ptr, int nrOfPoints);
    static void push_random_order(Quadtree* node, std::deque<Quadtree*>& vec);

    bool visited;
};

#endif // QUADTREE_H
