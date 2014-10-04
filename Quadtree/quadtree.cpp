#include "quadtree.h"

Quadtree::Quadtree(int _x, int _y, int _width, int _height,int minimal_area):
    x		(_x),
    y		(_y),
    width	(_width),
    height	(_height),
    minArea (minimal_area),
    visited (false)
{
    hasChildren=false;
    hasPoint=false;

    boundry=cv::Rect(cv::Point(x,y),cv::Point(x+width,y+height));
    area=boundry.area();
}

bool Quadtree::insert(cv::Point2f p)
{
    if(!(hasChildren||hasPoint)){
        point=p;
        hasPoint=true;
        return true;
    }

    if(!hasChildren){
        if((area/4)<minArea)
            return false;
        else{
            subdivide();
        }
    }

    if (northWest->contains(p)){
        northWest->insert(p);
        return true;
    }
    if (northEast->contains(p)){
        northEast->insert(p);
        return true;
    }
    if (southWest->contains(p)){
        southWest->insert(p);
        return true;
    }
    if (southEast->contains(p)){
        southEast->insert(p);
        return true;
    }

    return false;
}

bool Quadtree::subdivide()
{
    hasChildren=true;

    northWest=new Quadtree(x,y,width/2,height/2,minArea);
    northEast=new Quadtree(x+width/2,y,width/2,height/2,minArea);
    southWest=new Quadtree(x,y+height/2,width/2,height-height/2,minArea);
    southEast=new Quadtree(x+width/2,y+height/2,width/2,height-height/2,minArea);

    if(hasPoint){
        if (northWest->contains(point)){
            northWest->insert(point);
            return true;
        }
        if (northEast->contains(point)){
            northEast->insert(point);
            return true;
        }
        if (southWest->contains(point)){
            southWest->insert(point);
            return true;
        }
        if (southEast->contains(point)){
            southEast->insert(point);
            return true;
        }
    }

    return false;
}

void Quadtree::draw(cv::Mat &img)
{
    cv::rectangle(img,boundry,cv::Scalar(255,0,0));
    if(hasChildren)
    {
        northWest->draw(img);
        northEast->draw(img);
        southWest->draw(img);
        southEast->draw(img);
    }
}

bool Quadtree::contains(cv::Point2f p)
{
    if(boundry.contains(p))
        return true;
    else
        return false;
}

Quadtree::~Quadtree()
{
    if (!hasChildren)
        return;

    delete northWest;
    delete northEast;
    delete southWest;
    delete southEast;
}

bool Quadtree::isLeaf()
{
    if(hasChildren)
        return false;
    else
        return true;
}

bool Quadtree::containsPoint()
{
    if(hasPoint)
        return true;
    else
        return false;
}

cv::Point2f Quadtree::getPoint()
{
    return point;
}

Quadtree* Quadtree::getChidren(int i)
{
    switch(i){
    case 0:{
        return northWest;
        break;}
    case 1:{
        return northEast;
        break;}
    case 2:{
        return southEast;
        break;}
    case 3:{
        return southWest;
        break;}
    default:
        return northWest;
    }
}

void Quadtree::resetVisited()
{
    std::vector<Quadtree*> nodes;
    nodes.push_back(this);
    while(nodes.size()>0)
    {
        Quadtree* node=nodes.back();
        nodes.pop_back();
        if(node->isLeaf()){
            node->visited=false;
        }
        else{
            nodes.push_back(node->getChidren(0));
            nodes.push_back(node->getChidren(1));
            nodes.push_back(node->getChidren(2));
            nodes.push_back(node->getChidren(3));
        }
    }
}


void Quadtree::EDfilter(cv::Mat& source_img, cv::Mat& dest_img, std::vector<cv::KeyPoint> &keypoint_arr,
		Quadtree* tree_ptr, int nrOfPoints) {

	std::deque<Quadtree*> cur_level;
	std::deque<std::deque<Quadtree*> > map;
	cur_level.push_back(tree_ptr);
	map.push_back(cur_level);
	int z = 0;
	do {
		int size = map.front().size();
		LCG gen(rand() % size, size);
		for (int x = 0; x < size; x++) {
			int it = gen.NextInt();
			Quadtree* node = map.front()[it];

			if (node->isLeaf()) {
				if (node->containsPoint() && (!node->visited)) {
					node->visited = true;
					//keypoint_arr.push_back(KeyPoint(node->getPoint(),3));
				}
			} else {
				if (map.size() == 1) {
					std::deque<Quadtree*> tmp;
					tmp.push_back(node->getChidren(0));
					tmp.push_back(node->getChidren(1));
					tmp.push_back(node->getChidren(2));
					tmp.push_back(node->getChidren(3));
					map.push_back(tmp);
				} else {
					map[1].push_back(node->getChidren(0));
					map[1].push_back(node->getChidren(1));
					map[1].push_back(node->getChidren(2));
					map[1].push_back(node->getChidren(3));
				}

				std::deque<Quadtree*> stack;
				push_random_order(node, stack);
				while (stack.size() > 0) {
					Quadtree* dfs_node = stack.front();
					stack.pop_front();
					if (dfs_node->isLeaf()) {
						if (dfs_node->containsPoint() && (!dfs_node->visited)) {
							dfs_node->visited = true;
							keypoint_arr.push_back(
									cv::KeyPoint(dfs_node->getPoint(), 3));
							//map.clear();
							z++;
							break;
						}
					} else {
						push_random_order(dfs_node, stack);
					}
				}
			}
		}
		if (!map.empty()) {
			map.front().clear();
			map.pop_front();
		}
	} while (!map.empty() && z < nrOfPoints);

	tree_ptr->resetVisited();
	// drawKeypoints( source_img, keypoint_arr, dest_img, Scalar(0,0,255), Drawcv::MatchesFlags::DEFAULT );
}

void Quadtree::push_random_order(Quadtree* node, std::deque<Quadtree*>& vec) {
	int sequence[24][4] = { { 0, 1, 2, 3 }, { 0, 1, 3, 2 }, { 0, 2, 3, 1 }, { 0,
			2, 1, 3 }, { 0, 3, 1, 2 }, { 0, 3, 2, 1 }, { 1, 0, 2, 3 }, { 1, 0,
			3, 2 }, { 1, 2, 3, 0 }, { 1, 2, 0, 3 }, { 1, 3, 0, 2 },
			{ 1, 3, 2, 0 }, { 2, 1, 0, 3 }, { 2, 1, 3, 0 }, { 2, 0, 3, 1 }, { 2,
					0, 1, 3 }, { 2, 3, 1, 0 }, { 2, 3, 0, 1 }, { 3, 1, 2, 0 }, {
					3, 1, 0, 2 }, { 3, 2, 0, 1 }, { 3, 2, 1, 0 },
			{ 3, 0, 1, 2 }, { 3, 0, 2, 1 } };

	int it = rand() % 24;
	for (int i = 0; i < 4; i++) {
		vec.push_back(node->getChidren(sequence[it][i]));
	}
}

