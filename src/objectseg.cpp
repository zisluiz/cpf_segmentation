#include <segmentation/objectseg.h>

ObjectSeg::ObjectSeg() {
}

ObjectSeg::ObjectSeg(int id)
{
    this->id = id;
}

ObjectSeg::ObjectSeg(int id, int pointsLength, std::vector<PointSeg*> points)
{
    this->id = id;
    this->pointsLength = pointsLength;
    this->points = points;
}