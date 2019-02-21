#include "pointseg.h"
#include <vector>

class ObjectSeg
{
    public:
        ObjectSeg();
        ObjectSeg(int id);
        ObjectSeg(int id, int pointsLength, std::vector<PointSeg*> points);
        int id;
        int pointsLength;
        std::vector<PointSeg*> points; 
};