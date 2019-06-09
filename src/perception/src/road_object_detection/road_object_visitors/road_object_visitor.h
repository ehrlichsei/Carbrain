#ifndef ROAD_OBJECT_VISITOR_H
#define ROAD_OBJECT_VISITOR_H

#include "../road_objects/road_object.h"

namespace road_object_detection {

class RoadObjectVisitor {
 public:
  virtual ~RoadObjectVisitor() = default;
  virtual void visit(Unidentified& road_object) = 0;
  virtual void visit(Obstacle& road_object) = 0;
  virtual void visit(Junction& road_object) = 0;
  virtual void visit(Crosswalk& road_object) = 0;
  virtual void visit(RoadClosure& road_object) = 0;
  virtual void visit(SpeedLimitMarking& road_object) = 0;
  virtual void visit(ArrowMarking& road_object) = 0;
  virtual void visit(StartLine& road_object) = 0;
  virtual void visit(Pedestrian& road_object) = 0;
  virtual void visit(NoPassingZone& road_object) = 0;


  void visit(RoadObjects& road_objects) {
    for (auto& road_object : road_objects) {
      road_object->accept(*this);
    }
  }
};


}  // namespace road_object_detection

#endif  // ROAD_OBJECT_VISITOR_H
