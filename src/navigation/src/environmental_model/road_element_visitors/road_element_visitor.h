#ifndef ROAD_ELEMENT_VISITOR_H
#define ROAD_ELEMENT_VISITOR_H
#include "../road_elements/road_closure.h"
#include "../road_elements/unidentified.h"
#include "../road_elements/obstacle.h"
#include "../road_elements/junction.h"
#include "../road_elements/speed_limit.h"
#include "../road_elements/arrow.h"
#include "../road_elements/start_line.h"

namespace environmental_model {
class RoadElementVisitor {
 public:
  virtual ~RoadElementVisitor() = default;
  virtual void visit(Unidentified& /*road_object*/, TrackingElement& /*tracking_element*/) {}
  virtual void visit(Obstacle& /*road_object*/, TrackingElement& /*tracking_element*/) {}
  virtual void visit(Junction& /*road_object*/, TrackingElement& /*tracking_element*/) {}
  virtual void visit(Crosswalk& /*road_object*/, TrackingElement& /*tracking_element*/) {}
  virtual void visit(RoadClosure& /*road_object*/, TrackingElement& /*tracking_element*/) {}
  virtual void visit(SpeedLimit& /*road_object*/, TrackingElement& /*tracking_element*/) {}
  virtual void visit(Arrow& /*road_object*/, TrackingElement& /*tracking_element*/) {}
  virtual void visit(StartLine& /*road_object*/, TrackingElement& /*tracking_element*/) {}
};
namespace visitor_helper {
template <class Visitor>
void visitElements(Visitor& visitor, std::vector<TrackingElement>& tracking_elements) {
  for (auto& tracking_element : tracking_elements) {
    if (tracking_element.getRoadElement()) {
      tracking_element.getRoadElement()->accept(visitor, tracking_element);
    }
  }
}
}  // namespace visitor_helper
}  // namespace environmental_model

#endif  // ROAD_ELEMENT_VISITOR_H
