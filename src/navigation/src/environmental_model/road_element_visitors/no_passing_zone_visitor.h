#ifndef NO_PASSING_ZONE_VISITOR_H
#define NO_PASSING_ZONE_VISITOR_H

#include <common/macros.h>
#include "road_element_visitor.h"
#include "navigation/no_passing_zone.h"
THIRD_PARTY_HEADERS_BEGIN
THIRD_PARTY_HEADERS_END

namespace environmental_model {
class NoPassingZoneVisitor : public RoadElementVisitor {

 public:
  NoPassingZoneVisitor() = default;

  // RoadElementVisitor interface
 public:
  void visit(Unidentified &road_object, TrackingElement &trackinge_element) override;
  void visit(Obstacle &road_object, TrackingElement &trackinge_element) override;
  void visit(Junction &road_object, TrackingElement &trackinge_element) override;
  void visit(Crosswalk &road_object, TrackingElement &trackinge_element) override;
  void visit(RoadClosure &road_object, TrackingElement &trackinge_element) override;
  void visit(SpeedLimit &road_object, TrackingElement &trackinge_element) override;
  void visit(Arrow &road_object, TrackingElement &trackinge_element) override;
  void visit(StartLine &road_object, TrackingElement &trackinge_element) override;

  std::vector<NoPassingZone> getNoPassingZones() const;

 private:
  std::vector<NoPassingZone> no_passing_zones;
};
}  // namespace environmental_model

#endif  // NO_PASSING_ZONE_VISITOR_H
