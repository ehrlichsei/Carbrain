#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <fstream>
THIRD_PARTY_HEADERS_END

#include "../../common/include/common/test/dummy_parameter_handler.h"

#include "../src/utils/dbscan_clusterer.h"  //class getting testet

DISABLE_SUGGEST_OVERRIDE_WARNING

using namespace std;

namespace {

struct input_point {
  VehiclePoint point;
  unsigned int cluster;
};

const ParameterString<double> DBSCAN_CORE_EPSILON("dbscan/core_epsilon");
const ParameterString<int> DBSCAN_MIN_POINTS_TO_BE_CORE(
    "dbscan/min_points_to_be_core");

class DBScanClustererTest : public ::testing::Test {

 protected:
  unique_ptr<DBScanClusterer> testee;
  ImagePoints feature_points_image;
  VehiclePoints feature_points_vehicle;
  FeaturePointCluster input_cluster;
  // input with correct cluster for each point
  vector<input_point> input_points;
  DummyParameterHandler param;

  DBScanClustererTest()
      : input_cluster(feature_points_image, feature_points_vehicle) {

    param.addParam(DBSCAN_CORE_EPSILON, 200.0);
    param.addParam(DBSCAN_MIN_POINTS_TO_BE_CORE, 4);

    testee = make_unique<DBScanClusterer>(&param);

    const string input_file_path = "spiral.txt";
    ifstream source(input_file_path, ifstream::in);

    if (!source.is_open()) {
      cerr << "Error, could not open input file.";
      exit(1);
    }

    string current_line;
    vector<string> current_line_split;
    int current_point_x, current_point_y;
    VehiclePoint current_point;
    input_point current_input_point;

    while (getline(source, current_line)) {
      current_line_split = split(current_line, '\t');
      if (current_line_split.size() != 3) {
        cerr << "Error, parsed a line with incorrect number of entries. "
                "Expecting 3.";
        exit(1);
      }

      current_point_x = int(stod(current_line_split.at(0)) * 100);
      current_point_y = int(stod(current_line_split.at(1)) * 100);

      current_point = VehiclePoint(current_point_x, current_point_y, 0);
      input_cluster.feature_points_vehicle.push_back(current_point);

      input_cluster.feature_points_img.push_back(ImagePoint(0, 0));

      current_input_point = {current_point, unsigned(stoi(current_line_split.at(2)))};
      input_points.push_back(current_input_point);
    }

    // For debugging:
    // printVehiclePoints(input_cluster);

    if (source.bad()) {
      cerr << "Error, IO error while reading input file.";
    }

    source.close();
  }

 private:
  void splitRec(const std::string &s, char delim, vector<string> &result) {
    stringstream ss;
    ss.str(s);
    string item;
    while (getline(ss, item, delim)) {
      result.push_back(item);
    }
  }

  vector<string> split(const string &s, char delim) {
    vector<string> elems;
    splitRec(s, delim, elems);
    return elems;
  }

 public:
  // for debugging
  /* static void printVehiclePoints(const FeaturePointCluster &cluster) {
    cout << "Printing cluster " << cluster.id << ":" << endl;
    VehiclePoint current_point;
    for (size_t p = 0; p < cluster.feature_points_vehicle.size(); p++) {
      current_point = cluster.feature_points_vehicle[p];
      cout << "X:" << current_point[0] << " Y:" << current_point[1]
           << " Z:" << current_point[2] << endl;
    }
  }*/

  bool clusterContainsPoint(FeaturePointCluster cluster, VehiclePoint point) {
    VehiclePoints vehicle_points = cluster.feature_points_vehicle;
    VehiclePoint current_point;
    for (size_t i = 0; i < vehicle_points.size(); i++) {
      current_point = vehicle_points.at(i);
      if (current_point[0] == point[0] && current_point[1] == point[1] &&
          current_point[2] == point[2]) {  // match
        // Note: Here its ok to compare doubles as we only fill them with ints.
        return true;
      }
    }
    return false;
  }

  // for debugging
  /*void printClusterMap(std::map<unsigned int, size_t> map) {
    std::map<unsigned int, size_t>::iterator iterator;
    iterator = map.begin();
    cout << ">>> Printing cluster map (input cluster -> output cluster):" <<
  endl;
    while (iterator != map.end()) {
      cout << iterator->first << " -> " << iterator->second << endl;
      iterator++;
    }
    cout << ">>> End of cluster map. " << endl;
  }*/
};
}

TEST_F(DBScanClustererTest, StandardTest) {
  std::vector<FeaturePointCluster> output = testee->cluster(input_cluster);

  // For debugging:
  /* for (size_t c = 0; c < output.size(); c++) {
    printVehiclePoints(output[c]);
  }*/

  // Check result: Each input point must be in a cluster and in the correct one.
  // The input and outut clusters may have different ids.
  // The input cluster is identified by the id given in the input file.
  // The output cluster is defined by its index in 'output', NOT by its member
  // 'id'.

  std::map<unsigned int, size_t> cluster_map;
  std::map<unsigned int, size_t>::iterator map_iterator;
  input_point input_point;

  bool found_point;
  bool found_all_points = true;

  for (size_t i = 0; i < input_points.size(); i++) {  // Check each input point.
    // For debugging:
    // printClusterMap(cluster_map);

    found_point = false;
    input_point = input_points.at(i);
    map_iterator = cluster_map.find(input_point.cluster);

    if (map_iterator ==
        cluster_map
            .end()) {  // No mapping there yet, search through ALL clusters.
      // For debugging:
      // cout << "No mapping for: x:" << input_point.point[0] << " y: " <<
      //  input_point.point[1] << " z: " << input_point.point[2] << endl;

      for (size_t j = 0; j < output.size(); j++) {
        if (clusterContainsPoint(output.at(j), input_point.point)) {  // Match
          cluster_map[input_point.cluster] = j;  // Add cluster mapping to map.
          found_point = true;
          break;
        }
      }
    } else {  // There is an existing mapping. Search only through the
              // corresponding cluster.
      // For debugging:
      // cout << "Mapping for: x:" << input_point.point[0] << " y: " <<
      //   input_point.point[1] << " z: " << input_point.point[2] <<
      //   " output cluster: " << map_iterator->second << endl;

      if (clusterContainsPoint(output.at(map_iterator->second), input_point.point)) {  // Match
        found_point = true;
      }
    }

    if (found_point == false) {
      found_all_points = false;
      break;
    }
  }

  EXPECT_TRUE(found_all_points);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
