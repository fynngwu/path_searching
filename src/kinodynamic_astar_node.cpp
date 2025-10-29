#include <rclcpp/rclcpp.hpp>
#include <path_searching/kinodynamic_astar.h>
#include <plan_env/edt_environment.h>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/path.hpp>

class SimpleSDFMap : public plan_env::SDFMap {
public:
  SimpleSDFMap() {
    resolution_ = 0.1;
    origin_ = Eigen::Vector3d(-10, -10, -1);
    map_size_ = Eigen::Vector3d(20, 20, 2);
  }

  double getDistance(const Eigen::Vector3d& pos) override {
    // Simple obstacle: avoid center area
    double dist_to_center = (pos - Eigen::Vector3d(0, 0, 0)).norm();
    if (dist_to_center < 2.0) {
      return -1.0; // occupied
    }
    return 1.0; // free
  }

  double getResolution() override {
    return resolution_;
  }

  void getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector3d pts[2][2][2], Eigen::Vector3d& diff) override {
    // Simple implementation - return the same point for all surrounding points
    for (int x = 0; x < 2; x++) {
      for (int y = 0; y < 2; y++) {
        for (int z = 0; z < 2; z++) {
          pts[x][y][z] = pos;
        }
      }
    }
    diff = Eigen::Vector3d::Zero();
  }

  void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) override {
    ori = origin_;
    size = map_size_;
  }

private:
  double resolution_;
  Eigen::Vector3d origin_;
  Eigen::Vector3d map_size_;
};

class SimpleEnvironment : public plan_env::EDTEnvironment {
public:
  SimpleEnvironment() {
    sdf_map_ = std::make_shared<SimpleSDFMap>();
    init();
  }
};

class KinodynamicAstarNode : public rclcpp::Node {
public:
  KinodynamicAstarNode() : Node("kinodynamic_astar_node") {
    // Initialize the kinodynamic astar
    kino_astar_ = std::make_shared<path_searching::KinodynamicAstar>();
    
    // Set parameters
    this->declare_parameter("search.max_tau", 0.6);
    this->declare_parameter("search.init_max_tau", 0.8);
    this->declare_parameter("search.max_vel", 3.0);
    this->declare_parameter("search.max_acc", 2.0);
    this->declare_parameter("search.w_time", 10.0);
    this->declare_parameter("search.horizon", 7.0);
    this->declare_parameter("search.resolution_astar", 0.1);
    this->declare_parameter("search.time_resolution", 0.8);
    this->declare_parameter("search.lambda_heu", 5.0);
    this->declare_parameter("search.allocate_num", 100000);
    this->declare_parameter("search.check_num", 5);
    this->declare_parameter("search.optimistic", true);

    // Set environment
    auto env = std::make_shared<SimpleEnvironment>();
    kino_astar_->setEnvironment(env);
    kino_astar_->setParam(this->shared_from_this());
    kino_astar_->init();

    // Create publishers
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("kino_path", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("path_marker", 10);

    // Test the algorithm
    testSearch();
  }

private:
  void testSearch() {
    RCLCPP_INFO(this->get_logger(), "Testing Kinodynamic A* search...");

    // Define start and goal
    Eigen::Vector3d start_pt(5, 5, 0);
    Eigen::Vector3d start_vel(0, 0, 0);
    Eigen::Vector3d start_acc(0, 0, 0);
    Eigen::Vector3d end_pt(-5, -5, 0);
    Eigen::Vector3d end_vel(0, 0, 0);

    // Perform search
    int status = kino_astar_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true);

    if (status == path_searching::KinodynamicAstar::REACH_END) {
      RCLCPP_INFO(this->get_logger(), "Path found successfully!");
      
      // Get trajectory
      auto traj = kino_astar_->getKinoTraj(0.1);
      
      // Publish path
      publishPath(traj);
      
      RCLCPP_INFO(this->get_logger(), "Trajectory has %zu points", traj.size());
    } else {
      RCLCPP_WARN(this->get_logger(), "No path found. Status: %d", status);
    }
  }

  void publishPath(const std::vector<Eigen::Vector3d>& traj) {
    // Create nav_msgs::Path
    auto path_msg = nav_msgs::msg::Path();
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";

    for (const auto& pt : traj) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = pt.x();
      pose.pose.position.y = pt.y();
      pose.pose.position.z = pt.z();
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);

    // Create visualization marker
    auto marker = visualization_msgs::msg::Marker();
    marker.header = path_msg.header;
    marker.ns = "kino_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    for (const auto& pt : traj) {
      geometry_msgs::msg::Point point;
      point.x = pt.x();
      point.y = pt.y();
      point.z = pt.z();
      marker.points.push_back(point);
    }

    marker_pub_->publish(marker);
  }

  std::shared_ptr<path_searching::KinodynamicAstar> kino_astar_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KinodynamicAstarNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
