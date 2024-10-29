#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <Eigen/Dense>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <chrono>

using namespace std::chrono_literals;

class SplineInterpolationNode : public rclcpp::Node {
public:
    SplineInterpolationNode() : Node("spline_interpolation_node") {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);

        // 20개의 경유점 설정 (마지막 경유점 (0.8, 0.9, 0.5)로 조정)
    std::vector<Eigen::Vector3d> controlPoints = {
        {0.00, 0.00, 0.00}, {0.04, 0.08, 0.04}, {0.08, 0.16, 0.08}, 
        {0.12, 0.24, 0.04},{0.16, 0.28, 0.12}, {0.20, 0.20, 0.10}, 
        {0.24, 0.16, 0.08}, {0.28, 0.12, 0.10},{0.32, 0.08, 0.12}, 
        {0.36, 0.04, 0.14}, {0.40, 0.00, 0.16}, {0.44, 0.04, 0.18},
        {0.48, 0.08, 0.20}, {0.52, 0.12, 0.22}, {0.56, 0.16, 0.24}, 
        {0.60, 0.20, 0.26},{0.64, 0.24, 0.28}, {0.68, 0.28, 0.30}, 
        {0.72, 0.32, 0.32}, {0.80, 0.90, 0.50}
};



        int degree = 3;
        std::vector<double> knots(controlPoints.size() + degree + 1);
        for (size_t i = 0; i <= static_cast<size_t>(degree); ++i) knots[i] = 0.0;
        for (size_t i = degree + 1; i < controlPoints.size(); ++i) knots[i] = i - degree;
        for (size_t i = controlPoints.size(); i < knots.size(); ++i) knots[i] = controlPoints.size() - degree;

        // 경로 마커와 장애물 시각화를 생성
        marker_array_ = createPathMarkers(controlPoints, knots, degree);
        obstacle_array_ = createObstacleMarkers();

        // 주기적으로 퍼블리시할 타이머 설정
        timer_ = this->create_wall_timer(500ms, [this]() {
            marker_pub_->publish(marker_array_);
            marker_pub_->publish(obstacle_array_);
        });

        // 경로 좌표를 터미널에 출력
        printPath(controlPoints, knots, degree);
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    visualization_msgs::msg::MarkerArray marker_array_;
    visualization_msgs::msg::MarkerArray obstacle_array_;

    // Cubic B-spline Basis function
    double basisFunction(int i, int k, double t, const std::vector<double>& knots) {
        if (k == 0) {
            return (t >= knots[i] && t < knots[i + 1]) ? 1.0 : 0.0;
        } else {
            double denom1 = knots[i + k] - knots[i];
            double denom2 = knots[i + k + 1] - knots[i + 1];
            double term1 = (denom1 == 0) ? 0 : (t - knots[i]) / denom1 * basisFunction(i, k - 1, t, knots);
            double term2 = (denom2 == 0) ? 0 : (knots[i + k + 1] - t) / denom2 * basisFunction(i + 1, k - 1, t, knots);
            return term1 + term2;
        }
    }

    // B-spline interpolation function for 3D
    Eigen::Vector3d cubicBSplineInterpolation(double t, const std::vector<Eigen::Vector3d>& controlPoints, const std::vector<double>& knots, int degree) {
        Eigen::Vector3d result(0.0, 0.0, 0.0);
        int n = controlPoints.size() - 1;
        for (int i = 0; i <= n; ++i) {
            double B = basisFunction(i, degree, t, knots);
            result += B * controlPoints[i];
        }
        return result;
    }

    visualization_msgs::msg::MarkerArray createPathMarkers(const std::vector<Eigen::Vector3d>& controlPoints, const std::vector<double>& knots, int degree) {
        visualization_msgs::msg::MarkerArray marker_array;
        double t_min = knots[degree];
        double t_max = knots[controlPoints.size() - 1 + 1];

        int id = 0;
        for (double t = t_min; t <= t_max; t += 0.1) {
            Eigen::Vector3d point = cubicBSplineInterpolation(t, controlPoints, knots, degree);

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = this->now();
            marker.ns = "spline_path";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = point.x();
            marker.pose.position.y = point.y();
            marker.pose.position.z = point.z();
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.lifetime = rclcpp::Duration(0, 0);  // 지속적인 표시

            marker_array.markers.push_back(marker);
        }
        return marker_array;
    }

    visualization_msgs::msg::MarkerArray createObstacleMarkers() {
        visualization_msgs::msg::MarkerArray marker_array;

        std::vector<Eigen::Vector3d> cylinder_positions = { 
            {-0.8, 0, 0}, 
            {0, -0.8, 0}, 
            {0.8, 0, 0}, 
            {0, 0.8, 0}
        };

        for (size_t i = 0; i < cylinder_positions.size(); ++i) {
            visualization_msgs::msg::Marker cylinder_marker;
            cylinder_marker.header.frame_id = "world";
            cylinder_marker.header.stamp = this->now();
            cylinder_marker.ns = "cylinder_markers";
            cylinder_marker.id = i;
            cylinder_marker.type = visualization_msgs::msg::Marker::CYLINDER;
            cylinder_marker.action = visualization_msgs::msg::Marker::ADD;
            cylinder_marker.pose.position.x = cylinder_positions[i].x();
            cylinder_marker.pose.position.y = cylinder_positions[i].y();
            cylinder_marker.pose.position.z = 0.5;
            cylinder_marker.scale.x = 0.05;
            cylinder_marker.scale.y = 0.05;
            cylinder_marker.scale.z = 1.0;
            cylinder_marker.color.r = 0.0;
            cylinder_marker.color.g = 0.0;
            cylinder_marker.color.b = 1.0;
            cylinder_marker.color.a = 1.0;
            cylinder_marker.lifetime = rclcpp::Duration(0, 0);  // 지속적인 표시

            marker_array.markers.push_back(cylinder_marker);
        }
        return marker_array;
    }

    void printPath(const std::vector<Eigen::Vector3d>& controlPoints, const std::vector<double>& knots, int degree) {
        double t_min = knots[degree];
        double t_max = knots[controlPoints.size() - 1 + 1];

        RCLCPP_INFO(this->get_logger(), "Spline Interpolation Path:");
        for (double t = t_min; t <= t_max; t += 0.1) {
            Eigen::Vector3d point = cubicBSplineInterpolation(t, controlPoints, knots, degree);
            RCLCPP_INFO(this->get_logger(), "t=%.2f -> (x=%.2f, y=%.2f, z=%.2f)", t, point.x(), point.y(), point.z());
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SplineInterpolationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
