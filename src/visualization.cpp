#include <visualization.hpp>

Visualization::Visualization(ros::NodeHandle& nh): visualize_(nh) {
	visPub_ = visualize_.advertise<visualization_msgs::Marker>("model", 1);
}

Visualization::~Visualization() {
}

void Visualization::visualizeModel(){
	visualization_msgs::Marker marker;
    marker.header.frame_id = "base";
    marker.header.stamp = ros::Time();
    marker.ns = "model";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.74;
    marker.color.g = 0.74;
    marker.color.b = 0.74;
    marker.mesh_resource = "package://extended_kalman_filter/model/Cessna_177_Cardinal.stl";
    visPub_.publish( marker );
}