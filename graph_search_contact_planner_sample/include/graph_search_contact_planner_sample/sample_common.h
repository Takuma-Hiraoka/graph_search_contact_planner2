#pragma once
#include <graph_search_contact_planner/contact_graph.h>
#include <pinocchio/multibody/geometry.hpp>
namespace graph_search_contact_planner_sample {
  struct SampleModel {
    std::string robotDirectory;
    std::shared_ptr<pinocchio::GeometryModel> robotGeometry, worldGeometry;
    std::shared_ptr<mjModel> viewerModel;
  };
  void generateG1(graph_search_contact_planner::ContactPlanner::GSCPParam& param, SampleModel& sample);
  void generateStepWorld(graph_search_contact_planner::ContactPlanner::GSCPParam& param, SampleModel& sample);
  void setViewerConfiguration(mujoco_viewer::Viewer& viewer, const pinocchio::Model& model, const Eigen::VectorXd& q);
}
