#pragma once
#include <graph_search/graph.h>
#include <graph_search_contact_planner/contact_node.h>
#include <graph_search_contact_planner/util.h>
#include <global_inverse_kinematics_solver/global_inverse_kinematics_solver.h>
#include <prioritized_inverse_kinematics_solver/prioritized_inverse_kinematics_solver.h>

namespace graph_search_contact_planner {
  class ContactPlanner : public graph_search::Planner {
  public:
    class GSCPParam {
    public:
      GSCPParam();
      std::vector<std::shared_ptr<pinocchio::Model> > models;
      std::vector<std::shared_ptr<ik_constraint::IKConstraint> > constraints, rejections, nominals;
      std::shared_ptr<ContactState> currentContactState, goalContactState;
      std::vector<std::shared_ptr<ContactCandidate> > contactDynamicCandidates, contactStaticCandidates;
      prioritized_inverse_kinematics_solver::IKParam pikParam;
      global_inverse_kinematics_solver::GIKParam gikParam;
      std::shared_ptr<mujoco_viewer::Viewer> viewer = nullptr;
      double addCandidateDistance = 1.5;
      double contactClearance = 0.03;
      double contactPrecision = 1e-3;
    };
    class ContactTransitionCheckParam : public graph_search::Planner::TransitionCheckParam {
    public:
      ContactState preState, postState;
      GSCPParam param;
    };
    GSCPParam param;
    bool solve();
    void goalPath(std::vector<ContactState>& path);
    std::shared_ptr<graph_search::Planner::TransitionCheckParam> generateCheckParam() override;
    void preCheckTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam,
      std::shared_ptr<graph_search::Node> node) override;
    void postCheckTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam,
      std::shared_ptr<graph_search::Node> node) override;
    bool checkTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) override;
    bool isGoalSatisfied(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) override;
    std::vector<std::shared_ptr<graph_search::Node> > gatherAdjacentNodes(
      std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) override;
    void calcHeuristic(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam,
      std::shared_ptr<graph_search::Node> node) override;
    void addNodes2Graph(std::vector<std::shared_ptr<graph_search::Node> >& nodes) override;
    bool checkTransitionImpl(const ContactState& preState, ContactState& postState, const GSCPParam& parameters);
    bool solveContactIK(const ContactState& supportState, Contact& moveContact, ContactState& postState,
      const IKState& state, std::vector<Eigen::VectorXd>& configurations, const GSCPParam& parameters);
  };
}
