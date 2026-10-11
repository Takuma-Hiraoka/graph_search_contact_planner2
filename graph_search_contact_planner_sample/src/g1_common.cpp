#include <graph_search_contact_planner_sample/sample_common.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ik_constraint/JointAngleConstraint.h>
#include <ik_constraint/JointLimitConstraint.h>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <stdexcept>

namespace graph_search_contact_planner_sample {
  void generateG1(graph_search_contact_planner::ContactPlanner::GSCPParam& param, SampleModel& sample) {
    sample.robotDirectory = ament_index_cpp::get_package_share_directory("graph_search_contact_planner_sample") + "/unitree_ros/robots/g1_description";
    const std::string urdf = sample.robotDirectory + "/g1_29dof.urdf";
    std::shared_ptr<pinocchio::Model> model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(urdf,pinocchio::JointModelFreeFlyer(),*model); model->name = "G1";
    sample.robotGeometry = std::make_shared<pinocchio::GeometryModel>();
    pinocchio::urdf::buildGeom(*model,urdf,pinocchio::COLLISION,*sample.robotGeometry,std::vector<std::string>{sample.robotDirectory});
    Eigen::VectorXd q = pinocchio::neutral(*model); q[2] = 0.793;
    const std::string sides[2] = {"left","right"};
    for(int side = 0; side < 2; ++side) {
      q[model->idx_qs[model->getJointId(sides[side]+"_hip_pitch_joint")]] = -0.2;
      q[model->idx_qs[model->getJointId(sides[side]+"_knee_joint")]] = 0.4;
      q[model->idx_qs[model->getJointId(sides[side]+"_ankle_pitch_joint")]] = -0.2;
      q[model->idx_qs[model->getJointId(sides[side]+"_shoulder_pitch_joint")]] = -0.3;
      q[model->idx_qs[model->getJointId(sides[side]+"_elbow_joint")]] = 0.8;
    }
    param.models.push_back(model);
    param.currentContactState = std::make_shared<graph_search_contact_planner::ContactState>();
    param.goalContactState = std::make_shared<graph_search_contact_planner::ContactState>();
    pinocchio::Data data(*model); pinocchio::forwardKinematics(*model,data,q);
    const pinocchio::JointIndex foot = model->getJointId("left_ankle_roll_joint");
    q[2] -= (data.oMi[foot]*pinocchio::SE3(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0.03,0,-0.035))).translation().z();
    pinocchio::forwardKinematics(*model,data,q);
    param.currentContactState->frame.push_back(q);
    for(int side = 0; side < 2; ++side) {
      std::shared_ptr<graph_search_contact_planner::ContactCandidate> leg =
        std::make_shared<graph_search_contact_planner::ContactCandidate>("G1",sides[side]+"_ankle_roll_link",
          pinocchio::SE3(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0.03,0,-0.035)),false);
      param.contactDynamicCandidates.push_back(leg);
      const pinocchio::SE3 pose = data.oMi[model->getJointId(sides[side]+"_ankle_roll_joint")] * leg->localPose;
      graph_search_contact_planner::ContactCandidate floor("floor","floor",pose);
      param.currentContactState->contacts.push_back(graph_search_contact_planner::Contact(*leg,floor));
      std::shared_ptr<graph_search_contact_planner::ContactCandidate> hand =
        std::make_shared<graph_search_contact_planner::ContactCandidate>("G1",sides[side]+"_wrist_yaw_link",
          pinocchio::SE3(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0.10,0,0)),false);
      hand->weight.tail<3>().setZero(); hand->halfLength = 0.02; hand->halfWidth = 0.02;
      param.contactDynamicCandidates.push_back(hand);
    }
    for(pinocchio::JointIndex id = 1; id < model->njoints; ++id) {
      if(model->nqs[id] != 1 || model->nvs[id] != 1) continue;
      std::shared_ptr<ik_constraint::JointLimitConstraint> limit = std::make_shared<ik_constraint::JointLimitConstraint>(model);
      limit->setJointId(id); param.constraints.push_back(limit);
      std::shared_ptr<ik_constraint::JointAngleConstraint> nominal = std::make_shared<ik_constraint::JointAngleConstraint>(model);
      nominal->setAJointId(id); nominal->setBOffset(q[model->idx_qs[id]]); nominal->setPrecision(1e10);
      param.nominals.push_back(nominal);
    }
  }
  void setViewerConfiguration(mujoco_viewer::Viewer& viewer, const pinocchio::Model& model, const Eigen::VectorXd& q) {
    for(pinocchio::JointIndex id = 1; id < model.njoints; ++id) {
      const Eigen::Index qi = model.idx_qs[id];
      if(model.nqs[id] == 7 && model.nvs[id] == 6) {
        int joint = -1;
        for(int j = 0; j < viewer.model()->njnt; ++j) if(viewer.model()->jnt_type[j] == mjJNT_FREE) { joint = j; break; }
        if(joint < 0) throw std::runtime_error("G1 viewer free joint missing");
        const int address = viewer.model()->jnt_qposadr[joint];
        for(int i = 0; i < 3; ++i) viewer.data()->qpos[address+i] = q[qi+i];
        viewer.data()->qpos[address+3] = q[qi+6];
        for(int i = 0; i < 3; ++i) viewer.data()->qpos[address+4+i] = q[qi+3+i];
      } else if(model.nqs[id] == 1) {
        const int joint = mj_name2id(viewer.model(),mjOBJ_JOINT,model.names[id].c_str());
        if(joint < 0) throw std::runtime_error("G1 viewer joint missing: " + model.names[id]);
        viewer.data()->qpos[viewer.model()->jnt_qposadr[joint]] = q[qi];
      }
    }
    mj_forward(viewer.model(),viewer.data());
  }
}
