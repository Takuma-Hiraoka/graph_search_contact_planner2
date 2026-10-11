#include <graph_search_contact_planner/contact_graph.h>
#include <ik_constraint/PositionConstraint.h>
#include <ik_constraint/CollisionConstraint.h>
#include <ik_constraint_esdf/EsdfCollisionConstraint.h>
#include <ik_constraint_scfr/ScfrConstraint.h>
#include <pinocchio/algorithm/kinematics.hpp>
#include <algorithm>
#include <stdexcept>

namespace graph_search_contact_planner {
  std::size_t getModelIndex(const ContactCandidate& candidate,
    const std::vector<std::shared_ptr<pinocchio::Model> >& models) {
    for(std::size_t i = 0; i < models.size(); ++i) if(models[i]->name == candidate.bodyName) return i;
    throw std::invalid_argument("ContactPlanner: model not found: " + candidate.bodyName);
  }
  pinocchio::JointIndex getJointId(const ContactCandidate& candidate, const pinocchio::Model& model) {
    if(candidate.isStatic) return 0;
    if(model.existFrame(candidate.linkName)) return model.frames[model.getFrameId(candidate.linkName)].parentJoint;
    if(model.existJointName(candidate.linkName)) return model.getJointId(candidate.linkName);
    throw std::invalid_argument("ContactPlanner: link or joint not found: " + candidate.linkName);
  }
  pinocchio::SE3 getJointLocalPose(const ContactCandidate& candidate, const pinocchio::Model& model) {
    if(!candidate.isStatic && model.existFrame(candidate.linkName))
      return model.frames[model.getFrameId(candidate.linkName)].placement * candidate.localPose;
    return candidate.localPose;
  }
  pinocchio::SE3 getWorldPose(const ContactCandidate& candidate,
    const std::vector<std::shared_ptr<pinocchio::Model> >& models,
    const std::vector<std::shared_ptr<pinocchio::Data> >& data) {
    if(candidate.isStatic) return candidate.localPose;
    const std::size_t i = getModelIndex(candidate,models);
    return data[i]->oMi[getJointId(candidate,*models[i])] * getJointLocalPose(candidate,*models[i]);
  }
  namespace {
    bool matches(const ContactCandidate& candidate, const std::shared_ptr<const pinocchio::Model>& model,
      pinocchio::JointIndex joint) {
      return candidate.isStatic ? joint == 0 : candidate.bodyName == model->name && getJointId(candidate,*model) == joint;
    }
    bool matchesPair(const Contact& contact, const std::shared_ptr<ik_constraint::CollisionConstraint>& collision) {
      return (matches(contact.c1,collision->getAModel(),collision->getAJointId()) && matches(contact.c2,collision->getBModel(),collision->getBJointId())) ||
        (matches(contact.c2,collision->getAModel(),collision->getAJointId()) && matches(contact.c1,collision->getBModel(),collision->getBJointId()));
    }
    bool skipCollision(const std::shared_ptr<ik_constraint::IKConstraint>& constraint,
      const ContactState& support, const Contact& move, const IKState& state) {
      const std::shared_ptr<ik_constraint::CollisionConstraint> collision =
        std::dynamic_pointer_cast<ik_constraint::CollisionConstraint>(constraint);
      if(!collision) return false;
      const bool distanceField = static_cast<bool>(std::dynamic_pointer_cast<ik_constraint_esdf::EsdfCollisionConstraint>(constraint));
      for(std::size_t i = 0; i < support.contacts.size(); ++i) {
        const Contact& contact = support.contacts[i];
        if(distanceField) {
          if((matches(contact.c1,collision->getAModel(),collision->getAJointId()) && contact.c2.isStatic) ||
             (matches(contact.c2,collision->getAModel(),collision->getAJointId()) && contact.c1.isStatic)) return true;
        } else if(matchesPair(contact,collision)) return true;
      }
      if(state != IKState::ATTACH_FIXED && state != IKState::DETACH_FIXED) return false;
      if(distanceField) return matches(move.c1,collision->getAModel(),collision->getAJointId()) || matches(move.c2,collision->getAModel(),collision->getAJointId());
      return matchesPair(move,collision);
    }
    std::shared_ptr<ik_constraint::PositionConstraint> makePosition(const Contact& contact,
      const ContactPlanner::GSCPParam& param, double clearance) {
      const ContactCandidate& a = contact.c1.isStatic ? contact.c2 : contact.c1;
      const ContactCandidate& b = contact.c1.isStatic ? contact.c1 : contact.c2;
      if(a.isStatic) throw std::invalid_argument("ContactPlanner: static-static contact");
      const std::size_t ai = getModelIndex(a,param.models);
      std::shared_ptr<ik_constraint::PositionConstraint> result;
      if(b.isStatic) result = std::make_shared<ik_constraint::PositionConstraint>(param.models[ai]);
      else result = std::make_shared<ik_constraint::PositionConstraint>(param.models[ai],param.models[getModelIndex(b,param.models)]);
      result->setAJointId(getJointId(a,*param.models[ai])); result->setALocalPose(getJointLocalPose(a,*param.models[ai]));
      const pinocchio::Model& bModel = b.isStatic ? *param.models[ai] : *param.models[getModelIndex(b,param.models)];
      result->setBJointId(getJointId(b,bModel));
      pinocchio::SE3 target = getJointLocalPose(b,bModel);
      target.translation() += clearance * target.rotation().col(2);
      result->setBLocalPose(target); result->setPrecision(param.contactPrecision);
      result->setWeight(a.weight.cwiseMin(b.weight));
      result->setMaxError(Eigen::Matrix<double,6,1>::Constant(0.03));
      return result;
    }
    Eigen::SparseMatrix<double,Eigen::RowMajor> wrenchMatrix(const ContactCandidate& contact) {
      Eigen::SparseMatrix<double,Eigen::RowMajor> C(11,6);
      C.insert(0,2) = 1.0;
      for(int axis = 0; axis < 2; ++axis) {
        C.insert(1+axis*2,axis) = 1.0; C.insert(1+axis*2,2) = contact.friction;
        C.insert(2+axis*2,axis) = -1.0; C.insert(2+axis*2,2) = contact.friction;
      }
      const double coefficients[3] = {contact.halfWidth,contact.halfLength,contact.torsionalFriction};
      for(int axis = 0; axis < 3; ++axis) {
        C.insert(5+axis*2,3+axis) = 1.0; C.insert(5+axis*2,2) = coefficients[axis];
        C.insert(6+axis*2,3+axis) = -1.0; C.insert(6+axis*2,2) = coefficients[axis];
      }
      return C;
    }
  }
  bool ContactPlanner::solveContactIK(const ContactState& supportState, Contact& moveContact, ContactState& postState,
    const IKState& state, std::vector<Eigen::VectorXd>& configurations, const GSCPParam& parameters) {
    const std::vector<Eigen::VectorXd> initial = configurations;
    std::vector<std::vector<std::shared_ptr<ik_constraint::IKConstraint> > > levels(4);
    for(std::size_t i = 0; i < parameters.constraints.size(); ++i)
      if(!skipCollision(parameters.constraints[i],supportState,moveContact,state)) levels[0].push_back(parameters.constraints[i]);
    std::vector<std::shared_ptr<pinocchio::Data> > data = ik_constraint::IKConstraint::createDataVector(parameters.models);
    for(std::size_t i = 0; i < data.size(); ++i) pinocchio::forwardKinematics(*parameters.models[i],*data[i],configurations[i]);
    std::vector<Contact> supports;
    for(std::size_t i = 0; i < supportState.contacts.size(); ++i) {
      if(!moveContact.c1.linkName.empty() && supportState.contacts[i] == moveContact) continue;
      supports.push_back(supportState.contacts[i]);
      levels[1].push_back(makePosition(supportState.contacts[i],parameters,0.0));
    }
    for(std::size_t modelIndex = 0; modelIndex < parameters.models.size(); ++modelIndex) {
      if(parameters.models[modelIndex]->nv == 0) continue;
      std::vector<Eigen::Isometry3d> poses;
      std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > As, Cs;
      std::vector<Eigen::VectorXd> bs, dls, dus;
      for(std::size_t i = 0; i < supports.size(); ++i) {
        const ContactCandidate* dynamic = nullptr; const ContactCandidate* other = nullptr;
        if(!supports[i].c1.isStatic && supports[i].c1.bodyName == parameters.models[modelIndex]->name) {
          dynamic = &supports[i].c1; other = &supports[i].c2;
        } else if(!supports[i].c2.isStatic && supports[i].c2.bodyName == parameters.models[modelIndex]->name) {
          dynamic = &supports[i].c2; other = &supports[i].c1;
        }
        if(!dynamic) continue;
        Eigen::Isometry3d pose;
        pose.matrix() = getWorldPose(other->isStatic ? *other : *dynamic,parameters.models,data).toHomogeneousMatrix();
        poses.push_back(pose); As.emplace_back(0,6); bs.emplace_back(0); Cs.push_back(wrenchMatrix(*dynamic));
        dls.push_back(Eigen::VectorXd::Zero(11));
        Eigen::VectorXd upper = Eigen::VectorXd::Constant(11,1e10); upper[0] = dynamic->maxNormalForce; dus.push_back(upper);
      }
      if(poses.empty()) return false;
      std::shared_ptr<ik_constraint_scfr::ScfrConstraint> scfr = std::make_shared<ik_constraint_scfr::ScfrConstraint>(parameters.models[modelIndex]);
      scfr->setPoses(poses); scfr->setAs(As); scfr->setBs(bs); scfr->setCs(Cs); scfr->setDls(dls); scfr->setDus(dus);
      levels[0].push_back(scfr);
    }
    std::shared_ptr<ik_constraint::PositionConstraint> moving;
    if(!moveContact.c1.linkName.empty()) {
      const double clearance = state == IKState::DETACH_FIXED || state == IKState::ATTACH_PRE ? parameters.contactClearance : 0.0;
      moving = makePosition(moveContact,parameters,clearance); levels[2].push_back(moving);
    }
    levels[3] = parameters.nominals;
    std::shared_ptr<std::vector<std::vector<Eigen::VectorXd> > > path = std::make_shared<std::vector<std::vector<Eigen::VectorXd> > >();
    std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks;
    bool solved = prioritized_inverse_kinematics_solver::solveIKLoop(parameters.models,configurations,levels,parameters.rejections,tasks,parameters.pikParam,path);
    if(!solved && moving) {
      configurations = initial; path->clear();
      global_inverse_kinematics_solver::GIKParam gik = parameters.gikParam;
      const ContactCandidate& candidate = moveContact.c1.isStatic ? moveContact.c2 : moveContact.c1;
      gik.projectionModelIndex = getModelIndex(candidate,parameters.models);
      gik.projectionJointId = getJointId(candidate,*parameters.models[gik.projectionModelIndex]);
      gik.projectionLocalPose = getJointLocalPose(candidate,*parameters.models[gik.projectionModelIndex]);
      const std::vector<std::vector<std::shared_ptr<ik_constraint::IKConstraint> > > hard{levels[0],levels[1]};
      solved = global_inverse_kinematics_solver::solveGIK(parameters.models,configurations,hard,levels[2],parameters.nominals,parameters.rejections,gik,path);
      if(solved && !path->empty()) configurations = path->back();
    }
    if(!solved) { configurations = initial; return false; }
    if(path->empty()) path->push_back(configurations);
    postState.transition.insert(postState.transition.end(),path->begin(),path->end());
    return true;
  }
  bool ContactPlanner::checkTransitionImpl(const ContactState& preState, ContactState& postState, const GSCPParam& parameters) {
    if(preState.frame.size() != parameters.models.size()) throw std::invalid_argument("ContactPlanner: invalid state frame");
    for(std::size_t i = 0; i < parameters.models.size(); ++i)
      if(preState.frame[i].size() != parameters.models[i]->nq) throw std::invalid_argument("ContactPlanner: invalid configuration size");
    std::vector<Eigen::VectorXd> configurations = preState.frame;
    postState.transition.clear();
    if(postState.contacts.size() == preState.contacts.size()+1) {
      Contact& move = postState.contacts.back();
      if(!this->solveContactIK(preState,move,postState,IKState::ATTACH_PRE,configurations,parameters) ||
         !this->solveContactIK(preState,move,postState,IKState::ATTACH_FIXED,configurations,parameters)) return false;
    } else if(postState.contacts.size()+1 == preState.contacts.size()) {
      Contact move; bool found = false;
      for(std::size_t i = 0; i < preState.contacts.size(); ++i) {
        if(std::find(postState.contacts.begin(),postState.contacts.end(),preState.contacts[i]) == postState.contacts.end()) {
          move = preState.contacts[i]; found = true; break;
        }
      }
      if(!found || !this->solveContactIK(postState,move,postState,IKState::DETACH_FIXED,configurations,parameters)) return false;
    } else if(preState == postState) {
      Contact noMove;
      if(!this->solveContactIK(preState,noMove,postState,IKState::SLIDE,configurations,parameters)) return false;
    } else return false;
    postState.frame = configurations;
    return true;
  }
  void generateCandidateMarkers(mujoco_viewer::Viewer& viewer,
    const std::vector<std::shared_ptr<pinocchio::Model> >& models,
    const std::vector<Eigen::VectorXd>& configurations,
    const std::vector<std::shared_ptr<ContactCandidate> >& candidates) {
    std::vector<std::shared_ptr<pinocchio::Data> > data = ik_constraint::IKConstraint::createDataVector(models);
    for(std::size_t i = 0; i < models.size(); ++i) pinocchio::forwardKinematics(*models[i],*data[i],configurations[i]);
    const std::array<float,4> colors[3] = {{{1,0,0,1}},{{0,1,0,1}},{{0,0,1,1}}};
    for(std::size_t i = 0; i < candidates.size(); ++i) {
      const pinocchio::SE3 pose = getWorldPose(*candidates[i],models,data);
      const Eigen::Vector3d start = pose.translation();
      for(int axis = 0; axis < 3; ++axis) {
        const Eigen::Vector3d end = start+0.04*pose.rotation().col(axis);
        mujoco_viewer::appendLine(viewer,{start.x(),start.y(),start.z()},{end.x(),end.y(),end.z()},2.0,colors[axis]);
      }
    }
  }
}
std::ostream& operator<<(std::ostream& stream, const graph_search_contact_planner::ContactState& state) {
  for(std::size_t i = 0; i < state.contacts.size(); ++i)
    stream << state.contacts[i].c1.bodyName << "/" << state.contacts[i].c1.linkName << " -> " <<
      state.contacts[i].c2.bodyName << "/" << state.contacts[i].c2.linkName << " at " << state.contacts[i].c2.localPose.translation().transpose() << '\n';
  return stream;
}
