#include <graph_search_contact_planner_sample/sample_common.h>
#include <ik_constraint_coal/CoalCollisionConstraint.h>
#include <coal/shape/geometric_shapes.h>
#include <pinocchio/algorithm/kinematics.hpp>
#include <set>
#include <stdexcept>

namespace graph_search_contact_planner_sample {
  namespace {
    void addBox(SampleModel& sample, mjSpec* spec, const std::string& name,
      const Eigen::Vector3d& position, const Eigen::Vector3d& size) {
      std::shared_ptr<coal::Box> shape = std::make_shared<coal::Box>(size.x(),size.y(),size.z());
      shape->computeLocalAABB();
      sample.worldGeometry->addGeometryObject(pinocchio::GeometryObject(name,0,
        pinocchio::SE3(Eigen::Matrix3d::Identity(),position),shape));
      mjsGeom* geom = mjs_addGeom(mjs_findBody(spec,"world"),nullptr);
      mjs_setName(geom->element,name.c_str()); geom->type = mjGEOM_BOX;
      for(int axis = 0; axis < 3; ++axis) { geom->pos[axis] = position[axis]; geom->size[axis] = size[axis]/2.0; }
      geom->rgba[0] = 0.5f; geom->rgba[1] = 0.6f; geom->rgba[2] = 0.7f; geom->rgba[3] = 1.0f;
    }
  }
  void generateStepWorld(graph_search_contact_planner::ContactPlanner::GSCPParam& param, SampleModel& sample) {
    sample.worldGeometry = std::make_shared<pinocchio::GeometryModel>();
    char error[1024] = {};
    std::shared_ptr<mjSpec> spec(mj_parseXML((sample.robotDirectory+"/g1_29dof.xml").c_str(),nullptr,error,sizeof(error)),mj_deleteSpec);
    if(!spec) throw std::runtime_error(error);
    addBox(sample,spec.get(),"planner_floor",Eigen::Vector3d(0,0,-0.05),Eigen::Vector3d(3,2,0.1));
    addBox(sample,spec.get(),"step",Eigen::Vector3d(0.27,0,0.04),Eigen::Vector3d(0.24,0.7,0.08));
    addBox(sample,spec.get(),"hand_support",Eigen::Vector3d(0.38,0,0.65),Eigen::Vector3d(0.10,0.7,0.05));
    const std::shared_ptr<pinocchio::Model>& model = param.models[0];
    pinocchio::Data data(*model); pinocchio::forwardKinematics(*model,data,param.currentContactState->frame[0]);
    for(std::size_t i = 0; i < param.contactDynamicCandidates.size(); ++i) {
      const graph_search_contact_planner::ContactCandidate& dynamic = *param.contactDynamicCandidates[i];
      const bool hand = dynamic.linkName.find("wrist") != std::string::npos;
      const bool left = dynamic.linkName.find("left") != std::string::npos;
      const Eigen::Vector3d target = hand ? Eigen::Vector3d(0.38,left ? 0.20 : -0.20,0.675) :
        Eigen::Vector3d(0.27,left ? 0.12 : -0.12,0.08);
      std::shared_ptr<graph_search_contact_planner::ContactCandidate> candidate =
        std::make_shared<graph_search_contact_planner::ContactCandidate>("floor2",hand ? "hand_support" : "step",
          pinocchio::SE3(Eigen::Matrix3d::Identity(),target));
      if(hand) candidate->weight.tail<3>().setZero();
      candidate->allowedDynamicLinks.push_back(dynamic.linkName);
      param.contactStaticCandidates.push_back(candidate);
      param.goalContactState->contacts.push_back(graph_search_contact_planner::Contact(dynamic,*candidate));
    }
    std::set<pinocchio::JointIndex> ids;
    for(std::size_t i = 0; i < sample.robotGeometry->geometryObjects.size(); ++i) {
      sample.robotGeometry->geometryObjects[i].geometry->computeLocalAABB();
      ids.insert(sample.robotGeometry->geometryObjects[i].parentJoint);
    }
    for(std::set<pinocchio::JointIndex>::const_iterator id = ids.begin(); id != ids.end(); ++id) {
      std::shared_ptr<ik_constraint_coal::CoalCollisionConstraint> collision =
        std::make_shared<ik_constraint_coal::CoalCollisionConstraint>(model,sample.robotGeometry,*id,model,sample.worldGeometry,0);
      collision->setTolerance(0.005); collision->setPrecision(1e-3); collision->setIgnoreDistance(0.1);
      param.constraints.push_back(collision);
    }
    // Exclude the same joint and its parent, whose shapes intentionally touch.
    for(std::set<pinocchio::JointIndex>::const_iterator a = ids.begin(); a != ids.end(); ++a) {
      for(std::set<pinocchio::JointIndex>::const_iterator b = ids.upper_bound(*a); b != ids.end(); ++b) {
        if(model->parents[*a] == *b || model->parents[*b] == *a) continue;
        std::shared_ptr<ik_constraint_coal::CoalCollisionConstraint> collision =
          std::make_shared<ik_constraint_coal::CoalCollisionConstraint>(model,sample.robotGeometry,*a,model,sample.robotGeometry,*b);
        collision->setTolerance(0.001); collision->setPrecision(1e-3); collision->setIgnoreDistance(0.05);
        param.constraints.push_back(collision);
      }
    }
    sample.viewerModel = std::shared_ptr<mjModel>(mj_compile(spec.get(),nullptr),mj_deleteModel);
    if(!sample.viewerModel) throw std::runtime_error(mjs_getError(spec.get()));
  }
}
