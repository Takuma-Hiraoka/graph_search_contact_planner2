#include <graph_search_contact_planner_sample/sample_common.h>
#include <chrono>
#include <future>
#include <iostream>
#include <thread>

int main() {
  graph_search_contact_planner::ContactPlanner planner;
  graph_search_contact_planner_sample::SampleModel sample;
  graph_search_contact_planner_sample::generateG1(planner.param,sample);
  graph_search_contact_planner_sample::generateStepWorld(planner.param,sample);
  planner.debugLevel() = 1; planner.threads() = 1; planner.maxExtendNum() = 200;
  std::shared_ptr<mujoco_viewer::Viewer> viewer = std::make_shared<mujoco_viewer::Viewer>();
  planner.param.viewer = viewer; viewer->viewModel(sample.viewerModel);
  viewer->option()->geomgroup[0] = 1; viewer->option()->geomgroup[1] = 1;
  viewer->camera()->distance = 2.5; viewer->camera()->lookat[2] = 0.6;
  graph_search_contact_planner_sample::setViewerConfiguration(*viewer,*planner.param.models[0],planner.param.currentContactState->frame[0]);
  std::future<bool> result = std::async(std::launch::async,[&planner]() { return planner.solve(); });
  while(result.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
    if(viewer->isOpen()) {
      viewer->updateScene();
      graph_search_contact_planner::generateCandidateMarkers(*viewer,planner.param.models,
        planner.param.currentContactState->frame,planner.param.contactStaticCandidates);
      viewer->drawScene(); viewer->pollEvents();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }
  if(!result.get()) { std::cerr << "Contact planning failed" << std::endl; return 1; }
  std::vector<graph_search_contact_planner::ContactState> path; planner.goalPath(path);
  std::cout << "Contact planning succeeded: " << path.size() << " states" << std::endl;
  while(viewer->isOpen()) {
    for(std::size_t state = 0; state < path.size() && viewer->isOpen(); ++state) {
      std::cout << path[state];
      const std::size_t count = path[state].transition.size();
      for(std::size_t frame = 0; frame < count && viewer->isOpen(); ++frame) {
        graph_search_contact_planner_sample::setViewerConfiguration(*viewer,*planner.param.models[0],path[state].transition[frame][0]);
        viewer->updateScene();
        graph_search_contact_planner::generateCandidateMarkers(*viewer,planner.param.models,path[state].transition[frame],planner.param.contactStaticCandidates);
        viewer->drawScene(); viewer->pollEvents();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
  }
  return 0;
}
