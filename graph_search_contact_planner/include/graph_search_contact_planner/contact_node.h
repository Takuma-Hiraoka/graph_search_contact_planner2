#pragma once
#include <graph_search/node.h>
#include <graph_search_contact_planner/contact_state.h>
namespace graph_search_contact_planner {
  class ContactNode : public graph_search::Node {
  public:
    const ContactState& state() const { return this->state_; }
    ContactState& state() { return this->state_; }
  private:
    ContactState state_;
  };
}
