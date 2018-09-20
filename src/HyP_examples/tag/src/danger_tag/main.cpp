#include <despot/planner.h>
#include "danger_tag.h"
#include "base/base_tag.h"

using namespace despot;

class TUI: public Planner {
public:
  TUI() {
  }

  DSPOMDP* InitializeModel(option::Option* options) {
      BaseTag::NUM_ACTIONS = DangerTag::NUM_DIRECTIONS + 1;
      BaseTag::ERRORS_PER_DIRECTION = 2;
      BaseTag::DEFAULT_MOVEMENT_ERROR = 0.4;
      BaseTag::DANGER_PENALTY = -20;
      BaseTag::STABLE_OPPONENT = true;
      
    DSPOMDP* model = !options[E_PARAMS_FILE] ?
      new DangerTag() : new DangerTag(options[E_PARAMS_FILE].arg);
    return model;
  }

  void InitializeDefaultParameters() {
    Globals::config.pruning_constant = 0.01;
  }
  
  World* InitializeWorld(std::string& world_type, DSPOMDP* model, option::Option* options){
    return InitializePOMDPWorld(world_type, model, options);
  }

  std::string ChooseSolver(){ // Specify the solver used in the planner to be DESPOT
    return "DESPOT";
  }
};

int main(int argc, char* argv[]) {
  return TUI().RunPlanning(argc, argv);
}
