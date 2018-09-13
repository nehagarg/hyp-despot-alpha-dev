#include <despot/planner.h>
#include "tiger.h"
#include "tiger2actions.h"

using namespace despot;

class TUI: public Planner {
public:
  TUI() {
  }
 
  DSPOMDP* InitializeModel(option::Option* options) {
    DSPOMDP* model;
    model = !options[E_PARAMS_FILE] ?
       new  Tiger2actions(): new Tiger();
    return model;
  }
  
  void InitializeDefaultParameters() {
  }
};

int main(int argc, char* argv[]) {
  return TUI().RunPlanning(argc, argv);
}
