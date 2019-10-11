#include "grasping_ros_tui.h"
#include "grasping_real_arm.h"

#include "log_file_reader.h"

using namespace despot;



DSPOMDP* RosTUI::InitializeModel(option::Option* options) {
    return InitializeModelCommon(options);
  }







int main(int argc, char* argv[]) {
        std::cout << "In main" << std::endl;
        main_init(argc, argv);
       
  //  GatherSimulationData();
  //  return 0;
    
    //test_python();
    //return 0;
   
 //return RosWithoutDisplayTUI().run(argc, argv);
  
  return RosTUI().RunPlanning(argc, argv);
}


