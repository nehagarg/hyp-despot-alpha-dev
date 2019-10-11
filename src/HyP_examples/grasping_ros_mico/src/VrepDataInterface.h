/* 
 * File:   VrepDataInterface.h
 * Author: neha
 *
 * Created on November 11, 2016, 6:29 PM
 */

#ifndef VREPDATAINTERFACE_H
#define	VREPDATAINTERFACE_H

#include "RobotInterface.h"

class VrepDataInterface  : public RobotInterface{
public:
    VrepDataInterface(int start_state_index_ = -1);
    VrepDataInterface(const VrepDataInterface& orig);
    virtual ~VrepDataInterface();
    
    mutable int start_state_index;
    void CheckAndUpdateGripperBounds(GraspingStateRealArm& grasping_state, int action) const;

    //bool CheckTouch(double current_sensor_values[], int on_bits[], int size = 2) const;
    

    virtual void CreateStartState(GraspingStateRealArm& initial_state, std::string type) const;
    virtual std::pair <std::map<int,double>, std::vector<double> > GetBeliefObjectProbability(std::vector<int> belief_object_ids) const
    {
        return RobotInterface::GetBeliefObjectProbability(belief_object_ids);
    }

    void GetDefaultPickState(GraspingStateRealArm& grasping_state, double rand_num,int pick_type = 2) const;
    

    void GetRewardBasedOnGraspStability(GraspingStateRealArm grasping_state, GraspingObservation grasping_obs, double& reward) const;


    bool IsValidPick(GraspingStateRealArm grasping_state, GraspingObservation grasping_obs) const;
    

    bool IsValidState(GraspingStateRealArm grasping_state) const;
    
    static bool IsValidStateStatic(GraspingStateRealArm grasping_state, 
            GraspObject* grasp_object, double min_x_i, double max_x_i, 
            double min_y_i, double max_y_i, double gripper_out_y_diff);

    virtual bool StepActual(GraspingStateRealArm& state, double random_num, int action, double& reward, GraspingObservation& obs) const;

    std::vector<GraspingStateRealArm> InitialStartStateParticles(const GraspingStateRealArm start) const;



private:

};


#endif	/* VREPDATAINTERFACE_H */

