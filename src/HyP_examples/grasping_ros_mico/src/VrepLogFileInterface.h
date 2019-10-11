/* 
 * File:   VrepLogFileInterface.h
 * Author: neha
 *
 * Created on October 6, 2017, 11:29 AM
 */

#ifndef VREPLOGFILEINTERFACE_H
#define	VREPLOGFILEINTERFACE_H

#include "VrepDataInterface.h"
#include "LearningModel.h"
#include "Python.h"


class VrepLogFileInterface : public VrepDataInterface{
public:
    
    VrepLogFileInterface(int start_state_index_ = -1);
    virtual ~VrepLogFileInterface();
    

    bool StepActual(GraspingStateRealArm& state, double random_num, int action, double& reward, GraspingObservation& obs) const;

    ValuedAction NextAction(History h);
private:
    std::string logFileName;
    PyObject *lfp;
    PyObject *run_function;
    std::string next_step_values;

};

#endif	/* VREPLOGFILEINTERFACE_H */

