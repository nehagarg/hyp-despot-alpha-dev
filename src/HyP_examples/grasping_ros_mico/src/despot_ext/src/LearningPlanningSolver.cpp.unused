/* 
 * File:   LearningPlanningSolver.cpp
 * Author: neha
 * 
 * Created on September 5, 2016, 3:01 PM
 */

#include "LearningPlanningSolver.h"
#include <despot/core/lower_bound.h>
#include "LearningSolverBase.h"

LearningPlanningSolver::LearningPlanningSolver(const LearningModel* model, ScenarioLowerBound* lb, ScenarioUpperBound* ub, Belief* belief)
        : LearningSolverBase(model, belief), despotSolver(model, lb, ub, belief), deepLearningSolver(model),currentSolverLearning(true) {

    switching_method = model->GetSwitchingMethod();
    if(switching_method == 1 || switching_method == 2 )
    {
    //Py_SetProgramName(argv[0]);
    Py_Initialize();
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('python_scripts')");
    PyRun_SimpleString("sys.path.append('python_scripts/deepLearning')");
    char ** argv;
    PySys_SetArgv(0, argv);
    PyRun_SimpleString("print sys.argv[0]");
    //PyRun_SimpleString("import tensorflow as tf");
    PyObject *pName;
    pName = PyString_FromString("joint_training_model");
    PyObject *pModule = PyImport_Import(pName);
    if(pModule == NULL)
    {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"joint training model\"\n");
        assert(0==1);
        return ;
    }
    Py_DECREF(pName);
    load_function = PyObject_GetAttrString(pModule, "get_svm_model");
    if (!(load_function && PyCallable_Check(load_function)))
    {
        if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"get_svm_model\"\n");
    }
    run_function = PyObject_GetAttrString(pModule, "get_svm_output_from_model");
    if (!(run_function && PyCallable_Check(run_function)))
    {
        if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"get_svm_output_from_model\"\n");
    }
    
    
    load_svm_model(model->svm_model_prefix);

    }
    if(switching_method == 3)
    {
        deepPolicy = new LearnedPolicy(model_, model_->CreateParticleLowerBound(), belief_, &deepLearningSolver);
    }
}

LearningPlanningSolver::~LearningPlanningSolver() {
}

ValuedAction LearningPlanningSolver::Search() {
    ValuedAction ans;
    
    //Hack for running trajectory start
    /*
    if (hist_size == 0) return ValuedAction(1, 1);
    if (hist_size == 1) return ValuedAction(8, 1);
    if (hist_size == 2) return ValuedAction(9, 1);
    return ValuedAction(10,1);
    */        
    //Hack for running trajectory end
    
    //int automatic_switching_method = ((LearningModel*)model_)->GetSwitchingMethod();
    
    if (switching_method <= 2)
    {
        if (currentSolverLearning)
        {
            if(ShallISwitchFromLearningToPlanning(history_))
            {
                currentSolverLearning = false;
            }
        }
        else
        {
            if(ShallISwitchFromPlanningToLearning(history_))
            {
                currentSolverLearning = true;
            }
        }
    
        if (currentSolverLearning)
        {
            if (next_action == -1) //Not updated during svm command. So search required
            {
            ans =  deepLearningSolver.Search();
            std::cout << "(" << ans.action << "," << ans.value << ")" << std::endl;
            std::cout << "Belief printing from learning planning solver" << std::endl;
            //std::cout << *belief_ << std::endl;
            if (ans.value < 0.0)
            {
                ans =  despotSolver.Search();
            }
            }
            else
            {
                std::cout << "Before calling exec" << std::endl; //Phrase indicating use of learned policy
                ans.action = next_action;
                ans.value = 1;
                next_action = -1;
            }
            
        }
        else {
            ans =  despotSolver.Search();
        }
    }
    else 
    {
        ValuedAction ans1 =  despotSolver.Search();
        ValuedAction ans3 = deepLearningSolver.Search(history_, 0);
        if(ans3.action == ans1.action)
        {
            std::cout << "Before calling exec dummy" << std::endl;
            return ans1;
            
        }
        ValuedAction ans2 = GetLowerBoundForLearnedPolicy();
        std::cout << "Learned policy bound :(" << ans2.action << "," << ans2.value << ")\n";
        
        
        if (ans1.value > ans2.value)
        {
            ans = ans1;
        }
        else {
             std::cout << "Before calling exec" << std::endl; //Phrase indicating use of learned policy
            ans = ans2;
        }
        
    }
    
    return ans;
}

ValuedAction LearningPlanningSolver::GetLowerBoundForLearnedPolicy() {
    std::cout << "Getting lower bound for learned policy\n";
    ((LearningModel*)model_)->SetStoreObsHash(true);
    ValuedAction ans;
    if(deepPolicy == NULL)
    {
        std::cout << "Initializing deep policy\n";
        deepPolicy = new LearnedPolicy(model_, model_->CreateParticleLowerBound(), belief_, &deepLearningSolver);
        
    }
    //std::cout << "Initialized deep policy\n";
    int sim_len = Globals::config.max_policy_sim_len;
    if(switching_method == 3)
    {
        Globals::config.max_policy_sim_len = ((LearningModel*)model_)->switch_threshold ;
    }
    ans = deepPolicy->Search();
    Globals::config.max_policy_sim_len = sim_len;
    ((LearningModel*)model_)->SetStoreObsHash(false);
    return ans;
    
}


void LearningPlanningSolver::Update(int action, uint64_t obs) {
    despotSolver.Update(action, obs);
    deepLearningSolver.Update(action, obs);
    history_.Add(action, obs);


}

void LearningPlanningSolver::belief(Belief* b) {
    despotSolver.belief(b);
    belief_ = b;
    history_.Truncate(0);
    if(switching_method == 3)
    {
        deepPolicy->belief(b);
    }
}

bool LearningPlanningSolver::ShallISwitchFromLearningToPlanning(History h) {
    std::cout<< "Asking for switch using method" << switching_method << std::endl;
    next_action = -1;
        if (switching_method == 0)
        {
            int switch_threshold = ((LearningModel*)model_)->switch_threshold;
            int hist_size = h.Size();
            return ((hist_size/switch_threshold) % 2 != 0);
        }
        
        
    std::pair<int, int> svm_answer = GetSVMOutput(h);
    int seen_scenario_correct_int = svm_answer.first;
    int seen_scenario_wrong_int = svm_answer.second;
    
    int seen_scenario;
        if (switching_method == 1)
        {
            if((seen_scenario_correct_int == 1) && (seen_scenario_wrong_int == -1))
            {
                seen_scenario = 1;
            }
            else
            {
                seen_scenario = -1;
            }
        }
        
        if (switching_method == 2)
        {
            seen_scenario = seen_scenario_correct_int ; //for automatic switching method 2
        }
       
        if (seen_scenario == 1){
            return false;
        }
        else
        {
            return true;
        }
    
}

bool LearningPlanningSolver::ShallISwitchFromPlanningToLearning(History h) {
    if(ShallISwitchFromLearningToPlanning(h) == true)
        {
            return false;
        }
        else
        {
            return true;
        }

}

std::pair<int, int> LearningPlanningSolver::GetSVMOutput(History h) {
    
    //std::pair<int, int> ans2 =  GetSVMOutputUsingCommandLine(h);
    std::pair<int, int> ans1 =  GetSVMOutputUsingPythonFunction(h);
    //assert(ans1.first == ans2.first);
    //if(SecondSVMRequired())
    //{
    //    assert(ans1.second == ans2.second);
    //}
    return ans1;
}

std::pair<int, int> LearningPlanningSolver::GetSVMOutputUsingCommandLine(History h) {
    std::string command = ((LearningModel*)model_)->GetPythonExecutionStringForJointTraining(h);
    std::string result = exec(command.c_str());
    std::cout << result << std::endl;
    double seen_scenario_correct;
    double seen_scenario_wrong;
    
    std::istringstream iss(result);
    iss >> seen_scenario_correct;
    iss >> seen_scenario_wrong;
    iss >> next_action;
    //std::cout << seen_scenario_correct;
    //std::cout << seen_scenario_wrong;
    int seen_scenario_correct_int = (int)seen_scenario_correct;
    int seen_scenario_wrong_int = (int)seen_scenario_wrong;
    std::pair<int, int> ans;
    ans.first = seen_scenario_correct_int;
    ans.second = seen_scenario_wrong_int;
    return ans;
}

std::pair<int, int> LearningPlanningSolver::GetSVMOutputUsingPythonFunction(History h) {
    int seen_scenario_correct_int = 1;
    int seen_scenario_wrong_int = -1;
    next_action = -1;
    if(h.Size() > 0)
    {

    ValuedAction ans = deepLearningSolver.Search(h, 0);
    next_action = ans.action;
    PyObject * rnn_output = deepLearningSolver.rnn_output_history.back();
    PyObject *ans0 = get_svm_model_output(rnn_output, 0);
    seen_scenario_correct_int = PyInt_AsLong(ans0);
    std::cout<< seen_scenario_correct_int << std::endl;
    Py_DECREF(ans0);
    if(SecondSVMRequired())
    {
        PyObject *ans1 = get_svm_model_output(rnn_output, 1);
        seen_scenario_wrong_int = PyInt_AsLong(ans1);
        std::cout<< seen_scenario_wrong_int << std::endl;
        Py_DECREF(ans1);
    }
    }
    std::pair<int, int> ans2;
    ans2.first = seen_scenario_correct_int;
    ans2.second = seen_scenario_wrong_int;
    return ans2;
}

void LearningPlanningSolver::load_svm_model(std::string svm_model_prefix) {

    PyObject *pArgs, *pValue;
    PyObject *ans;
   
    char* prev_directory = getcwd(NULL, 0);
    char *directory = "python_scripts/deepLearning";
    int directory_changed = chdir(directory);
    assert(directory_changed == 0);
    pArgs = PyTuple_New(2);
    pValue = PyString_FromString(svm_model_prefix.c_str());
    /* pValue reference stolen here: */
    PyTuple_SetItem(pArgs, 0, pValue);
    /* pValue reference stolen here: */
    pValue = PyString_FromString((((LearningModel*)model_)->problem_name).c_str());
    PyTuple_SetItem(pArgs, 1, pValue);

    ans = PyObject_CallObject(load_function, pArgs);
    Py_DECREF(pArgs);
    if (ans != NULL) {
        std::cout << "Call to load svm model succeeded \n";
    }
    else {

        PyErr_Print();
        fprintf(stderr,"Call to load svm model failed\n");
        assert(0==1);
    }
    correct_svm = PyTuple_GetItem(ans,0);
    Py_INCREF(correct_svm);
    if(SecondSVMRequired())
    {
        wrong_svm = PyTuple_GetItem(ans,1);
        Py_INCREF(wrong_svm);
    }
    Py_DECREF(ans);
    directory_changed = chdir(prev_directory);
    assert(directory_changed == 0);
}

PyObject* LearningPlanningSolver::get_svm_model_output(PyObject* rnn_output, int svm_type) {
    PyObject *ans;
    PyObject *pArgs;
    //std::cout << "Getting svm output" << std::endl;
    pArgs = PyTuple_New(3);
    PyObject *pValue = PyString_FromString((((LearningModel*)model_)->problem_name).c_str());
    PyTuple_SetItem(pArgs, 0, pValue );
    if(svm_type == 0)
    {
        PyTuple_SetItem(pArgs, 1, correct_svm );
        Py_INCREF(correct_svm);
    }
    else
    {
        PyTuple_SetItem(pArgs, 1, wrong_svm );
        Py_INCREF(wrong_svm);
    }
     
    PyTuple_SetItem(pArgs, 2, rnn_output );
    Py_INCREF(rnn_output);

    ans = PyObject_CallObject(run_function, pArgs);
    Py_DECREF(pArgs);
    //Call python function to give correct_svm output
    if (ans != NULL) {
        //std::cout << "Call to run svm succeeded \n";
    }
    else {

        PyErr_Print();
        fprintf(stderr,"Call to run svm failed\n");
        assert(0==1);
    }
    
    return ans;
}


bool LearningPlanningSolver::SecondSVMRequired() {
    return switching_method == 1;
}




