/* 
 * File:   DeepLearningSolver.cpp
 * Author: neha
 * 
 * Created on August 25, 2016, 11:49 AM
 */

#include "DeepLearningSolver.h"
#include <despot/interface/lower_bound.h>
#include <unistd.h>

DeepLearningSolver::DeepLearningSolver(const LearningModel* model, Belief* belief) : LearningSolverBase(model, belief) {
    
    

    //Py_SetProgramName(argv[0]);
    Py_Initialize();
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('python_scripts')");
    PyRun_SimpleString("sys.path.append('python_scripts/deepLearning')");
    char ** argv;
    //PySys_SetArgv(0, argv);
    PySys_SetArgvEx(0, argv, 0);
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
    load_function = PyObject_GetAttrString(pModule, "get_learning_model");
    if (!(load_function && PyCallable_Check(load_function)))
    {
        if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"get_learning_model\"\n");
    }
    run_function = PyObject_GetAttrString(pModule, "get_output_learning_model");
    if (!(run_function && PyCallable_Check(run_function)))
    {
        if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"get_learning_model_output\"\n");
    }
    
    Py_DECREF(pModule);
    load_deep_policy(model->learned_model_name);
    //cout << "Initializing adaboost solver" << endl;
    //Currently id mapping done manually 
    //Ideally should be done by parsing .data file in the model
    /*int a1 = 6; adaboostIdsToModelIds.push_back(a1);
    int a2 = 4; adaboostIdsToModelIds.push_back(a2);
    int a3 = 7; adaboostIdsToModelIds.push_back(a3);
    int a4 = 3; adaboostIdsToModelIds.push_back(a4);
    int a5 = 1; adaboostIdsToModelIds.push_back(a5);
    int a6 = 9; adaboostIdsToModelIds.push_back(a6);
    int a7 = 2; adaboostIdsToModelIds.push_back(a7);
    int a8 = 8; adaboostIdsToModelIds.push_back(a8);
    int a9 = 5; adaboostIdsToModelIds.push_back(a9);
    int a10 = 0; adaboostIdsToModelIds.push_back(a10);
    */
    //cout << "Initialized adaboost solver" << endl;
}



DeepLearningSolver::~DeepLearningSolver() {
    Py_Finalize();
}

/*template <typename T>
std::string to_string(T const& value) {
    std::stringstream sstr;
    sstr << value;
    return sstr.str();
}*/


PyObject* DeepLearningSolver::get_state_and_next_action(PyObject* rnn_state, int action, uint64_t obs, int print_info) {
    PyObject *ans;
    History h;
    if (action >=0)
    {
        h.Add(action, obs);
    }
    std::ostringstream oss;
    ((LearningModel*)model_)->GetInputSequenceForLearnedmodel(h, oss);
    
    PyObject *pArgs;
    if(rnn_state != NULL)
    {
       // std::cout << "Rnn state is not null " << std::endl;
     pArgs = PyTuple_New(5);
     PyTuple_SetItem(pArgs, 4, rnn_state );
     Py_INCREF(rnn_state);
    }
    else
    {
        //std::cout << "Rnn state is null " << std::endl;
       pArgs = PyTuple_New(4); 
    }
    PyObject *pValue = PyString_FromString((((LearningModel*)model_)->problem_name).c_str());
    PyTuple_SetItem(pArgs, 0, pValue );
    PyTuple_SetItem(pArgs, 1, h_to_a_model );
    Py_INCREF(h_to_a_model);
    pValue = PyString_FromString(oss.str().c_str());
    PyTuple_SetItem(pArgs, 2, pValue );
    pValue = PyInt_FromLong(print_info);
    PyTuple_SetItem(pArgs, 3, pValue );
    ans = PyObject_CallObject(run_function, pArgs);
    
    
    Py_DECREF(pArgs);
    //Call python function to give rnn state and next action
    if (ans != NULL) {
        //std::cout << "Call to run model succeeded \n";
    }
    else {

        PyErr_Print();
        fprintf(stderr,"Call to run model failed\n");
        assert(0==1);
    }
    return ans;
}

void DeepLearningSolver::load_deep_policy(std::string learned_model_name) {
    
    PyObject *pArgs, *pValue;
   
    char* prev_directory = getcwd(NULL, 0);
    char *directory = "python_scripts/deepLearning";
    int directory_changed = chdir(directory);
    assert(directory_changed == 0);
    pArgs = PyTuple_New(2);
    pValue = PyString_FromString(learned_model_name.c_str());
    /* pValue reference stolen here: */
    PyTuple_SetItem(pArgs, 0, pValue);
    /* pValue reference stolen here: */
    pValue = PyString_FromString((((LearningModel*)model_)->problem_name).c_str());
    PyTuple_SetItem(pArgs, 1, pValue);

    h_to_a_model = PyObject_CallObject(load_function, pArgs);
    Py_DECREF(pArgs);
    if (h_to_a_model != NULL) {
        std::cout << "Call to load model succeeded \n";
    }
    else {

        PyErr_Print();
        fprintf(stderr,"Call to load model failed\n");
        assert(0==1);
    }
    
    directory_changed = chdir(prev_directory);
    assert(directory_changed == 0);
        
}





ValuedAction DeepLearningSolver::Search() {
    std::cout << "Starting search" << std::endl;
    
   // std::string cmd_string = "cd python_scripts/deepLearning ; python model.py " + to_string((int)((LearningModel*)model_)->GetStartStateIndex()) + "; cd - ;";
    
    
    return Search(history_);
}

void DeepLearningSolver::TruncateRnnHistories(int history_size) {
    //std::cout << "In truncate " << rnn_state_history.size() << ","
    //        << rnn_output_history.size() << std::endl;
    assert(history_size <= rnn_state_history.size());
    for(int i = history_size; i< rnn_state_history.size(); i++)
    {
        Py_DECREF(rnn_state_history[i]);
        Py_DECREF(rnn_output_history[i]);
         //TODO delete PyObjects properly
    }
    rnn_state_history.resize(history_size);
    rnn_output_history.resize(history_size);
}

ValuedAction DeepLearningSolver::SearchUsingPythonFunction(History h, int print_info) {
    if(print_info == 1)
    {
        std::cout << "Before calling exec" << std::endl; //For counting how many times learned call is used
    }
//    std::cout << "Rnn state history size and history size " << rnn_state_history.size() << "," << h.Size() << std::endl;
    PyObject *rnn_state = NULL;
    int history_size = h.Size();
    TruncateRnnHistories(history_size);
    
    int action = -1;
    uint64_t obs;
    if(history_size > 0){
        if(print_info ==1)
        {
        std::cout << "History size is " << history_size <<  std::endl;
        }
        rnn_state = rnn_state_history[history_size -1 ];
        if(rnn_state == NULL)
        {
            std::cout << "failed ERROR: Null Rnn state history size and history size " << rnn_state_history.size() << "," << h.Size() << std::endl;
   
        }
        action = h.LastAction();
        obs = h.LastObservation();
    }
    PyObject *new_rnn_state;
    PyObject *new_rnn_output;
    PyObject *ans;
    
    ans = get_state_and_next_action(rnn_state, action, obs, print_info);
    action = (int) PyInt_AsLong(PyTuple_GetItem(ans, 0));
    new_rnn_state = PyTuple_GetItem(ans, 1);
    Py_INCREF(new_rnn_state);
    rnn_state_history.push_back(new_rnn_state);
    
    new_rnn_output = PyTuple_GetItem(ans, 2);
    Py_INCREF(new_rnn_output);
    rnn_output_history.push_back(new_rnn_output);
    
    Py_DECREF(ans);
    double value = 1;
    return ValuedAction(action, value);
}

ValuedAction DeepLearningSolver::SearchUsingCommandLineCall(History h) {
        std::string cmd_string = ((LearningModel*)model_)->GetPythonExecutionString(h);
    std::cout << "Before calling exec" << std::endl;
    std::string result = exec(cmd_string.c_str());
    std::cout << result << std::endl;
    int action = 0;
    double  value = -100000;
    //parse result and generate action value
    std::istringstream iss(result);
    iss >> action;
    std::cout << "Action is " << action << std::endl;
    /*int num_actions = model_->NumActions();
    for(int i = 0; i < num_actions; i++)
    {
        double current_value;
        iss >> current_value;
        if (i == action)
        {
            value = current_value;
        }
        
        
    }*/
    value = 1;
    return ValuedAction(action, value);
    //For toy problem
    //return ValuedAction(deepLearningIdsToModelIds[action], value);
}

ValuedAction DeepLearningSolver::Search(History h, int print_info) {
    
    ValuedAction ans1 = SearchUsingPythonFunction(h, print_info);
    //ValuedAction ans2 = SearchUsingCommandLineCall(h);
    //assert(ans1.action == ans2.action);
    return ans1;
    
}

void DeepLearningSolver::Update(int action, uint64_t obs) {
    
    std::cout << "Deep Learnign Update: Rnn state history size and history size " 
            << rnn_state_history.size() << "," << history_.Size() << std::endl;

    if(rnn_state_history.size() == history_.Size()) 
        //rnn_state_history size should be 1 more than history size before updating history
    {
        //perfor search to update rnn_state_history
        std::cout<<"Updating rnn state history" << std::endl;
        Search(history_, 0);
    }
    history_.Add(action, obs);
    TruncateRnnHistories(history_.Size());
}



