/* 
 * File:   log_file_reader.cpp
 * Author: neha
 * 
 * Created on November 12, 2016, 4:48 PM
 */

#include "log_file_reader.h"
#include "Python.h"

HistoryWithReward log_file_reader::getHistoryWithReward() {

    HistoryWithReward h;
    return h;
    std::ifstream logFile;
    logFile.open(logFileName);
    Py_Initialize();
    //while(!logFile.eof())
    //{
        
    //}
}

void log_file_reader::testPythonCall() {
    
    PyObject *pName, *pModule, *pDict, *pFunc;
    PyObject *pArgs, *pValue;
    int i;

    //Py_SetProgramName(argv[0]);
    Py_Initialize();
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append(\".\")");
    PyRun_SimpleString("sys.path.append('/home/neha/WORK_FOLDER/phd2013/phdTopic/despot/python_scripts/')");
    pName = PyString_FromString("test_python_call");
    /* Error checking of pName left out */

    pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (pModule != NULL) {
        pFunc = PyObject_GetAttrString(pModule, "multiply");
        /* pFunc is a new reference */

        if (pFunc && PyCallable_Check(pFunc)) {
            pArgs = PyTuple_New(2);
            for (i = 0; i < 2; ++i) {
                pValue = PyInt_FromLong(3);
                if (!pValue) {
                    Py_DECREF(pArgs);
                    Py_DECREF(pModule);
                    fprintf(stderr, "Cannot convert argument\n");
                    return ;
                }
                /* pValue reference stolen here: */
                PyTuple_SetItem(pArgs, i, pValue);
            }
            pValue = PyObject_CallObject(pFunc, pArgs);
            Py_DECREF(pArgs);
            if (pValue != NULL) {
                printf("Result of call: %ld\n", PyInt_AsLong(pValue));
                Py_DECREF(pValue);
            }
            else {
                Py_DECREF(pFunc);
                Py_DECREF(pModule);
                PyErr_Print();
                fprintf(stderr,"Call failed\n");
                return ;
            }
        }
        else {
            if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"multiply\"\n");
        }
        Py_XDECREF(pFunc);
        Py_DECREF(pModule);
    }
    else {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"test_python_call\"\n");
        return ;
    }
    Py_Finalize();
}
