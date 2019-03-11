/* 
 * File:   ScikitModels.cpp
 * Author: neha
 * 
 * Created on November 17, 2017, 7:17 PM
 */

#include <vector>
#include <string>

#include "ScikitModels.h"
#include "yaml-cpp/yaml.h"
#include <string>
#include <fstream>

ScikitModels::ScikitModels(std::string model_type_, std::string yaml_file) {
    model_type = model_type_;
}

ScikitModels::ScikitModels(const ScikitModels& orig) {
}

ScikitModels::~ScikitModels() {
}

MultiScikitModels::MultiScikitModels(std::string model_dir, std::string classifier_type, int action, int num_predictions) {
    for(int i = -1; i < num_predictions; i++)
    {
        std::string model_filename = model_dir + "/" + classifier_type + "-" + std::to_string(action);
        if(i==-1)
        {
            model_filename = model_filename + ".yaml";
        }
        else{
            model_filename = model_filename + "-" + std::to_string(i) + ".yaml";
        }
        std::ifstream infile(model_filename.c_str());
        if(infile.good())
        {
            ScikitModels* m;
            if(classifier_type == "linear")
            {
               m = new LinearScikitModel(classifier_type,model_filename);
                
            }
            if(classifier_type == "DTR" or classifier_type == "DTRM")
            {
               m = new DecisionTreeScikitModel(classifier_type,model_filename);
            }
            if(classifier_type == "Earth")
            {
                m = new EarthModel(classifier_type, model_filename);
            }
            m->loadModel(model_filename);
            models.push_back(m);
        }
    }
}

std::vector<double> MultiScikitModels::predict(std::vector<double> x, bool debug) {
    
    if(models.size() ==1)
    {
        return models[0]->predict(x);
    }
    else
    {
        std::vector<double>  ans;
        for(int i = 0; i < models.size(); i++)
        {
            double v = models[i]->predict(x)[0];
            ans.push_back(v);
        }
        return ans;
    }
}


void DecisionTreeScikitModel::loadModel(std::string yaml_file) {
    YAML::Node model_params = YAML::LoadFile(yaml_file);
    
    max_depth = model_params["max_depth"].as<int>();
    n_nodes = model_params["n_nodes"].as<int>();
    assert(model_params["children_left"].size() == n_nodes);
    for(int i = 0; i < n_nodes; i++)
    {
        children_left.push_back(model_params["children_left"][i].as<int>());
        children_right.push_back(model_params["children_right"][i].as<int>());
        feature.push_back(model_params["feature"][i].as<int>());
        threshold.push_back(model_params["threshold"][i].as<double>());
        std::vector <double> value_;
        for(int j = 0; j< model_params["values"][i].size(); j++)
        {
            value_.push_back(model_params["values"][i][j][0].as<double>());
        }
        value.push_back(value_);
    }
}

void LinearScikitModel::loadModel(std::string yaml_file) {
    YAML::Node model_params = YAML::LoadFile(yaml_file);
    intercept = model_params["intercept"].as<double>();
    for(int i = 0; i < model_params["coef"].size(); i++)
    {
        coeff.push_back(model_params["coef"][i].as<double>());
    }

     
           
            
}

void EarthModel::loadModel(std::string yaml_file) {
    YAML::Node model_params = YAML::LoadFile(yaml_file);
    for(int i = 0; i < model_params["coef"].size(); i++)
    {
        coeff.push_back(model_params["coef"][i].as<double>());
        bf_knot.push_back(model_params["bf_knot"][i].as<double>());
        bf_variable.push_back(model_params["bf_variable"][i].as<int>());
        bf_type.push_back(model_params["bf_type"][i].as<std::string>());
        bf_reverse.push_back(model_params["bf_reverse"][i].as<bool>());
    }
}


int DecisionTreeScikitModel::apply(std::vector<double> x) {
    int node_id = 0;
    while(feature[node_id]>=0)
    {
        if(x[feature[node_id]] <= threshold[node_id])
        {
            node_id = children_left[node_id];
        }
        else
        {
            node_id = children_right[node_id];
        }        
    }
    return node_id;

}

std::vector<double>  DecisionTreeScikitModel::predict(std::vector<double> x, bool debug) {
    int node_id = apply(x);
    return value[node_id];
    
}

std::vector<double>  LinearScikitModel::predict(std::vector<double> x, bool debug) {

    std::vector<double> ngs_vec;
    double prob = intercept;
    for(int i = 0; i < coeff.size(); i++)
        {
            prob = prob+ (coeff[i]*x[i]);
        }

        prob = -1*prob;
        prob = exp(prob);
        prob = prob+1;
        prob = 1.0/prob;
        ngs_vec.push_back(1-prob);
        ngs_vec.push_back(prob);
        
        return ngs_vec;
}

std::vector<double> EarthModel::predict(std::vector<double> x, bool debug) {
    std::vector<double> ngs_vec;
    double ans = 0.0;
    for(int i = 0; i < coeff.size(); i++)
    {
        double val;
        if(bf_type[i]=="Intercept")
        {
            val = 1.0;
        }
        else
        {
            if(bf_reverse[i])
            {
                if(x[bf_variable[i]]>bf_knot[i])
                {
                    val = 0.0;   
                }
                else
                {
                    val = bf_knot[i] - x[bf_variable[i]];
                }
                    
            }
            else
            {
                if(x[bf_variable[i]]<=bf_knot[i])
                {
                    val = 0.0;  
                }
                else
                {
                     val = x[bf_variable[i]] - bf_knot[i];
                }
            }
        }
        ans = ans + (coeff[i]*val);
    }
    
    ngs_vec.push_back(ans);
    return ngs_vec;
}

