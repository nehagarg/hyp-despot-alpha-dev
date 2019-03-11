/* 
 * File:   ScikitModels.h
 * Author: neha
 *
 * Created on November 17, 2017, 7:17 PM
 */

#ifndef SCIKITMODELS_H
#define	SCIKITMODELS_H


class ScikitModels {
public:
    ScikitModels(std::string model_type, std::string yaml_file);
    ScikitModels(const ScikitModels& orig);
    virtual ~ScikitModels();
    virtual void loadModel(std::string yaml_file) = 0;
    virtual std::vector<double>  predict(std::vector<double> x, bool debug=false) = 0;
private:
    std::string model_type;
    
};

class MultiScikitModels {
public:
    MultiScikitModels(std::string model_dir,  std::string classifier_type, int action, int num_predictions);
    virtual ~MultiScikitModels() {
    }
    virtual  std::vector<double>  predict(std::vector<double> x, bool debug=false);
private:
    std::vector<ScikitModels*> models;
};

class DecisionTreeScikitModel : public ScikitModels {
public:
    DecisionTreeScikitModel(std::string model_type, std::string yaml_file) : ScikitModels(model_type, yaml_file){};
    virtual ~DecisionTreeScikitModel() {};

    void loadModel(std::string yaml_file);

    std::vector<double>  predict(std::vector<double> x, bool debug=false);


private:
    int n_nodes ;
    int max_depth;
    std::vector<int> children_left ;
    std::vector<int>children_right;
    std::vector<int>feature ;
    std::vector<double>threshold;
    std::vector<std::vector<double> >  value;
    
    int apply(std::vector<double> x);
    
    
};

class LinearScikitModel : public ScikitModels {
public:
    LinearScikitModel(std::string model_type, std::string yaml_file): ScikitModels(model_type, yaml_file){};
    virtual ~LinearScikitModel() {};
    
    void loadModel(std::string yaml_file);

    std::vector<double> predict(std::vector<double> x, bool debug=false);



private:
    std::vector<double>coeff;
    double intercept;
};

class EarthModel : public ScikitModels {
    
public:
    EarthModel(std::string model_type, std::string yaml_file): ScikitModels(model_type, yaml_file) {};
    virtual ~EarthModel() {
    }

    virtual void loadModel(std::string yaml_file);

    virtual std::vector<double> predict(std::vector<double> x, bool debug=false);
private:
    std::vector<double>coeff;
    std::vector<double>bf_knot;
    std::vector<int>bf_variable;
    std::vector<bool>bf_reverse;
    std::vector<std::string> bf_type;

};

#endif	/* SCIKITMODELS_H */

