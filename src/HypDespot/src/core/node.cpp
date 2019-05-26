#include <despot/core/node.h>
#include <despot/solver/despot.h>

using namespace std;

namespace despot {

/* =============================================================================
 * VNode class
 * =============================================================================*/


VNode::VNode(vector<State*>& particles,std::vector<int> particleIDs, int depth, QNode* parent,
	OBS_TYPE edge) :
	particles_(particles),
	particleIDs_(particleIDs),
	GPU_particles_(NULL),
	num_GPU_particles_(0),
	belief_(NULL),
	depth_(depth),
	parent_(parent),
	edge_(edge),
	vstar(this),
	likelihood(1),
        extra_node(false),
        common_parent_(NULL),
        obs_probs_holder(NULL) {
	logd << "Constructed vnode with " << particles_.size() << " particles"
		<< endl;
	for (int i = 0; i < particles_.size(); i++) {
	  if(particleIDs_.size() > i)
	    {
		logd << " " << i << " = " <<"("<< particleIDs_[i]<<")";
		if(particles_[i]!=NULL) //can be null in case of GPU expansion
		{
			logd << *particles_[i];
		}
		logd << endl;
	    }
	  else
	    {
	      logd << " " << i << " = " ;
	      if(particles_[i]!=NULL) //can be null in case of GPU expansion
		  {
	    	  logd << *particles_[i];
		  }
	      logd << endl;

	    }
	}
	weight_=0;
        if(Globals::config.track_alpha_vector)
        {
            //common_data_holder_ = this;
            common_parent_ = new QNode(particles, particleIDs);
            particle_weights.resize(Globals::config.num_scenarios, 0);
            for (int i = 0; i < common_parent_->particles_.size(); i++) {
		particle_weights[particles_[i]->scenario_id] = particles_[i]->weight;
                particles_[i] = NULL;
            }
            particles_.clear();
            prob_o_given_b = 1.0;
            
           //upper_bound_alpha_vector_.resize(Globals::config.num_scenarios, 0);
           //lower_bound_alpha_vector.resize(Globals::config.num_scenarios, 0); 
        }
}

VNode::VNode(int depth, QNode* parent, OBS_TYPE edge):
  GPU_particles_(NULL),
  num_GPU_particles_(0),
  belief_(NULL),
	depth_(depth),
	parent_(parent),
	edge_(edge),
	vstar(this),
	likelihood(1),
            extra_node(false),
            obs_probs_holder(NULL)    {
        if(Globals::config.track_alpha_vector)
        {
            particle_weights.resize(Globals::config.num_scenarios, 0);
            //upper_bound_alpha_vector_.resize(Globals::config.num_scenarios, 0);
            //obs_probs.resize(Globals::config.num_scenarios, 0);
            //observation_particle_size = -1;
        }
	weight_=0; 
    }
    VNode::VNode(int depth, QNode* parent, QNode* common_parent, OBS_TYPE edge):
    VNode(depth, parent, edge){
        common_parent_ = common_parent;
	
    }


void VNode::Initialize(vector<State*>& particles,std::vector<int> particleIDs, int depth, QNode* parent,
	OBS_TYPE edge){
	particles_=particles;
	particleIDs_=particleIDs;
	GPU_particles_=NULL;
	num_GPU_particles_=0;
	belief_=NULL;
	depth_=depth;
	parent_=parent;
	edge_=edge;
	vstar=this;
	likelihood=1;
	weight_=0;
}

VNode::VNode(Belief* belief, int depth, QNode* parent, OBS_TYPE edge) :
	GPU_particles_(NULL),
	num_GPU_particles_(0),
	belief_(belief),
	depth_(depth),
	parent_(parent),
	edge_(edge),
	vstar(this),
	likelihood(1) {
	weight_=0;
}

VNode::VNode(int count, double value, int depth, QNode* parent, OBS_TYPE edge) :
	GPU_particles_(NULL),
	num_GPU_particles_(0),
	belief_(NULL),
	depth_(depth),
	parent_(parent),
	edge_(edge),
	count_(count),
	value_(value) {
	weight_=0;
}


VNode::~VNode() {
	for (int a = 0; a < children_.size(); a++) {
		QNode* child = children_[a];
		assert(child != NULL);
		if(child->IsAllocated())//allocated by memory pool
			DESPOT::qnode_pool_.Free(child);
		else
			delete child;
	}
        if(Globals::config.track_alpha_vector)
        {
            if (parent_== NULL)//
            {
                delete common_parent_;
            }
        }
	children_.clear();

	if (belief_ != NULL)
		delete belief_;
}

Belief* VNode::belief() const {
	return belief_;
}

const std::vector<State*>& VNode::particles() const {
    if(Globals::config.track_alpha_vector)
    {
        
            return common_parent_->particles_;
        
    }
    else
    {
        return particles_;
    }
	//return particles_;
}

const std::vector<int>& VNode::particleIDs() const {
	if(Globals::config.track_alpha_vector)
	    {

	            return common_parent_->particleIDs_;

	    }
	    else
	    {
	        return particleIDs_;
	    }
	//return particleIDs_;
}

const std::vector<float>& VNode::particle_keras_batch() const
{
	if(Globals::config.track_alpha_vector)
		    {

		            return common_parent_->particle_keras_batch;

		    }
		    else
		    {
		        return particle_keras_batch_;
		    }

}

void VNode::set_keras_particle_batch(std::vector<float>& particle_keras_batch)
{
	particle_keras_batch_ = particle_keras_batch;
}
void VNode::depth(int d) {
	depth_ = d;
}

int VNode::depth() const {
	return depth_;
}

void VNode::parent(QNode* parent) {
	parent_ = parent;
}

QNode* VNode::parent() {
	return parent_;
}

OBS_TYPE VNode::edge() {
	return edge_;
}

double VNode::Weight() {
	/*if(Globals::config.useGPU==false ||!PassGPUThreshold())
		if(GPUWeight()>0)
		  {
		    //std::cout << "Returning GPU Weight " << std::endl;
			return GPUWeight();
		  }
		else*/
                {
                    if(Globals::config.track_alpha_vector)
                    {
                        //Can simply return 1 as weight sums up to 1
                        return 1.0;
                        //double w = 0;
                        //for(int i = 0 ; i < particle_weights.size(); i++)
                        //{
                        //    w = w + particle_weights[i];
                        //}
                        //return w;
                    }
                    else
                    {
                        return State::Weight(particles_);
                    }
			//return State::Weight(particles_);
                }
	//else/* if(num_GPU_particles_>0)*/
	//	return GPUWeight();
}
bool VNode::PassGPUThreshold(){
	return (particleIDs().size()>2/*0*/ || depth()<1);
}
const vector<QNode*>& VNode::children() const {
	return children_;
}

vector<QNode*>& VNode::children() {
	return children_;
}

const QNode* VNode::Child(int action) const {
	return children_[action];
}

QNode* VNode::Child(int action) {
	return children_[action];
    }

QNode* VNode::common_parent() {
        return common_parent_;
    }


 const QNode* VNode::CommonChild(int action) const {
     int common_children_size = common_parent_->common_children_.size();
     //logd << "Action " << action << " common_children size " << common_children_size << std::endl;
    if(common_children_size==action)
    {
        QNode* common_qnode = children_[action];
        common_parent_->common_children_.push_back(common_qnode);
    }
    else
    {
        //assert(common_children_size == num_actions);
    }
     
     return common_parent_->common_children_[action];
    }

    QNode* VNode::CommonChild(int action) {
     int common_children_size = common_parent_->common_children_.size();
     //logd << "Action " << action << " common_children size " << common_children_size << std::endl;
     if(common_children_size==action)
    {
        QNode* common_qnode = children_[action];
        common_parent_->common_children_.push_back(common_qnode);
    }
    else
    {
        //assert(common_children_size == num_actions);
    }
     
     return common_parent_->common_children_[action];
    }

int VNode::Size() const {
	int size = 1;
	for (int a = 0; a < children_.size(); a++) {
		size += children_[a]->Size();
	}
	return size;
}

int VNode::PolicyTreeSize() const {
	if (children_.size() == 0)
		return 0;

	QNode* best = NULL;
	for (int a = 0; a < children_.size(); a++) {
		QNode* child = children_[a];
		if (best == NULL || child->lower_bound() > best->lower_bound())
			best = child;
	}
	return best->PolicyTreeSize();
}

void VNode::default_move(ValuedAction move) {
    /*if(Globals::config.track_alpha_vector)
    {

        if(common_parent_ == NULL)
        {
            default_move_ = move;
        }
        else
        {

            common_parent_->default_move = move;
        }
        
        
    }
    else
    {*/
	default_move_ = move;
    //}
}

ValuedAction VNode::default_move() const {
 /*   if(Globals::config.track_alpha_vector)
    {
        if(common_parent_ == NULL)
        {
            return default_move_;
        }
        else
        {

            return common_parent_->default_move;
        }
        
    }
    else
    {*/
	return default_move_;
    //}
}

void VNode::lower_bound(double value) {
	lower_bound_ = value;
}

double VNode::lower_bound() const {
	return lower_bound_;
}

void VNode::upper_bound(double value) {
	upper_bound_ = value;
}

double VNode::upper_bound() const {
	return upper_bound_;
}
void VNode::utility_upper_bound(double value){
	utility_upper_bound_=value;
}
double VNode::utility_upper_bound() const {
	return utility_upper_bound_;
}
double VNode::calculate_lower_bound() const {
        double lower_bound = 0;
        //const std::vector<State*>& particles = this->particles();
        //std::cout << "Lower bound alpha vector" << lower_bound_alpha_vector << std::endl;
        //std::cout << "particle_weight_size" << particle_weights.size() << std::endl;
	//double weight_sum = 0;
	for(int i = 0; i < Globals::config.num_scenarios; i++)
        {
          //std::cout << "Particle_weight " << particle_weights[i];
	  //weight_sum += particle_weights[i];
            //int particle_index = particles[i]->scenario_id;
            lower_bound += particle_weights[i]*((*(lower_bound_alpha_vector.value_array))[i]);
        }
	//std::cout << "Particle weight is " << weight_sum << " for " << this->particles().size() << " partciles" << std::endl;
        //std::cout << "Lower bound is " << lower_bound << std::endl;
        return lower_bound;
    }
    
    double VNode::calculate_upper_bound() const {
        double upper_bound = 0;
        //const std::vector<State*>& particles = this->particles();
        for(int i = 0; i < Globals::config.num_scenarios; i++)
        {
          //  std::cout << "Particle_weight " << particle_weights[i];
            //int particle_index = particles[i]->scenario_id;            
            upper_bound += particle_weights[i]* (common_parent_->default_upper_bound_alpha_vector[i]);
        }
        //std::cout << "Calculating upper bound " << upper_bound << upper_bound_alpha_vector << std::endl;
        return upper_bound;
    }
bool VNode::IsLeaf() {
	return children_.size() == 0;
}

void VNode::Add(double val) {
	value_ = (value_ * count_ + val) / (count_ + 1);
	count_++;
}

void VNode::count(int c) {
	count_ = c;
}
int VNode::count() const {
	return count_;
}
void VNode::value(double v) {
	value_ = v;
}
double VNode::value() const {
	return value_;
}

void VNode::Free(const DSPOMDP& model) {
	for (int i = 0; i < particles_.size(); i++) {
		if(particles_[i])
		  { model.Free(particles_[i]);
		    //particles_[i] = NULL;
		  }
	}
        if(Globals::config.track_alpha_vector)
        {
            for (int i = 0; i < common_parent_->particles_.size(); i++) {
		if(common_parent_->particles_[i])
		  {
		    model.Free(common_parent_->particles_[i]);
		    common_parent_->particles_[i] = NULL;
		  }
            }
        }

	for (int a = 0; a < children().size(); a++) {
		QNode* qnode = Child(a);
		map<OBS_TYPE, VNode*>& children = qnode->children();
		for (map<OBS_TYPE, VNode*>::iterator it = children.begin();
			it != children.end(); it++) {
			it->second->Free(model);
		}
	}
}

void VNode::PrintPolicyTree(int depth, ostream& os) {
	if (depth != -1 && this->depth() > depth)
		return;

	vector<QNode*>& qnodes = children();
	if (qnodes.size() == 0) {
		int astar = this->default_move().action;
		os << this << "-a=" << astar << endl;
	} else {
		QNode* qstar = NULL;
		for (int a = 0; a < qnodes.size(); a++) {
			QNode* qnode = qnodes[a];
			if (qstar == NULL || qnode->lower_bound() > qstar->lower_bound()) {
				qstar = qnode;
			}
		}

		os << this << "-a=" << qstar->edge() << endl;

		vector<OBS_TYPE> labels;
		map<OBS_TYPE, VNode*>& vnodes = qstar->children();
		for (map<OBS_TYPE, VNode*>::iterator it = vnodes.begin();
			it != vnodes.end(); it++) {
			labels.push_back(it->first);
		}

		for (int i = 0; i < labels.size(); i++) {
			if (depth == -1 || this->depth() + 1 <= depth) {
				os << repeat("|   ", this->depth()) << "| o=" << labels[i]
					<< ": ";
				qstar->Child(labels[i])->PrintPolicyTree(depth, os);
			}
		}
	}
}

void VNode::PrintTree(int depth, ostream& os) {
	if (depth != -1 && this->depth() > depth)
		return;

	if (this->depth() == 0) {
		os << "d - default value" << endl
			<< "l - lower bound" << endl
			<< "u - upper bound" << endl
			<< "r - totol weighted one step reward" << endl
			<< "w - total particle weight" << endl
		    << "ogb - obs prob" << endl;
	}

	os << "(" << "d:" << this->default_move().value <<
		" l:" << this->lower_bound() << ", u:" << this->upper_bound()
		<< ", w:" << this->Weight() << ", ogb:" << this->prob_o_given_b
		<< ", weu:" << DESPOT::WEU(this)
		<< ")"
		<< endl;


	vector<QNode*>& qnodes = children();
	for (int a = 0; a < qnodes.size(); a++) {
		QNode* qnode = qnodes[a];

		vector<OBS_TYPE> labels;
		map<OBS_TYPE, VNode*>& vnodes = qnode->children();
		for (map<OBS_TYPE, VNode*>::iterator it = vnodes.begin();
			it != vnodes.end(); it++) {
			labels.push_back(it->first);
		}

		os << repeat("|   ", this->depth()) << "a="
			<< qnode->edge() << ": "
			<< "(d:" << qnode->default_value << ", l:" << qnode->lower_bound()
			<< ", u:" << qnode->upper_bound()
			<< ", r:" << qnode->step_reward << ")" << endl;

		for (int i = 0; i < labels.size(); i++) {
			if (depth == -1 || this->depth() + 1 <= depth) {
				os << repeat("|   ", this->depth()) << "| o=" << labels[i]
					<< ": ";
				qnode->Child(labels[i])->PrintTree(depth, os);
			}
		}
	}
}

Dvc_State* VNode::GetGPUparticles(){
		if(Globals::config.track_alpha_vector)
		{
			return common_parent_->GPU_particles_;
		}
		else
		{
			return GPU_particles_;
		}

		//return GPU_particles_;

	};


/* =============================================================================
 * QNode class
 * =============================================================================*/
QNode::QNode() :
	parent_(NULL),
	edge_(-1),
	vstar(NULL) {
	weight_=0;
	//lower_bound_=0;upper_bound_=0;
}

QNode::QNode(VNode* parent, int edge) :
	parent_(parent),
	edge_(edge),
	vstar(NULL) {
	weight_=0;
}

QNode::QNode(int count, double value) :
	count_(count),
	value_(value) {
	weight_=0;
    }

QNode::QNode(std::vector<State*>& particles):
particles_(particles), parent_(NULL){
    weight_ = 0;
    GPU_particles_=NULL;
    	num_GPU_particles_=0;
}

QNode::QNode(std::vector<State*>& particles, std::vector<int> particleIDs):
particles_(particles), particleIDs_(particleIDs), parent_(NULL){
    weight_ = 0;
    GPU_particles_=NULL;
	num_GPU_particles_=0;
}

QNode::~QNode() {
	for (map<OBS_TYPE, VNode*>::iterator it = children_.begin();
		it != children_.end(); it++) {
		assert(it->second != NULL);
		if(it->second->IsAllocated())//allocated by memory pool
			DESPOT::vnode_pool_.Free(it->second);
		else
			delete it->second;
	}
	children_.clear();
}

void QNode::parent(VNode* parent) {
	parent_ = parent;
}

VNode* QNode::parent() {
	return parent_;
}

int QNode::edge() const{
	return edge_;
}

map<OBS_TYPE, VNode*>& QNode::children() {
	return children_;
}

VNode* QNode::Child(OBS_TYPE obs) {
	return children_[obs];
}

int QNode::Size() const {
	int size = 0;
	for (map<OBS_TYPE, VNode*>::const_iterator it = children_.begin();
		it != children_.end(); it++) {
		size += it->second->Size();
	}
	return size;
}

int QNode::PolicyTreeSize() const {
	int size = 0;
	for (map<OBS_TYPE, VNode*>::const_iterator it = children_.begin();
		it != children_.end(); it++) {
		size += it->second->PolicyTreeSize();
	}
	return 1 + size;
}

double QNode::Weight() /*const*/ {
	if(weight_>1e-5){
		//assert((parent()->depth()==0 && weight_>1-1e-5) ||
		//		parent()->depth()!=0 && weight_<1+1e-5);

		/*if(parent()->depth()==0 && weight_<=1-1e-5){
			cout<<"Wrong stored weight as depth 1 " << this << " "<<weight_<<endl;
			//exit(-1);
		}*/

		return weight_;
	}
	else
	{
		weight_=0;
		for (map<OBS_TYPE, VNode*>::const_iterator it = children_.begin();
			it != children_.end(); it++) {
			weight_ += it->second->Weight();
		}

		/*if(parent()->depth()==0 && weight_<=1-1e-5){
			cout<<"Wrong calcuated weight as depth 1 " << this << " "<<weight_<<endl;
			//exit(-1);
		}*/
		return weight_;
	}
}

void QNode::lower_bound(double value) {
	lower_bound_ = value;
}

double QNode::lower_bound() const {
	return lower_bound_;
}

void QNode::upper_bound(double value) {
	upper_bound_ = value;
}

double QNode::upper_bound() const {
	return upper_bound_;
}
void QNode::utility_upper_bound(double value){
	utility_upper_bound_=value;
}
double QNode::utility_upper_bound() const {
	return utility_upper_bound_;
}

void QNode::Add(double val) {
	value_ = (value_ * count_ + val) / (count_ + 1);
	count_++;
}

void QNode::count(int c) {
	count_ = c;
}

int QNode::count() const {
	return count_;
}

void QNode::value(double v) {
	value_ = v;
}

double QNode::value() const {
	return value_;
}

} // namespace despot
