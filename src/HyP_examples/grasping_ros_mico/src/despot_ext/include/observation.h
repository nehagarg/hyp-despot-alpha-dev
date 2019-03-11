/* 
 * File:   observation.h
 * Author: neha
 *
 * Created on February 9, 2015, 3:47 PM
 */

#ifndef OBSERVATION_H
#define	OBSERVATION_H

class ObservationClass {
public:
    ObservationClass() {};
    ObservationClass(uint64_t obs_)
    {
        obs = obs_;
    }
    uint64_t GetHash() const {return obs;     };
    void SetIntObs(uint64_t obs_) { obs = obs_;};
private:
  uint64_t obs;  
};

#endif	/* OBSERVATION_H */

