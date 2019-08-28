#ifndef __PARTICLEFILTER_H__
#define __PARTICLEFILTER_H__

#include "incl.h"
#include "model.h"

using namespace std; 

class ParticleFilter 
{
private:

    static int m_cov;

    bool m_init;
    int m_step;
    int m_iter;
    int m_particlesSize;
    vector<Model> m_particles;   
    vector<float> m_weight; 
    float m_totalWeight;

public:

    ParticleFilter();
    ParticleFilter(int numOfParticles, int numOfStep);
    void calAverage(float& x, float& y);
    void priorUpdate(Model& car, const float& acc, const float& steeringAngle);
    void assignWeight(const Model& car);
    void resample();
    vector<float> implement(Model& car);
    int getStep() const; 
};

#endif