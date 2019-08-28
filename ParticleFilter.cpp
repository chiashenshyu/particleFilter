#include "ParticleFilter.h"

int ParticleFilter::m_cov = 100;

ParticleFilter::ParticleFilter()
{
    m_init = false;
    m_iter = 1;
    m_step = 100; 
    m_particlesSize = 1000;
    m_weight.resize(m_particlesSize);
}

ParticleFilter::ParticleFilter(int numOfParticles, int numOfStep)
:m_particlesSize(numOfParticles),
 m_step(numOfStep)
{
    m_init = false; 
    m_iter = 1;
    m_weight.resize(m_particlesSize);
}

void ParticleFilter::calAverage(float& x, float& y){
    float x_tot = 0, y_tot = 0; 
    for(auto p : m_particles){
        vector<float> pos = p.getPos(); 
        x_tot += pos[0];
        y_tot += pos[1];
    }
    x = x_tot / m_particles.size();
    y = y_tot / m_particles.size(); 
}

void ParticleFilter::priorUpdate(Model& car, const float& acc, const float& steeringAngle)
{
    default_random_engine generator;
    normal_distribution<float> distribution(60, 1); // (center, std)
    if(!m_init){
        m_init = true;
        for(int i = 0; i < m_particlesSize; i++){
            Model a = car; 
            a.setIcNoise(distribution(generator));
            m_particles.push_back(a); 
        }
    }else{
        car.move(acc, steeringAngle);
        for(int i = 0; i < m_particlesSize; i++){
            m_particles[i].move(acc, steeringAngle);
        }
    }
}

void ParticleFilter::assignWeight(const Model& car)
{
    m_totalWeight = 0; 
    float x1 = car.getPos()[0], y1 = car.getPos()[1], x2, y2, delta; 
    for(int i = 0; i < m_particlesSize; i++){
        x2 = m_particles[i].getPos()[0];
        y2 = m_particles[i].getPos()[1];
        delta = sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
        m_weight[i] = 1 / sqrt(2 * M_PI * m_cov) * exp(-(pow(delta, 2) / (2 * m_cov))); 
        m_totalWeight += m_weight[i];
    }
}

void ParticleFilter::resample()
{
    default_random_engine generator(time(0)); 
    uniform_real_distribution<float> distribution(0.0, m_totalWeight);
    normal_distribution<float> d(0.0,1.0);
    vector<Model> tmp;
    // cout << "weight" << m_totalWeight << endl; 
    for(int i = 0; i < m_particlesSize; i++){
        float threshold = distribution(generator);
        float accu = 0; 
        int iter = 0; 
        while(accu < threshold){
            accu += m_weight[iter++];
        }
        Model _p = (iter != 0)? m_particles[--iter] : m_particles[iter];
        _p.setIcNoise(d(generator));
        tmp.push_back(_p);
    }
    m_particles = tmp; 
}

vector<float> ParticleFilter::implement(Model& car)
{
    if(!m_init){
        priorUpdate(car, car.acc, car.steeringAngle);
    }
    //pf.priorUpdate(ff91, particles);
    priorUpdate(car, car.acc, car.steeringAngle);
    assignWeight(car);
    resample();

    float x, y; 
    calAverage(x, y);
    double err = sqrt(pow(x - car.getPos()[0], 2) + pow(y - car.getPos()[1], 2));

    vector<float> estimation = {x, y, (float)err};
    
    return estimation;
}

int ParticleFilter::getStep() const
{
    return m_step;
}