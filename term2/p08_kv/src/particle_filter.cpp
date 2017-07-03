/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  num_particles = 50;
  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];
  
  //particles.clear();
  particles.reserve(num_particles);
  
  std::default_random_engine rand_engine;
  // random device 
  // std::random_device rd; 
  // Mersenne twister PRNG, initialized with seed from previous random device instance
  // std::mt19937 gen(rd()); 
  
  std::normal_distribution<double> norm_dist_x(x, std_x);
  std::normal_distribution<double> norm_dist_y(y, std_y);
  std::normal_distribution<double> norm_dist_theta(theta, std_theta);
    
  double weight = 1.0;
  
  for (int i=0; i < num_particles; i++){
    Particle p = {i, norm_dist_x(rand_engine), norm_dist_y(rand_engine), 
    norm_dist_theta(rand_engine), weight};
    particles.push_back(p);
  }  
  
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
  double std_x = std_pos[0];
  double std_y = std_pos[1];
  double std_theta = std_pos[2];

  std::normal_distribution<double> norm_dist_x(0, std_x);
  std::normal_distribution<double> norm_dist_y(0, std_y);
  std::normal_distribution<double> norm_dist_theta(0, std_theta);
  
  std::default_random_engine rand_engine;
  
  for (int i=0; i < num_particles; i++){
    double x = particles[i].x;
    double y = particles[i].y;
    double theta = particles[i].theta;
    
    // avoid division by zero
    if (fabs(yaw_rate > 0.0001)){
      particles[i].x = x + (velocity/yaw_rate) * (sin(theta + yaw_rate * delta_t) - sin(theta));
      particles[i].y = y + (velocity/yaw_rate) * (cos(theta) - cos(theta + yaw_rate * delta_t));
      particles[i].theta = theta + yaw_rate * delta_t;
    } else {
      particles[i].x = x + velocity * cos(theta) * delta_t;
      particles[i].y = y + velocity * sin(theta) * delta_t;
      particles[i].theta = theta + yaw_rate * delta_t;      
    }
    
    // Add Gaussian noise
    particles[i].x += norm_dist_x(rand_engine);
    particles[i].y += norm_dist_y(rand_engine);
    particles[i].theta += norm_dist_theta(rand_engine);
  }  
  
  return;
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
