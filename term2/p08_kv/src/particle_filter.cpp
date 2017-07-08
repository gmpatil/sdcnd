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
#include "map.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  
  // 500particles  passed (error x .110, y .103, yaw .004)
  // 50particles  passed (error x .120, y .116, yaw .004)
  num_particles = 500;  
  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];

  //particles.clear();
  particles.reserve(num_particles);
  weights.reserve(num_particles);

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
    
    weights.push_back(weight);
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

  // Associate each observed value to nearest predicted landmark.
  for (int i = 0; i < observations.size(); i++) {
    double min_dist = 1e9;
    for (int j = 0; j < predicted.size(); j++) {
      double dst = dist(predicted[j].x, predicted[j].y, observations[i].x, observations[i].y);
      if (dst < min_dist) {
        min_dist = dst;
        observations[i].id = j;
      }
    }
  }
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

  //

  int num_obs = observations.size();
  int num_landmarks = map_landmarks.landmark_list.size();
  
  double std_x = std_landmark[0];
  double std_y = std_landmark[1];
  double bi_variate_pdf_const = (1.0/(2 * M_PI * std_x * std_y)) ;  
  
  std::vector<LandmarkObs> obss_in_map_coord;
  obss_in_map_coord.reserve( num_obs);

  std::vector<LandmarkObs> landmarks_in_range;

  for (int p = 0; p < num_particles; p++){
    double p_x = particles[p].x;
    double p_y = particles[p].y;
    double p_theta = particles[p].theta;

    // convert vehicle observation to map observation for each particle.
    obss_in_map_coord.clear();
    for (int obs_num = 0; obs_num < num_obs; obs_num++){
      LandmarkObs obs_in_vehicle_coord = observations[obs_num];

      LandmarkObs obs_in_map_coord;
      obs_in_map_coord.id = obs_in_vehicle_coord.id;
      obs_in_map_coord.x = p_x + obs_in_vehicle_coord.x * cos(p_theta)
              - obs_in_vehicle_coord.y * sin(p_theta);
      obs_in_map_coord.y = p_y + obs_in_vehicle_coord.x * sin(p_theta)
              + obs_in_vehicle_coord.y * cos(p_theta);
      obss_in_map_coord.push_back(obs_in_map_coord);
    }

    // Filter landmarks within range
    landmarks_in_range.clear();

    for (int landmark_num = 0; landmark_num < num_landmarks; landmark_num++ ) {
      Map::single_landmark_s map_landmark = map_landmarks.landmark_list[landmark_num];

      if (dist(p_x, p_y, map_landmarks.landmark_list[landmark_num].x_f,
              map_landmarks.landmark_list[landmark_num].y_f) <= sensor_range){
        LandmarkObs landmark = {map_landmark.id_i, map_landmark.x_f, map_landmark.y_f};
        landmarks_in_range.push_back(landmark);
      }
    }


    // Associate observations to in range landmarks.
    dataAssociation(landmarks_in_range, obss_in_map_coord);

    double particle_weight = 1.0;
    
    // update weights
    for (int i = 0; i < obss_in_map_coord.size(); i++) {
      LandmarkObs closest_land_mark = landmarks_in_range[obss_in_map_coord[i].id] ;
      double x_diff_by_std_sq = pow( (closest_land_mark.x - obss_in_map_coord[i].x)/std_x, 2) ;
      double y_diff_by_std_sq = pow( (closest_land_mark.y - obss_in_map_coord[i].y)/std_y, 2) ;
      double expo_part = exp(-0.5 * (x_diff_by_std_sq + y_diff_by_std_sq));
      double obs_prob = bi_variate_pdf_const * expo_part;
      particle_weight *= obs_prob; 
    }
    
    particles[p].weight = particle_weight;
    weights[p] = particle_weight;    
  }

  return ;
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  
  // discrete_distribution will return weights' index randomly but in proportion to weight. 
	std::discrete_distribution<int> disc_dist(weights.begin(), weights.end()); 
	std::vector<Particle> resampled_particles; 
	std::default_random_engine rand_eng;
	
	for (int i = 0; i< num_particles; i++){
		resampled_particles.push_back(std::move(particles[disc_dist(rand_eng)]));
	}
  
	particles = std::move(resampled_particles);
  
  return ;
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
