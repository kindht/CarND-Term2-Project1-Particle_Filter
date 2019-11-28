/**
 * particle_filter.cpp
 *
 * Created on: Nov 15, 2019
 * Author: Mian Huang 
 * 
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include<time.h>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;

using std::cout;
using std::endl;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  cout << "Initializing..." << endl;
  // Create normal distributions around the initial position(x,y) & orientation(theta)
  std::default_random_engine gen;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  
  num_particles = 100;  // TODO: Set the number of particles
  for (int i = 0 ; i < num_particles; ++i) {
    // Create a sample Particle from normal distributions 
    Particle sample_p;
    sample_p.id = i;
    sample_p.x = dist_x(gen);
    sample_p.y = dist_y(gen);
    sample_p.theta = dist_theta(gen);
    sample_p.weight = 1.0;
    
    particles.push_back(sample_p);
    weights.push_back(sample_p.weight);
  }//End for

  is_initialized = true;
  cout << "End init" << endl;
} //End init

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
   //cout << "Predicting..." << endl;
   // Previous state(x0, y0, theta_0) & Predicted state (x, y, theta) for a particle
   double x0, y0, theta_0,  x, y, theta;
   
   std::default_random_engine gen;
   
   // Predict new state for each particle
   for (int i = 0; i < num_particles; ++i) {
      x0 = particles[i].x;
      y0 = particles[i].y;
      theta_0 = particles[i].theta;
     
      if (yaw_rate !=0 ) {
        x = x0 + velocity/yaw_rate * (sin(theta_0 + yaw_rate * delta_t)-sin(theta_0));
        y = y0 + velocity/yaw_rate * (cos(theta_0) - cos(theta_0 + yaw_rate * delta_t));
        theta = theta_0 + yaw_rate * delta_t;
      } else {
        x = x0 + velocity * delta_t * cos(theta_0);
        y = y0 + velocity * delta_t * sin(theta_0);
        theta = theta_0;
      }
      
      // Create normal distributions around the predicted position x, y , theta
      normal_distribution<double> dist_x(x, std_pos[0]);
      normal_distribution<double> dist_y(y, std_pos[1]);
      normal_distribution<double> dist_theta(theta, std_pos[2]);

      // Update each particle with predicted state
      particles[i].x = dist_x(gen);
      particles[i].y = dist_y(gen);
      particles[i].theta = dist_theta(gen);
   } // End for
   //cout << "End prediciton" << endl;
} // End prediction



void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  //cout << "Update weights..." << endl;
  vector<Map::single_landmark_s> map_landmarks = map.landmark_list;
   
  weights.clear();  

  // Loop through each particle to calculate weight 
  for (int i=0; i< num_particles; ++i) {
      //Get a reference to each particle
      Particle &p = particles[i];

      // initialize for each particle
      p.associations.clear();
      p.sense_x.clear();
      p.sense_y.clear();

      // Find landmarks within sensor range for a particle
      vector<Map::single_landmark_s> predicted = {};    
      for (Map::single_landmark_s landmark: map_landmarks){
          double distance_p = dist(p.x, p.y ,landmark.x_f, landmark.y_f);
          if (distance_p <= sensor_range) {
              predicted.push_back(landmark);
          }// End if
      }// End for landmark

      double weight = 1.0; //initialize weight 
      // Loop through each observation for a particle
      for (LandmarkObs obs: observations) {      
          // Transform local/vehicle coords (obs.x, obs.y) of the observation to world/map coords    
          double x_map = p.x + (cos(p.theta) * obs.x) - (sin(p.theta) * obs.y);
          double y_map = p.y + (sin(p.theta) * obs.x) + (cos(p.theta) * obs.y);         

          // Find nearest landmark to the obs within the predicted landmarks
          double min_dist = std::numeric_limits<const double>::infinity();

          Map::single_landmark_s nearest_mark;
          for (Map::single_landmark_s landmark: predicted){
              double distance = dist(x_map, y_map, landmark.x_f, landmark.y_f);
              if (distance < min_dist) {
                  min_dist = distance;
                  nearest_mark.x_f  = landmark.x_f;
                  nearest_mark.y_f  = landmark.y_f;
                  nearest_mark.id_i = landmark.id_i;                 
              } // End if
          }// End for landmark
          
          double x_mu = nearest_mark.x_f;
          double y_mu = nearest_mark.y_f;

          // Calculate Mutlti-Variate Gaussian Probability Density as weight 
          double prob = multivGauss(x_map, y_map, x_mu, y_mu, std_landmark);
          double MIN_PROB = 0.000001;
          if (prob < MIN_PROB) { prob = MIN_PROB;}
          weight *= prob;   

          // For visulization purpose
          p.associations.push_back(nearest_mark.id_i);
          p.sense_x.push_back(x_map);
          p.sense_y.push_back(y_map);
      }//End for obs    
      weights.push_back(weight); // raw weight for each particle
  } // End for particles
  
  // Calculate sum of all weights
  double sum_weights = accumulate(weights.begin(), weights.end(), double(0.0));
  // Normalize weights 
  for (int i =0; i < num_particles ; ++i ){
      Particle &p = particles[i];
      p.weight = weights[i]/sum_weights;
      weights[i] = p.weight;
  }
  //cout << "End updateWeights" << endl;
}// End updateWeights

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   *  
   *  std::discrete_distribution produces random integers on the interval [0, n),
   *  where the probability of each individual integer i is defined as 
   *  wi/S, that is the weight of the ith weight divided by the sum of all n weights.
   */
  //cout << "Resample..." << endl;
  //std::random_device seed;
  //std::mt19937 random_generator(seed());

  std::default_random_engine random_generator(int(time(nullptr)));
  // sample particles based on their weight
  std::discrete_distribution<> sample(weights.begin(), weights.end());

  std::vector<Particle> new_particles(num_particles);
  for(auto &p : new_particles){
    p = particles[sample(random_generator)];
  }
  particles = std::move(new_particles);
 
  //cout << "End resample" << endl;
}// End resample

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

const double ParticleFilter::multivGauss(double x_map, double y_map, double x_mu, double y_mu, 
                      double std_landmark[]){ 

  double stdx = std_landmark[0];
  double stdy = std_landmark[1];

  double prob = 1/(2*M_PI*stdx*stdy) * 
                  exp(-0.5*( pow( (x_map -x_mu)/stdx , 2) + 
                             pow( (y_map -y_mu)/stdy , 2)));
  return prob;                       
}