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
	num_particles = 1000;
	particles = std::vector<Particle>(num_particles);

	std::default_random_engine rand_engine;
	std::normal_distribution<double> x_distribution(x,std[0]);
	std::normal_distribution<double> y_distribution(y,std[1]);
	std::normal_distribution<double> theta_distribution(theta,std[2]); 

	for(int i=0; i<num_particles; i++){
		particles[i].id = i;
		particles[i].x = x_distribution(rand_engine);
		particles[i].y = y_distribution(rand_engine);
		particles[i].theta = theta_distribution(rand_engine);
		particles[i].weight = 1;

	}

	weights = std::vector<double>(num_particles);

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	std::default_random_engine rand_engine;
	for(int i=0; i<num_particles; i++){
		double mean_delta_x, mean_delta_y, mean_delta_theta;
		if(yaw_rate != 0){
			mean_delta_x = (velocity/yaw_rate) * (sin(particles[i].theta+yaw_rate*delta_t)-sin(particles[i].theta));
			mean_delta_y = (velocity/yaw_rate) * (cos(particles[i].theta)-cos(particles[i].theta+yaw_rate*delta_t));
			mean_delta_theta = yaw_rate * delta_t;
		}
		else{
			mean_delta_x = velocity * cos(yaw_rate)*delta_t;
			mean_delta_y = velocity * sin(yaw_rate)*delta_t;
			mean_delta_theta = particles[i].theta;
		}

		std::normal_distribution<double> x_distribution(mean_delta_x,std_pos[0]);
		std::normal_distribution<double> y_distribution(mean_delta_y,std_pos[1]);
		std::normal_distribution<double> theta_distribution(mean_delta_theta,std_pos[2]); 
	
		particles[i].x += x_distribution(rand_engine);
		particles[i].y += y_distribution(rand_engine);
		particles[i].theta += theta_distribution(rand_engine);
		 
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	if(predicted.size() == 0){
		return;
	}

	for(int i=0; i<observations.size(); i++){
		int nearest_landmark_id = predicted[0].id;
		double nearest_landmark_dist = dist(observations[i].x,observations[i].y, predicted[0].x, predicted[0].y);

		for(int j=1; j<predicted.size(); j++){
			double new_dist = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
			if(new_dist < nearest_landmark_dist){
				nearest_landmark_dist = new_dist;
				nearest_landmark_id = predicted[j].id;
			}
		}

		observations[i].id = nearest_landmark_id;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map) {
	// For every particle, find evey landmark in the map within the sensor's range
	// and add the landmark's coordinates, in map's FoR, to the list of predicted
	// landmarks. 
	std::vector<Map::single_landmark_s> all_landmarks = map.landmark_list; 
	for(int i=0; i<num_particles; i++){
		Particle *this_particle = &particles[i];

		std::vector<LandmarkObs> predicted_landmarks;
		
		for(int j=0; j<all_landmarks.size(); j++){
			Map::single_landmark_s that_landmark = all_landmarks[j];
			double distance_to_landmark = dist(this_particle->x, this_particle->y, 
				that_landmark.x_f, that_landmark.y_f);
			
			// If lanndmark is within sensor range of this particle, add the landmark'
			// to the list of predictions.
			if(distance_to_landmark < sensor_range){
				LandmarkObs predicted_landmark;
				predicted_landmark.id = that_landmark.id_i;				
				predicted_landmark.x  = that_landmark.x_f;;
				predicted_landmark.y  = that_landmark.y_f;
				predicted_landmarks.push_back(predicted_landmark);
			}
		}

		// Now that we have a list of predicted landmarks for this particle,
		// loop over every observation, convert its landmark cooridnates to map's FoR,
		// and associate it with  its nearest predicted landmark.

		// Build a list of observations having landmark coordinates in the map's FoR.
		std::vector<LandmarkObs> observed_landmarks;
		for(int k=0; k<observations.size(); k++){
			LandmarkObs this_observation = observations[k];
			double x_obs = this_observation.x;
			double y_obs = this_observation.y;
			
			double x_particle = this_particle->x;
			double y_particle = this_particle->y;
			double theta = this_particle->theta;

			// Update observation coordinates from particle to map FoR.
			LandmarkObs observed_landmark;
			observed_landmark.x = x_obs * cos(theta) - y_obs * sin(theta) + x_particle;
			observed_landmark.y = x_obs * sin(theta) + y_obs * cos(theta) + y_particle;
			
			observed_landmarks.push_back(observed_landmark);	
		}

		// Associate our observations with nearest predictions.
		dataAssociation(predicted_landmarks,observed_landmarks);

		// Copy observation data into this particle's sense_x, sense_y and
		// asssociation variables. This is used by the simulator and accessed
		// through ParticleFilter::etAssociations();
		std::vector<int> landmark_id;
		std::vector<double> landmark_x;
		std::vector<double> landmark_y;
		for(int l=0; l<observed_landmarks.size(); l++){
			landmark_id.push_back(observed_landmarks[l].id);
			landmark_x.push_back(observed_landmarks[l].x);
			landmark_y.push_back(observed_landmarks[l].y);
		}

		particles[i] = SetAssociations(*this_particle,landmark_id,landmark_x,landmark_y);

		// Update weight of this particle.
		// Traverse the list of observations for this particle and for
		// each observation find the probability of the observed coordinates
		// of its landmark using Gaussian distribution with mean equal to the 
		// actual position of the landmark and standard deviation received as
		// a parameter to this function.
		this_particle->weight = 1;
		double std_x = std_landmark[0];
		double std_y = std_landmark[1];
		
		for(int m=0; m<observed_landmarks.size(); m++){
			int landmark_id = observed_landmarks[m].id;
			double observed_x = observed_landmarks[m].x;
			double observed_y = observed_landmarks[m].y;

			double mean_x = 0.;
			double mean_y = 0.;
			bool match_found = false;
			for(int n=0; n<predicted_landmarks.size(); n++){
				if(predicted_landmarks[n].id == landmark_id){
					mean_x = predicted_landmarks[n].x;
					mean_y = predicted_landmarks[n].y;
					match_found = true;
					break;
				}
			}
			if(match_found == false){
				cout << "Warning: No matching landmark found with id: " << landmark_id << endl;
			}

			double normalizing_factor = 1.0 / (2*M_PI*std_x*std_y); 
			double exponent = (observed_x - mean_x)*(observed_x - mean_x) / (2*std_x*std_x) + 
							  (observed_y - mean_y)*(observed_y - mean_y) / (2*std_y*std_y);
							  
			double observation_prob = normalizing_factor * exp(-exponent);
			this_particle->weight *= observation_prob;
		}

	}

}

void ParticleFilter::resample() {

	//Find normalized weights.
	double sum_weights = 0.0;
	for(int i=0; i<particles.size(); i++){
		weights[i] = particles[i].weight;
		sum_weights += weights[i];
	}

	for(int i=0; i<weights.size(); i++){
		weights[i] /= sum_weights;
	}

	// Create new particles by sampling old particles with replacement.
	std::default_random_engine rand_eng;
	std::discrete_distribution<int> bag_of_particle_indices(weights.begin(),weights.end());
	std::vector<Particle> new_particles;
	for(int n=0; n<num_particles; n++){
		int resampled_particle_id = bag_of_particle_indices(rand_eng);
		new_particles.push_back(particles[resampled_particle_id]);
	}

	particles = new_particles;

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
