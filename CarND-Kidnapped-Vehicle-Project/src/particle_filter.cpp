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
	std::default_random_engine generator;
    std::normal_distribution<double> x_dist(x, std[0]);
    std::normal_distribution<double> y_dist(y, std[1]);
    std::normal_distribution<double> theta_dist(theta, std[2]);
	for( int i = 0; i < 100; i++){
		Particle p;
		p.x = x_dist(generator);
		p.y = y_dist(generator);
		p.theta = theta_dist(generator);
		p.weight = 1.0;
		this->particles.push_back(p);
	}
	this->is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	std::default_random_engine gen;
    std::normal_distribution<double> x_dist(0, std_pos[0]);
    std::normal_distribution<double> y_dist(0, std_pos[1]);
    std::normal_distribution<double> theta_dist(0, std_pos[2]);

	for (int i=0; i < this->particles.size(); i++) {
	    float x = this->particles[i].x;
		float y = this->particles[i].y;
		float theta = this->particles[i].theta;

		float new_x =  x + velocity / yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta)) + x_dist(gen);
		float new_y = y + velocity / yaw_rate * (cos(theta) - cos(theta + yaw_rate * delta_t)) + y_dist(gen);
		float new_theta = theta + yaw_rate * delta_t +  theta_dist(gen);

		this->particles[i].x = new_x;
		this->particles[i].y = new_y;
		this->particles[i].theta = new_theta;
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.
	for(int i = 0; i < observations.size(); i++){

		double min_rmse = std::numeric_limits<double>::max();
		double distance = 0;

		for(int j = 0; j < predicted.size(); j++){

			distance = dist(
				observations[i].x, observations[i].y,
				predicted[j].x, predicted[j].y
			);

			if(distance < min_rmse){
				min_rmse = distance;
				observations[i].id = predicted[j].id;
			}
		}
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
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

	std::vector<LandmarkObs> landmarks;
	for(int i=0; i < map_landmarks.landmark_list.size(); i++){
		LandmarkObs obs;
		obs.x = map_landmarks.landmark_list[i].x_f;
		obs.y = map_landmarks.landmark_list[i].y_f;
		obs.id = map_landmarks.landmark_list[i].id_i;
		landmarks.push_back(obs);
	}

	for (int i=0; i < this->particles.size(); i++) {
		std::vector<LandmarkObs> transformedObservations;
		for(int j=0; j < observations.size(); j++){
			LandmarkObs obs;
			obs.x = this->particles[i].x + observations[j].x * cos(this->particles[i].theta) - observations[j].y * sin(this->particles[i].theta);
			obs.y = this->particles[i].y + observations[j].x * sin(this->particles[i].theta) + observations[j].y * cos(this->particles[i].theta);
			transformedObservations.push_back(obs);
		}
		dataAssociation(landmarks, transformedObservations);

		this->particles[i].weight = 1.0;
		for(int j=0; j < transformedObservations.size(); j++){

			for(int k=0; k < landmarks.size(); k++){
				if(landmarks[k].id == transformedObservations[j].id){
					double diff_x = transformedObservations[j].x - landmarks[k].x;
					double diff_y = transformedObservations[j].y - landmarks[k].y;
					double prob = 1/(2*M_PI) * exp(-0.5*(
						pow(diff_x/std_landmark[0],2) + pow(diff_y/std_landmark[1],2))
					) / std_landmark[1] / std_landmark[0];
					this->particles[i].weight *= prob;
				}
			}
		}
	}

	double weight_norm = 0;
	for (int i=0; i < this->particles.size(); i++) {
		weight_norm += this->particles[i].weight;
	}

	for (int i=0; i < this->particles.size(); i++) {
		this->particles[i].weight /= weight_norm;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	int index = 0;
	double beta = 0;
	std::vector<Particle> particles;

	double max_weight = 0;
	for (int i=0; i < this->particles.size(); i++) {
		if (this->particles[i].weight > max_weight){
			max_weight = this->particles[i].weight;
		}
	}

	while(particles.size() < this->particles.size()){
		beta += ((double) rand() / (RAND_MAX)) * max_weight * 2;
		while(this->particles[index].weight < beta){
			beta -= this->particles[index].weight;
			index += 1;
			if (index >= this->particles.size()){
				index = 0;
			}
		}
		Particle p;
		p.x = this->particles[index].x;
		p.y = this->particles[index].y;
		p.theta = this->particles[index].theta;
		p.weight = this->particles[index].weight;
		particles.push_back(p);
	}
	this->particles = particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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
