/*
 * particle_filter.cpp
 *
 *  Created on: Jul 06, 2017
 *      Author: Oleg Potkin
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
#include <map>

#include "particle_filter.h"

using namespace std;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
    // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
    // x, y, theta and their uncertainties from GPS) and all weights to 1.
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    // Number of particles to draw
    num_particles = 100;

    // Normal distributions
    std::default_random_engine generator;
    std::normal_distribution<double> dist_x(x, std[0]);
    std::normal_distribution<double> dist_y(y, std[1]);
    std::normal_distribution<double> dist_theta(theta, std[2]);

    // Pre-allocate vectors
    particles.resize(num_particles);        // Set of current particles
    weights.resize(num_particles);          // Vector of weights of all particles

    for (unsigned i = 0; i < num_particles; ++i) {
        particles[i].id     = i;
        particles[i].x      = dist_x(generator);
        particles[i].y      = dist_y(generator);
        particles[i].theta  = dist_theta(generator);
        particles[i].weight = 1.0;
        weights[i]          = 1.0;
    }

    // Flag, if filter is initialized
    is_initialized = true;
}


void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/

    double x_0, y_0, theta_0;
    double x_f, y_f, theta_f;
    double small_num = 0.000001;

    std::default_random_engine generator;

    for (unsigned i = 0; i < num_particles; ++i) {
        x_0 = particles[i].x;
        y_0 = particles[i].y;
        theta_0 = particles[i].theta;

        if (fabs(yaw_rate) > small_num) {
            x_f     = x_0 + velocity / yaw_rate * (sin(theta_0 + yaw_rate * delta_t) - sin(theta_0));
            y_f     = y_0 + velocity / yaw_rate * (cos(theta_0) - cos(theta_0 + yaw_rate * delta_t));
            theta_f = theta_0 + yaw_rate * delta_t;
        }
        else {
            x_f     = x_0 + velocity * delta_t * cos(theta_0);
            y_f     = y_0 + velocity * delta_t * sin(theta_0);
            theta_f = theta_0;
        }

        // Update the particle position with the prediction
        // and add Gaussian noise
        std::normal_distribution<double> dist_x(x_f, std_pos[0]);
        std::normal_distribution<double> dist_y(y_f, std_pos[1]);
        std::normal_distribution<double> dist_theta(theta_f, std_pos[2]);

        particles[i].x      = dist_x(generator);
        particles[i].y      = dist_y(generator);
        particles[i].theta  = dist_theta(generator);
    }
}


void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
    //   implement this method and use it as a helper during the updateWeights phase.

    double meas;
    double min_meas;

    for (unsigned i = 0; i < observations.size(); ++i) {
        // Use large number in new iteration
        min_meas = 1e6;
        for (unsigned j = 0; j < predicted.size(); ++j) {
            meas = dist(observations[i].x,
                            observations[i].y,
                            predicted[j].x,
                            predicted[j].y);

            if (meas < min_meas) {
                observations[i].id = predicted[j].id;
                min_meas = meas;
            }
        }
    }
}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   std::vector<LandmarkObs> observations,
                                   Map map_landmarks) {
    // TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
    //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
    // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
    //   according to the MAP'S coordinate system. You will need to transform between the two systems.
    //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
    //   The following is a good resource for the theory:
    //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
    //   and the following is a good resource for the actual equation to implement (look at equation 3.33)
    //   http://planning.cs.uiuc.edu/node99.htm

    // To allow access to landmarks by id
    // We need to include <map>
    std::map<int, Map::single_landmark_s> landmarks_id_map;

    for (unsigned i = 0; i < map_landmarks.landmark_list.size(); ++i) {
        landmarks_id_map.insert(std::make_pair(map_landmarks.landmark_list[i].id_i,
                                               map_landmarks.landmark_list[i]));
    }

    // Main particle loop
    for (unsigned i = 0; i < num_particles; ++i) {
        // Extract particle position
        double x_p = particles[i].x;
        double y_p = particles[i].y;
        double theta_p = particles[i].theta;
        double cos_t = cos(theta_p);
        double sin_t = sin(theta_p);

        // Convert observations from vehicle to map coordinate system
        double x_m;
        double y_m;
        std::vector<LandmarkObs> observations_transformed;
        observations_transformed.resize(observations.size());

        for (unsigned j = 0; j < observations.size(); ++j) {
            x_m = x_p + observations[j].x * cos_t - observations[j].y * sin_t;
            y_m = y_p + observations[j].x * sin_t + observations[j].y * cos_t;
            observations_transformed[j] = LandmarkObs{-1, x_m, y_m};
        }

        // Landmarks in range
        std::vector<LandmarkObs> lm_in_range;

        for (unsigned j = 0; j < map_landmarks.landmark_list.size(); ++j) {
            double distance  = dist(x_p,
                                    y_p,
                                    map_landmarks.landmark_list[j].x_f,
                                    map_landmarks.landmark_list[j].y_f);

            if (distance < sensor_range) {
                lm_in_range.push_back(LandmarkObs{map_landmarks.landmark_list[j].id_i,
                                                  map_landmarks.landmark_list[j].x_f,
                                                  map_landmarks.landmark_list[j].y_f });
            }
        }

        // Calculate weights
        if (lm_in_range.size() > 0) {
            // Associate each measurement with the respective nearest
            // neighbor landmark in range
            dataAssociation(lm_in_range, observations_transformed);

            // Reset weight of the particle
            particles[i].weight = 1.0;

            for (const auto obs:observations_transformed) {
                double x = landmarks_id_map[obs.id].x_f;
                double y = landmarks_id_map[obs.id].y_f;

                double x_mu = obs.x;
                double y_mu = obs.y;

                double dx = x - x_mu;
                double dy = y - y_mu;

                double std_x = std_landmark[0];
                double std_y = std_landmark[1];

                double num = exp(-0.5 * (dx * dx / (std_x * std_x) + dy * dy / (std_y * std_y)));
                double den = 2 * M_PI * std_x * std_y;
                double prob = num/den;

                particles[i].weight *= prob;
            }

            weights[i] = particles[i].weight; // Assign to weight of ith particle
        }
        else {

            weights[i] = 0.0;
        }
    }
}


void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    std::default_random_engine generator;
    std::discrete_distribution<> dist(weights.begin(), weights.end());
    std::vector<Particle> particles_new;

    particles_new.resize(num_particles);

    for (unsigned i = 0; i < num_particles; ++i) {
        particles_new[i] = particles[dist(generator)];
    }

    particles = particles_new;
}


Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	// Clear the previous associations
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
