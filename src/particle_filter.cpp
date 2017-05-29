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

#include "particle_filter.h"

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    std::random_device rd;
    std::default_random_engine gen(rd());

    std::normal_distribution<double> dist_x(x, std[0]);
    std::normal_distribution<double> dist_y(y, std[1]);
    std::normal_distribution<double> dist_theta(theta, std[2]);

    num_particles = 100;
    particles.resize(num_particles);

    for (int i = 0; i < num_particles; ++i) {
        Particle particle = { i, dist_x(gen), dist_y(gen), dist_theta(gen), 1 };
        particles[i] = particle;
    }

    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    std::random_device rd;
    std::default_random_engine gen(rd());

    for (int i = 0; i < num_particles; ++i) {
        Particle particle = particles[i];
        double x = particle.x;
        double y = particle.y;
        double theta = particle.theta;

        // cache intermediate calculations
        double tptddt = theta + yaw_rate * delta_t;
        double vdtddt = velocity / yaw_rate;

        double xf = x + vdtddt * (sin(tptddt) - sin(theta));
        double yf = y + vdtddt * (cos(theta) - cos(tptddt));
        double tf = tptddt;

        std::normal_distribution<double> dist_x(xf, std_pos[0]);
        std::normal_distribution<double> dist_y(yf, std_pos[1]);
        std::normal_distribution<double> dist_t(tf, std_pos[2]);

        particle.x = dist_x(gen);
        particle.y = dist_y(gen);
        particle.theta = dist_t(gen);

        particles[i] = particle;
    }
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
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    // simultaneously normalize weights and find the max
    double total_weight = 0;
    for (int i = 0; i < num_particles; ++i) {
        total_weight += particles[i].weight;
    }
    double w_max = 0;
    for (int i = 0; i < num_particles; ++i) {
        particles[i].weight /= total_weight;
        if (particles[i].weight > w_max) {
            w_max = particles[i].weight;
        }
    }

    std::random_device rd;
    std::default_random_engine gen(rd());

    std::uniform_int_distribution<int> int_dist(0, num_particles - 1);
    int index = int_dist(gen);

    std::uniform_real_distribution<double> weight_dist(0, 2 * w_max);
    double beta = 0;

    // resample particles using wheel method
    std::vector<Particle> resampled_particles;
    for (int i = 0; i < num_particles; ++i) {
        beta += weight_dist(gen);
        while (particles[index].weight < beta) {
            beta -= particles[index].weight;
            index = (index + 1) % num_particles;
        }
        resampled_particles.push_back(particles[index]);
    }

    particles = resampled_particles;
}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
