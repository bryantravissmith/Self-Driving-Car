#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <algorithm>
#include "classifier.h"

/**
 * Initializes GNB
 */
GNB::GNB() {

}

GNB::~GNB() {}

double normal(double x, double mu, double std){
    return 1.0 / sqrt(2*M_PI*pow(std, 2)) * exp(-0.5 * pow( ( x - mu ) / std , 2));
}

void GNB::train(vector<vector<double>> data, vector<string> labels)
{
    int variable_cnt = data[0].size();
    this->data_cnt = data.size();

    for(int i = 0; i < this->possible_labels.size(); i++){
        for(int j = 0; j < variable_cnt; j++){
             this->sums[this->possible_labels[i]][j] = 0.0;
             this->sq_sums[this->possible_labels[i]][j] = 0.0;
        }
    }

    for(int i = 0; i < this->data_cnt; i++){
        for(int j=0; j < variable_cnt; j++){
            this->sums[labels[i]][j] += data[i][j];
            this->sq_sums[labels[i]][j] += pow(data[i][j], 2);
        }
    }
}

string GNB::predict(vector<double> sample)
{
	vector<double> v = {1.0, 1.0, 1.0};

	for(int i = 0; i < this->possible_labels.size(); i++){
        for(int j = 0; j < sample.size(); j++){
            double avg = this->sums[this->possible_labels[i]][j] / this->data_cnt;
            double std = sqrt(this->sq_sums[this->possible_labels[i]][j] / this->data_cnt - pow(avg, 2));
            v[i] *= normal(sample[j], avg, std);
        }
    }

    int argmax = distance(v.begin(), max_element(v.begin(), v.end()));
	return this->possible_labels[argmax];

}
