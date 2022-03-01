#include <ros/ros.h>
#include <GaussianMixtureModel/ExpectationMaximization.h>
#include <GaussianMixtureModel/GaussianMixtureModelFactory.h>
#include <GaussianUtils/GaussianDistributionFactory.h>
#include <vector>
#include <cmath>
#include <set>

constexpr std::size_t SAMPLES = 500;

using namespace std;

void emplace_back(std::vector<gauss::gmm::Cluster>& clusters, const double w, const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance) {
    clusters.emplace_back();
    clusters.back().weight = w;
    clusters.back().distribution = std::make_unique<gauss::GaussianDistribution>(mean, covariance);
}


Eigen::VectorXd make_vector(const std::vector<double> &values) {
  if (values.empty()) {
    throw std::runtime_error("empty buffer");
  }
  Eigen::VectorXd vector(values.size());
  Eigen::Index index = 0;
  for (const auto &value : values) {
    vector(index) = value;
    ++index;
  }
  return vector;
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "sampling_test");
    //ros::NodeHandle n;
    //ros::Rate loop_rate(1500);
    //
    std::vector<gauss::gmm::Cluster> clusters;
    const std::size_t nClusters = 4;
  {
    Eigen::VectorXd eigs_cov(2);
    eigs_cov << 0.1, 0.2;
    clusters.reserve(nClusters);
    double angle = 0.0;
    double angle_delta = 3.14 / static_cast<double>(nClusters);
    for (std::size_t c = 0; c < nClusters; ++c) {
        emplace_back(clusters,
          1.0 / static_cast<double>(nClusters),
              make_vector({2.0 * cos(angle), 2.0 * sin(angle)}),
              gauss::make_random_covariance(eigs_cov));
      angle += angle_delta;
    }
  }
  gauss::gmm::GaussianMixtureModel ref_model(clusters);

  auto samples = ref_model.drawSamples(SAMPLES);
  for(size_t i(0); i< samples.size();i++)
      std::cout<<i<<" : "<<samples[i][0]<<","<<samples[i][1]<<std::endl;



    
    return 0;

}

