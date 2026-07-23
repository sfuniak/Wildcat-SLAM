#include "knn_surfel_matcher.h"

#include <absl/container/flat_hash_set.h>

#include <algorithm>

#include "common/parallel_for.h"

KnnSurfelMatcher::KnnSurfelMatcher(int num_threads) : num_threads_(num_threads) {
  CHECK_GT(num_threads_, 0);
}

void KnnSurfelMatcher::BuildIndex(const std::deque<Surfel::Ptr> &surfels) {
  if (surfels.empty()) {
    return;
  }
  target_surfels_ = surfels;
  std::vector<FloatType> cloud(target_surfels_.size() * dim_);
  ParallelFor(target_surfels_.size(), num_threads_, [&](std::size_t index) {
    const auto center = target_surfels_[index]->GetCenterInWorld() / kCenterDistThreshold;
    const auto norm   = target_surfels_[index]->GetNormInWorld() / kAngularDistThreshold;
    const auto offset = index * dim_;
    Eigen::Map<Vector3d>(cloud.data() + offset)     = center;
    Eigen::Map<Vector3d>(cloud.data() + offset + 3) = norm;
  });
  this->FLANNBuildIndex(cloud);
}

void KnnSurfelMatcher::Match(std::deque<Surfel::Ptr> &surfels, std::vector<SurfelCorrespondence> &surfels_corrs) {
  surfels_corrs.clear();
  if (target_surfels_.empty()) {
    return;
  }

  std::vector<std::vector<int>> candidate_indices(surfels.size());
  ParallelFor(surfels.size(), num_threads_, [&](std::size_t index) {
    const auto &surfel = surfels[index];
    std::vector<int> k_indices;
    this->KNearestSearchIndices(surfel, kNearestSurfelCandidatesNum, k_indices);
    for (const int nearest_index : k_indices) {
      const auto &nearest_surfel = target_surfels_[nearest_index];
      if (std::abs(nearest_surfel->timestamp - surfel->timestamp) < kTimeDiffThreshold) {
        continue;
      }
      if (surfel->AngularDistance(*nearest_surfel) > kAngularDistThreshold) {
        continue;
      }
      if (std::abs(surfel->GetNormInWorld().dot(surfel->GetCenterInWorld() - nearest_surfel->GetCenterInWorld())) > kSurfelDistThreshold) {
        continue;
      }
      candidate_indices[index].push_back(nearest_index);
    }
  });

  const bool same_surfel_set =
      surfels.size() == target_surfels_.size() &&
      std::equal(surfels.begin(), surfels.end(), target_surfels_.begin());
  absl::flat_hash_set<std::pair<int, int>> surfel_pairs;
  surfel_pairs.reserve(surfels.size());
  for (std::size_t index = 0; index < surfels.size(); ++index) {
    const auto &surfel = surfels[index];
    for (const int nearest_index : candidate_indices[index]) {
      const auto &nearest_surfel = target_surfels_[nearest_index];
      const std::pair<int, int> pair = std::minmax(static_cast<int>(index), nearest_index);
      if (same_surfel_set && !surfel_pairs.insert(pair).second) {
        continue;
      }

      if (surfel->timestamp < nearest_surfel->timestamp) {
        surfels_corrs.push_back({surfel, nearest_surfel});
      } else {
        surfels_corrs.push_back({nearest_surfel, surfel});
      }
      break;
    }
  }
}

void KnnSurfelMatcher::KNearestSearch(const Surfel::Ptr &surfel, int k, std::vector<Surfel::Ptr> &k_nearest_surfels) {
  std::vector<int> k_indices;
  this->KNearestSearchIndices(surfel, k, k_indices);
  for (const int index : k_indices) {
    k_nearest_surfels.push_back(target_surfels_[index]);
  }
}

void KnnSurfelMatcher::KNearestSearchIndices(const Surfel::Ptr &surfel, int k, std::vector<int> &k_indices) {
  std::vector<FloatType> query = ToVector(surfel);
  CHECK_EQ(query.size(), dim_);

  std::vector<FloatType> k_distances;

  this->FLANNKNearestSearch(query, k, k_indices, k_distances);
}

void KnnSurfelMatcher::FLANNBuildIndex(const std::vector<FloatType> &cloud) {
  CHECK_EQ(cloud.size() % dim_, 0);
  cloud_ = cloud;  // save cloud_ for flann search
  index_.reset(
      new FLANNIndex(
          flann::Matrix<FloatType>(cloud_.data(), cloud_.size() / dim_, dim_),
          flann::KDTreeSingleIndexParams(15)));
  index_->buildIndex();
}

void KnnSurfelMatcher::FLANNKNearestSearch(std::vector<FloatType> &query, int k, std::vector<int> &k_indices, std::vector<FloatType> &k_distances) {
  CHECK(query.size() == dim_);

  k_indices.resize(k);
  k_distances.resize(k);

  flann::Matrix<int>       k_indices_mat(&k_indices[0], 1, k);
  flann::Matrix<FloatType> k_distances_mat(&k_distances[0], 1, k);

  // Wrap the k_indices and k_distances vectors (no data copy)
  index_->knnSearch(
      flann::Matrix<FloatType>(&query[0], 1, dim_),
      k_indices_mat, k_distances_mat, k,
      flann::SearchParams(-1, 0.0));
}

std::vector<KnnSurfelMatcher::FloatType> KnnSurfelMatcher::ToVector(const Surfel::Ptr &surfel) {
  // todo use resolution
  auto     center         = surfel->GetCenterInWorld();
  auto     norm           = surfel->GetNormInWorld();
  Vector3d center_uniform = center / kCenterDistThreshold;
  Vector3d norm_uniform   = norm / kAngularDistThreshold;
  return {center_uniform.x(), center_uniform.y(), center_uniform.z(), norm_uniform.x(), norm_uniform.y(), norm_uniform.z()};
}
