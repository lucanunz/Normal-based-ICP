#pragma once

template <typename IteratorType_>
int bruteForceSearch(std::vector<typename IteratorType_::value_type*>& answers,
                     IteratorType_ begin,
                     IteratorType_ end,
                     const typename IteratorType_::value_type& query,
                     const typename IteratorType_::value_type::Scalar norm) {
  using Scalar = typename IteratorType_::value_type::Scalar;
  const Scalar  squared_norm = norm*norm;
  int matches=0;
  for (auto it=begin; it!=end; ++it) {
    auto& p=*it;
    if ((p-query).squaredNorm()<squared_norm) {
      answers.push_back(&p);
      ++matches;
    }
  }
  return matches;
}

template <typename IteratorType_>
typename IteratorType_::value_type*
bruteForceBestMatch(IteratorType_ begin,
                    IteratorType_ end,
                    const typename IteratorType_::value_type& query,
                    const typename IteratorType_::value_type::Scalar norm) {
  using PointType = typename IteratorType_::value_type;
  using Scalar = typename PointType::Scalar;
  const Scalar  squared_norm = norm*norm;
  PointType* best=0;
  Scalar best_squared_norm=norm*norm;
  for (auto it=begin; it!=end; ++it) {
    auto& p=*it;
    Scalar squared_distance  = (p-query).squaredNorm();
    if ( squared_distance < best_squared_norm) {
      best=&p;
      best_squared_norm = squared_distance;
    }
  }
  return best;
}
