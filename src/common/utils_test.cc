#include <gtest/gtest.h>

#include "parallel_for.h"
#include "utils.h"

TEST(Utils, ParallelForVisitsEachIndexOnce) {
  std::vector<int> visits(100, 0);

  ParallelFor(visits.size(), 4, [&](std::size_t index) {
    ++visits[index];
  });

  for (const int visit_count : visits) {
    EXPECT_EQ(visit_count, 1);
  }
}

TEST(Utils, Jl_Jl_inv) {
  Vector3d v{1, 2, 3};

  auto jl_inv_mat = Jl_inv(v);
  auto jl         = Jl(v);

  EXPECT_TRUE(jl_inv_mat.isApprox(jl.inverse()));
}

TEST(Utils, Jl_Jr) {
  Vector3d v{1, 2, 3};

  auto jl = Jl(v);
  auto jr = Jr(-v);

  EXPECT_TRUE(jl.isApprox(jr));
}
