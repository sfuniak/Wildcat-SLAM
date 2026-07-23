#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <thread>
#include <utility>
#include <vector>

template <typename Function>
void ParallelFor(std::size_t item_count, std::size_t num_threads, Function &&function) {
  std::atomic<std::size_t> next_index{0};
  const auto worker = [&]() {
    for (std::size_t index = next_index.fetch_add(1, std::memory_order_relaxed);
         index < item_count;
         index = next_index.fetch_add(1, std::memory_order_relaxed)) {
      function(index);
    }
  };

  const auto worker_count = std::min(item_count, std::max<std::size_t>(num_threads, 1));
  std::vector<std::thread> workers;
  workers.reserve(worker_count);
  for (std::size_t worker_index = 0; worker_index < worker_count; ++worker_index) {
    workers.emplace_back(worker);
  }
  for (auto &thread : workers) {
    thread.join();
  }
}
