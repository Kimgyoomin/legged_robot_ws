#include <cstdint>
#include <cstdio>
int main() {
  int64_t currStartIndex = 2147480000LL;
  int64_t samplesStep = 40000, downSample = 1, maxPointSize = 800000;
  int64_t end_index = currStartIndex + samplesStep;
  for (int k = currStartIndex; k < end_index; k += downSample) {
    auto index = k % maxPointSize;
    if (index < 0 || index >= maxPointSize) {
      std::fprintf(stderr, "Out-of-bounds pattern access: k=%d, index=%lld, size=%lld\n", k, (long long)index, (long long)maxPointSize);
      return 1;
    }
  }
  return 0;
}
