#include "rubi_global_heightmap_wrapper/global_grid.hpp"

#include <cmath>
#include <functional>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace {
int checks = 0;
void check(bool condition, const char* message) {
  ++checks;
  if (!condition) throw std::runtime_error(message);
}
void rejects(const std::function<void()>& call, const char* message) {
  bool rejected = false;
  try { call(); } catch (const std::invalid_argument&) { rejected = true; }
  check(rejected, message);
}
bool near(double a, double b) { return std::abs(a - b) < 1e-6; }
}  // namespace

int main() {
  try {
    rubi::Geometry geometry{"map", .05, -.2, -.1, 12, 8};
    // Keep the legacy invalidation contract covered alongside accumulation.
    rubi::GlobalGrid grid(geometry, .04F, rubi::InvalidCellPolicy::Invalidate);
    rubi::Patch patch{{"map", .05, -.1, 0, 3, 2}, 100,
                      {0, .05F, .15F, 1, 1.05F, 1.15F}, std::vector<float>(6, .001F)};
    // Physical +Y/+X patch is 2 x 3. The standard GridMap matrix is 3 x 2:
    // [[1.15, .15], [1.05, .05], [1, 0]] (rows=-X, columns=-Y).
    const std::vector<float> column_data{1.15F, 1.05F, 1, .15F, .05F, 0};
    const std::vector<float> row_data{1.15F, .15F, 1.05F, .05F, 1, 0};
    auto decoded = rubi::decode_grid_map_layer(column_data, 0,
      "column_index", 2, 6, "row_index", 3, 3, 2, 3);
    check(decoded == patch.elevation, "GridMap column-major world axes incorrect");
    auto offset_data = column_data;
    offset_data.insert(offset_data.begin(), -999);
    check(rubi::decode_grid_map_layer(offset_data, 1, "column_index", 2, 6,
      "row_index", 3, 3, 2, 3) == decoded, "MultiArray offset ignored");
    check(rubi::decode_grid_map_layer(row_data, 0, "row_index", 3, 6,
      "column_index", 2, 2, 2, 3) == decoded, "Row-major decoding failed");
    rejects([&]() { rubi::decode_grid_map_layer(column_data, 0, "column_index", 2, 6,
      "row_index", 3, 2, 2, 3); }, "Bad stride accepted");
    rejects([&]() { rubi::decode_grid_map_layer(column_data, 0, "", 2, 6,
      "", 3, 3, 2, 3); }, "Unspecified layout accepted");
    rejects([&]() { rubi::decode_grid_map_layer(column_data, 100, "column_index", 2, 6,
      "row_index", 3, 3, 2, 3); }, "Bad offset accepted");
    rejects([&]() { rubi::decode_grid_map_layer(column_data, 0, "column_index", 3, 6,
      "row_index", 2, 2, 2, 3); }, "Untransposed native dimensions accepted");
    auto masked_patch = patch;
    masked_patch.elevation = decoded;
    masked_patch.variance = rubi::decode_grid_map_layer(
      {.006F, .005F, .5F, .003F, .002F, .001F}, 0,
      "column_index", 2, 6, "row_index", 3, 3, 2, 3);
    rubi::GlobalGrid masked_grid(geometry);
    check(masked_grid.apply(masked_patch).invalid == 1, "Variance gate not applied");
    check(!masked_grid.cells()[3 * 12 + 2].usable,
          "High variance removed the wrong world cell");
    check(near(masked_grid.cells()[3 * 12 + 3].variance, .005),
          "Variance and height world axes differ");
    rubi::GlobalGrid retained(geometry);  // Accumulation is the default.
    retained.apply(patch);
    auto occluded = patch;
    occluded.stamp_ns = 200;
    occluded.elevation[0] = std::numeric_limits<float>::quiet_NaN();
    occluded.elevation[1] = std::numeric_limits<float>::infinity();
    occluded.variance[2] = .5F;
    occluded.variance[3] = -1;
    occluded.variance[4] = std::numeric_limits<float>::quiet_NaN();
    occluded.variance[5] = std::numeric_limits<float>::infinity();
    check(retained.apply(occluded).invalid == 6, "Invalid estimates accepted in retain mode");
    check(retained.surface().size() == 6, "Occlusion erased accepted global terrain");
    check(retained.last_snapshot_ns() == 200, "Invalid snapshot time not tracked");
    for (std::size_t i = 0; i < 6; ++i) {
      const auto& c = retained.cells()[(i / 3 + 2) * 12 + i % 3 + 2];
      check(c.usable && near(c.elevation, patch.elevation[i]) && near(c.variance, .001) &&
            c.estimate_snapshot_ns == 100, "Retention changed or freshened historical estimate");
    }
    occluded.geometry.origin_x = .15;
    occluded.stamp_ns = 300;
    retained.apply(occluded);
    check(retained.surface().size() == 6, "Invalid unseen region was filled");
    auto reobserved = patch;
    reobserved.stamp_ns = 400;
    reobserved.elevation[0] = -.15F;
    reobserved.variance[0] = .002F;
    retained.apply(reobserved);
    check(near(retained.surface()[0].z, -.15), "Retention froze a valid lower observation");
    check(near(retained.cells()[26].variance, .002) &&
          retained.cells()[26].estimate_snapshot_ns == 400, "Reobservation not replaced exactly");
    auto new_region = reobserved;
    new_region.geometry.origin_x = .15;
    new_region.stamp_ns = 500;
    retained.apply(new_region);
    occluded.geometry.origin_x = patch.geometry.origin_x;
    occluded.stamp_ns = 600;
    retained.apply(occluded);
    check(retained.surface().size() == 12, "Occluded re-entry erased global accumulation");
    std::stringstream retained_file;
    retained.save(retained_file);
    rubi::GlobalGrid retained_copy(geometry);
    retained_copy.load(retained_file);
    std::stringstream retained_again;
    retained_copy.save(retained_again);
    check(retained_file.str() == retained_again.str(), "Retained history round-trip changed state");
    patch.elevation = decoded;
    check(grid.apply(patch).accepted == 6, "Initial patch not accepted");
    auto points = grid.surface();
    check(points.size() == 6, "Wrong coverage");
    check(near(points[0].x, -.075) && near(points[0].y, .025), "Wrong world cell center");
    check(near(points[2].z, .15) && near(points[3].z, 1), "Asymmetric world axes changed");
    check(near(points[1].z - points[0].z, .05), "5 cm detail changed");
    check(near(points[2].z - points[0].z, .15), "15 cm detail changed");
    check(grid.apply(patch).duplicate, "Repeated posterior not ignored");
    check(near(grid.cells()[2 * 12 + 2].variance, .001), "Duplicate reduced uncertainty");

    auto bad = patch;
    bad.geometry.origin_x += .025;
    rejects([&]() { grid.apply(bad); }, "Half-cell phase mismatch accepted");
    bad = patch; bad.geometry.frame = "odom";
    rejects([&]() { grid.apply(bad); }, "Wrong frame accepted");
    bad = patch; bad.geometry.resolution = .1;
    rejects([&]() { grid.apply(bad); }, "Wrong resolution accepted");
    bad = patch; bad.stamp_ns = 99;
    rejects([&]() { grid.apply(bad); }, "Backwards time accepted");
    bad = patch; bad.elevation.pop_back();
    rejects([&]() { grid.apply(bad); }, "Malformed patch accepted");
    check(grid.last_snapshot_ns() == 100 && grid.surface().size() == 6,
          "Rejected patch modified state");

    auto moved = patch;
    moved.geometry.origin_x = .15;
    moved.stamp_ns = 200;
    grid.apply(moved);
    check(grid.surface().size() == 12, "Leaving rolling window erased old terrain");
    patch.stamp_ns = 300;
    patch.elevation[0] = -.15F;
    patch.variance[0] = .002F;
    grid.apply(patch);
    check(near(grid.surface()[0].z, -.15), "Revisit averaged or rejected lower height");
    check(near(grid.cells()[2 * 12 + 2].variance, .002), "Revisit fused posterior twice");
    patch.stamp_ns = 400;
    patch.elevation[0] = std::numeric_limits<float>::quiet_NaN();
    patch.variance[1] = .5F;
    patch.variance[2] = -1;
    check(grid.apply(patch).invalid == 3, "NaN/high/negative variance not invalidated");
    check(grid.surface().size() == 9, "Invalid local cells exported as usable");
    check(near(grid.cells()[2 * 12 + 2].elevation, -.15), "Invalidation erased history");

    std::stringstream saved;
    grid.save(saved);
    rubi::GlobalGrid restored(geometry);
    restored.load(saved);
    // Loading an old map must not silently reactivate hidden historical cells,
    // even though the new runtime default is retain.
    check(restored.surface().size() == 9 && restored.last_snapshot_ns() == 400,
          "Load changed coverage/time");
    check(!restored.cells()[2 * 12 + 2].usable &&
          near(restored.cells()[2 * 12 + 2].elevation, -.15), "Load lost invalid history");
    std::stringstream again;
    restored.save(again);
    check(saved.str() == again.str(), "Save/load not exact");
    auto truncated_text = saved.str().substr(0, saved.str().size() / 2);
    std::stringstream truncated(truncated_text);
    rejects([&]() { restored.load(truncated); }, "Truncated save accepted");
    check(restored.surface().size() == 9, "Failed load destroyed current map");
    std::stringstream extra(saved.str() + "unexpected\n");
    rejects([&]() { restored.load(extra); }, "Trailing data accepted");
    std::stringstream pcd;
    restored.save_pcd(pcd);
    check(pcd.str().find("POINTS 9\n") != std::string::npos, "PCD count wrong");

    moved.stamp_ns = 500;
    moved.geometry.origin_x = .35;
    check(grid.apply(moved).outside == 4, "Global bound clipping wrong");
    grid.clear();
    check(grid.surface().empty() && grid.last_snapshot_ns() == 0, "Clear failed");
    patch.stamp_ns = 1;
    check(grid.apply(patch).accepted == 3, "Replay after reset rejected");
    rejects([]() { rubi::GlobalGrid huge({"map", .05, 0, 0, 5000001, 2}); },
            "Unbounded allocation permitted");
    std::cout << "PASS " << checks << " checks\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
