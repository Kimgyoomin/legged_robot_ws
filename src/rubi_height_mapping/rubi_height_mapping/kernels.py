# Copyright (c) 2022, Takahiro Miki. All rights reserved.
# Licensed under MIT; see LICENSE and UPSTREAM.md.
# RUBI height-only derivative, 2026-10-14.
import cupy as cp
import string
from .ray_selection import voxel_selector_preamble

def map_utils(
    resolution,
    width,
    height,
    sensor_noise_factor,
    min_valid_distance,
    max_height_range,
    ramped_height_range_a,
    ramped_height_range_b,
    ramped_height_range_c,
):
    util_preamble = string.Template(
        """
        __device__ float16 clamp(float16 x, float16 min_x, float16 max_x) {

            return max(min(x, max_x), min_x);
        }
        __device__ int get_x_idx(float16 x, float16 center) {
            float fi = (x - center) / ${resolution} + 0.5 * (${width} - 1);
            int i = (int)floorf(fi + 0.5f);
            return i;
        }
        __device__ int get_y_idx(float16 y, float16 center) {
            float fi = (y - center) / ${resolution} + 0.5 * (${height} - 1);
            int i = (int)floorf(fi + 0.5f);
            return i;
        }
        __device__ bool is_inside(int idx) {
            // Fixed: Row-Major (Row=Y, Col=X)
            // Row index (Y)
            int idx_y = idx / ${width};
            // Column index (X)
            int idx_x = idx % ${width};
            // Check Col bounds (Width)
            if (idx_x == 0 || idx_x == ${width} - 1) {
                return false;
            }
            // Check Row bounds (Height)
            if (idx_y == 0 || idx_y == ${height} - 1) {
                return false;
            }
            return true;
        }
        __device__ int get_idx(float16 x, float16 y, float16 center_x, float16 center_y) {
            int idx_x = clamp(get_x_idx(x, center_x), 0, ${width} - 1);
            int idx_y = clamp(get_y_idx(y, center_y), 0, ${height} - 1);
            // Fixed: Row-Major (Row=Y, Col=X)
            return ${width} * idx_y + idx_x;
        }
        __device__ int get_map_idx(int idx, int layer_n) {
            const int layer = ${width} * ${height};
            return layer * layer_n + idx;
        }
        __device__ float transform_p(float16 x, float16 y, float16 z,
                                     float16 r0, float16 r1, float16 r2, float16 t) {
            return r0 * x + r1 * y + r2 * z + t;
        }
        __device__ float point_noise(float16 x, float16 y, float16 z){
            // Noise model based on squared range in the sensor frame.
            // This avoids v=0 for flat ground points (z=0) and works for both
            // depth-camera optical frames (where z is range) and generic frames.
            return ${sensor_noise_factor} * (x * x + y * y + z * z);
        }

        __device__ float point_sensor_distance(float16 x, float16 y, float16 z,
                                               float16 sx, float16 sy, float16 sz) {
            float d = (x - sx) * (x - sx) + (y - sy) * (y - sy) + (z - sz) * (z - sz);
            return d;
        }

        __device__ bool is_valid(float16 x, float16 y, float16 z,
                               float16 sx, float16 sy, float16 sz) {
            float d = point_sensor_distance(x, y, z, sx, sy, sz);
            float dxy = max(sqrt((x - sx) * (x - sx) + (y - sy) * (y - sy)) - ${ramped_height_range_b}, 0.0);
            if (d < ${min_valid_distance} * ${min_valid_distance}) {
                return false;
            }
            else if (z - sz > dxy * ${ramped_height_range_a} + ${ramped_height_range_c} || z - sz > ${max_height_range}) {
                return false;
            }
            else {
                return true;
            }
        }

        __device__ float ray_vector(float16 tx, float16 ty, float16 tz,
                                    float16 px, float16 py, float16 pz,
                                    float16& rx, float16& ry, float16& rz){
            float16 vx = px - tx;
            float16 vy = py - ty;
            float16 vz = pz - tz;
            float16 norm = sqrt(vx * vx + vy * vy + vz * vz);
            if (norm > 0) {
                rx = vx / norm;
                ry = vy / norm;
                rz = vz / norm;
            }
            else {
                rx = 0;
                ry = 0;
                rz = 0;
            }
            return norm;
        }

        __device__ float inner_product(float16 x1, float16 y1, float16 z1,
                                       float16 x2, float16 y2, float16 z2) {

            float product = (x1 * x2 + y1 * y2 + z1 * z2);
            return product;
       }

        """
    ).substitute(
        resolution=resolution,
        width=width,
        height=height,
        sensor_noise_factor=sensor_noise_factor,
        min_valid_distance=min_valid_distance,
        max_height_range=max_height_range,
        ramped_height_range_a=ramped_height_range_a,
        ramped_height_range_b=ramped_height_range_b,
        ramped_height_range_c=ramped_height_range_c,
    )
    return util_preamble

def propose_points_kernel(
    resolution,
    width,
    height,
    sensor_noise_factor,
    mahalanobis_thresh,
    outlier_variance,
    wall_num_thresh,
    max_ray_length,
    cleanup_step,
    min_valid_distance,
    max_height_range,
    cleanup_cos_thresh,
    ramped_height_range_a,
    ramped_height_range_b,
    ramped_height_range_c,
    enable_edge_shaped=True,
    enable_visibility_cleanup=True,
    select_rays=False,
):
    """Read one immutable scan snapshot; write only endpoint/ray proposals.

    ``newmap`` layers: height sum, variance sum, accepted count, drift-inlier
    count, valid input count, outlier count, reserved. Layers 3/4 are completed
    by the preceding counting pass and read-only here. ``visibility`` layers:
    removal weight sum, contradiction count, minimum proposed upper bound.
    Both proposal buffers are float64; the map and per-point estimates float32.
    """
    proposal_kernel = cp.ElementwiseKernel(
        in_params=("raw U center_x, raw U center_y, raw U R, raw U t, raw U map, raw U norm_map"
                   + (", raw int32 ray_slots, raw int32 ray_winners" if select_rays else "")),
        out_params="raw U p, raw T newmap, raw T visibility",
        preamble=map_utils(
            resolution,
            width,
            height,
            sensor_noise_factor,
            min_valid_distance,
            max_height_range,
            ramped_height_range_a,
            ramped_height_range_b,
            ramped_height_range_c,
        ) + """
        // Finite visibility proposals, initialized to +inf. All contenders write
        // only this buffer; the previous map is immutable during the scan.
        __device__ void atomic_min_double(double* address, double value) {
            unsigned long long* bits = (unsigned long long*)address;
            unsigned long long old = atomicCAS(bits, 0ULL, 0ULL);
            while (value < __longlong_as_double(old)) {
                unsigned long long assumed = old;
                old = atomicCAS(bits, assumed, __double_as_longlong(value));
                if (old == assumed) break;
            }
        }
        """,
        operation=string.Template(
            """
            U rx = p[i * 3];
            U ry = p[i * 3 + 1];
            U rz = p[i * 3 + 2];
            U x = transform_p(rx, ry, rz, R[0], R[1], R[2], t[0]);
            U y = transform_p(rx, ry, rz, R[3], R[4], R[5], t[1]);
            U z = transform_p(rx, ry, rz, R[6], R[7], R[8], t[2]);
            U v = point_noise(rx, ry, rz);
            int idx = get_idx(x, y, center_x[0], center_y[0]);
            if (is_valid(x, y, z, t[0], t[1], t[2])) {
                if (is_inside(idx)) {
                    U map_h = map[get_map_idx(idx, 0)];
                    U map_v = map[get_map_idx(idx, 1)];
                    U num_points = newmap[get_map_idx(idx, 4)];
                    if (abs(map_h - z) > (map_v * ${mahalanobis_thresh})) {
                        atomicAdd(&newmap[get_map_idx(idx, 5)], 1.0);
                    }
                    else {
                        if (${enable_edge_shaped} && (num_points > ${wall_num_thresh}) && (z < map_h - map_v * ${mahalanobis_thresh} / num_points)) {
                          // continue;
                        }
                        else {
                            U new_h = (map_h * v + z * map_v) / (map_v + v);
                            U new_v = (map_v * v) / (map_v + v);
                            atomicAdd(&newmap[get_map_idx(idx, 0)], (T)new_h);
                            atomicAdd(&newmap[get_map_idx(idx, 1)], (T)new_v);
                            atomicAdd(&newmap[get_map_idx(idx, 2)], 1.0);
                            // Endpoint acceptance is committed once per cell.
                        }
                        // visibility cleanup
                    }
                }
            }
            if (${cleanup_condition}) {
                float16 ray_x, ray_y, ray_z;
                float16 ray_length = ray_vector(t[0], t[1], t[2], x, y, z, ray_x, ray_y, ray_z);
                ray_length = min(ray_length, (float16)${max_ray_length});
                int last_nidx = -1;
                for (float16 s=${ray_step}; s < ray_length; s+=${ray_step}) {
                    // iterate through ray
                    U nx = t[0] + ray_x * s;
                    U ny = t[1] + ray_y * s;
                    U nz = t[2] + ray_z * s;
                    int nidx = get_idx(nx, ny, center_x[0], center_y[0]);
                    if (last_nidx == nidx) {continue;}  // Skip if we're still in the same cell
                    else {last_nidx = nidx;}
                    if (!is_inside(nidx)) {continue;}

                    U nmap_h = map[get_map_idx(nidx, 0)];
                    U nmap_v = map[get_map_idx(nidx, 1)];
                    U nmap_valid = map[get_map_idx(nidx, 2)];
                    // Time layer
                    U non_updated_t = map[get_map_idx(nidx, 4)];

                    // If point is close or is farther away than ray length, skip.
                    float16 d = (x - nx) * (x - nx) + (y - ny) * (y - ny) + (z - nz) * (z - nz);
                    if (d < 0.1 || !is_valid(x, y, z, t[0], t[1], t[2])) {continue;}

                    // If invalid, do upper bound check, then skip
                    if (nmap_valid < 0.5) {
                      atomic_min_double(&visibility[get_map_idx(nidx, 2)], (double)nz);
                      continue;
                    }
                    // If updated recently, skip
                    if (non_updated_t < 0.5) {continue;}

                    if (nmap_h > nz + 0.01 - min(nmap_v, 1.0) * 0.05) {
                        // If ray and norm is vertical, skip
                        U norm_x = norm_map[get_map_idx(nidx, 0)];
                        U norm_y = norm_map[get_map_idx(nidx, 1)];
                        U norm_z = norm_map[get_map_idx(nidx, 2)];
                        float product = inner_product(ray_x, ray_y, ray_z, norm_x, norm_y, norm_z);
                        if (fabs(product) < ${cleanup_cos_thresh}) {continue;}
                        U num_points = newmap[get_map_idx(nidx, 3)];
                        if (num_points > ${wall_num_thresh} && non_updated_t < 1.0) {continue;}

                        // Finally, this cell is penetrated by the ray.
                        // Proposals only. A valid same-scan endpoint takes
                        // precedence over this evidence in finalization.
                        U removal = ${cleanup_step}/(ray_length / ${max_ray_length});
                        atomicAdd(&visibility[get_map_idx(nidx, 0)], (T)removal);
                        atomicAdd(&visibility[get_map_idx(nidx, 1)], 1.0);
                        atomic_min_double(&visibility[get_map_idx(nidx, 2)], (double)nz);
                    }
                }
            }
            p[i * 3]= idx;
            p[i * 3 + 1] = is_valid(x, y, z, t[0], t[1], t[2]);
            p[i * 3 + 2] = is_inside(idx);
            """
        ).substitute(
            mahalanobis_thresh=mahalanobis_thresh,
            outlier_variance=outlier_variance,
            wall_num_thresh=wall_num_thresh,
            ray_step=resolution / 2 ** 0.5,
            max_ray_length=max_ray_length,
            cleanup_step=cleanup_step,
            cleanup_cos_thresh=cleanup_cos_thresh,
            enable_edge_shaped=int(enable_edge_shaped),
            cleanup_condition=(str(int(enable_visibility_cleanup)) +
                (" && (ray_slots[i] == -2 || (ray_slots[i] >= 0 && ray_winners[ray_slots[i]] == i))"
                 if select_rays else "")),
        ),
        name="propose_points_kernel",
    )
    return proposal_kernel

def error_counting_kernel(
    resolution,
    width,
    height,
    sensor_noise_factor,
    mahalanobis_thresh,
    outlier_variance,
    traversability_inlier,
    min_valid_distance,
    max_height_range,
    ramped_height_range_a,
    ramped_height_range_b,
    ramped_height_range_c,
    select_rays=False,
    ray_voxel_size=0.05,
):
    error_counting_kernel = cp.ElementwiseKernel(
        in_params=("raw U map, raw U p, raw U center_x, raw U center_y, raw U R, raw U t"
                   + (", raw U center_z, uint64 ray_mask" if select_rays else "")),
        out_params=("raw T newmap, raw T error, raw T error_cnt"
                    + (", raw uint64 ray_keys, raw int32 ray_winners, raw int32 ray_slots" if select_rays else "")),
        preamble=map_utils(
            resolution,
            width,
            height,
            sensor_noise_factor,
            min_valid_distance,
            max_height_range,
            ramped_height_range_a,
            ramped_height_range_b,
            ramped_height_range_c,
        ) + (voxel_selector_preamble(ray_voxel_size) if select_rays else ""),
        operation=string.Template(
            """
            U rx = p[i * 3];
            U ry = p[i * 3 + 1];
            U rz = p[i * 3 + 2];
            U x = transform_p(rx, ry, rz, R[0], R[1], R[2], t[0]);
            U y = transform_p(rx, ry, rz, R[3], R[4], R[5], t[1]);
            U z = transform_p(rx, ry, rz, R[6], R[7], R[8], t[2]);
            U v = point_noise(rx, ry, rz);
            // if (!is_valid(z, t[2])) {return;}
            ${ray_selection_and_validation}
            // if ((x - t[0]) * (x - t[0]) + (y - t[1]) * (y - t[1]) + (z - t[2]) * (z - t[2]) < 0.5) {return;}
            int idx = get_idx(x, y, center_x[0], center_y[0]);
            if (!is_inside(idx)) {
                return;
            }
            U map_h = map[get_map_idx(idx, 0)];
            U map_v = map[get_map_idx(idx, 1)];
            U map_valid = map[get_map_idx(idx, 2)];
            U map_t = map[get_map_idx(idx, 3)];
            if (map_valid > 0.5 && (abs(map_h - z) < (map_v * ${mahalanobis_thresh}))
                && map_v < ${outlier_variance} / 2.0
                && map_t > ${traversability_inlier}) {
                T e = z - map_h;
                atomicAdd(&error[0], e);
                atomicAdd(&error_cnt[0], 1);
                atomicAdd(&newmap[get_map_idx(idx, 3)], 1.0);
            }
            atomicAdd(&newmap[get_map_idx(idx, 4)], 1.0);
            """
        ).substitute(
            mahalanobis_thresh=mahalanobis_thresh,
            outlier_variance=outlier_variance,
            traversability_inlier=traversability_inlier,
            ray_selection_and_validation=(
                "ray_slots[i] = -1;\n"
                "if (!is_valid(x, y, z, t[0], t[1], t[2])) {return;}\n"
                "select_voxel_ray((int)i,x,y,z,center_z[0],ray_mask,"
                "(unsigned long long*)&ray_keys[0],&ray_winners[0],&ray_slots[0]);"
                if select_rays else "if (!is_valid(x, y, z, t[0], t[1], t[2])) {return;}"),
        ),
        name="error_counting_kernel",
    )
    return error_counting_kernel

def finalize_map_kernel(width, height, max_variance, initial_variance, outlier_variance):
    """One writer per cell; accepted endpoint fusion takes precedence.

    Adapted from leggedrobotics/elevation_mapping_cupy v2.2.0 (MIT).
    See UPSTREAM.md. The previous/proposal buffers must not alias the output.
    """
    return cp.ElementwiseKernel(
        in_params="raw U previous, raw T newmap, raw T visibility",
        out_params="raw U map",
        preamble=string.Template(
            """
            __device__ int get_map_idx(int idx, int layer_n) {
                const int layer = ${width} * ${height};
                return layer * layer_n + idx;
            }
            """
        ).substitute(width=width, height=height),
        operation=string.Template(
            """
            for (int layer = 0; layer < 7; ++layer) {
                map[get_map_idx(i, layer)] = previous[get_map_idx(i, layer)];
            }

            U new_count = newmap[get_map_idx(i, 2)];
            U outlier_count = newmap[get_map_idx(i, 5)];
            if (new_count > 0) {
                U new_height = newmap[get_map_idx(i, 0)] / new_count;
                U new_variance = newmap[get_map_idx(i, 1)] / new_count;
                map[get_map_idx(i, 0)] = new_height;
                map[get_map_idx(i, 1)] = new_variance;
                map[get_map_idx(i, 2)] = 1.0;
                map[get_map_idx(i, 4)] = 0.0;
                map[get_map_idx(i, 5)] = new_height;
                map[get_map_idx(i, 6)] = 0.0;
                if (new_variance > ${max_variance}) {
                    map[get_map_idx(i, 0)] = 0.0;
                    map[get_map_idx(i, 1)] = ${initial_variance};
                    map[get_map_idx(i, 2)] = 0.0;
                }
            }
            else {
                map[get_map_idx(i, 1)] += outlier_count * ${outlier_variance};
                U cleanup = visibility[get_map_idx(i, 0)];
                if (cleanup > 0.0) {
                    map[get_map_idx(i, 2)] -= cleanup;
                    map[get_map_idx(i, 1)] += (
                        visibility[get_map_idx(i, 1)] * ${outlier_variance}
                    );
                }

                U proposed_upper_bound = visibility[get_map_idx(i, 2)];
                U previous_upper_bound = previous[get_map_idx(i, 5)];
                U previous_has_upper_bound = previous[get_map_idx(i, 6)];
                if (
                    isfinite(proposed_upper_bound)
                    && (
                        previous_has_upper_bound < 0.5
                        || proposed_upper_bound < previous_upper_bound
                    )
                ) {
                    map[get_map_idx(i, 5)] = proposed_upper_bound;
                    map[get_map_idx(i, 6)] = 1.0;
                }
            }

            if (map[get_map_idx(i, 2)] < 0.5) {
                map[get_map_idx(i, 0)] = 0.0;
                map[get_map_idx(i, 1)] = ${initial_variance};
                map[get_map_idx(i, 2)] = 0.0;
            }
            """
        ).substitute(
            max_variance=max_variance,
            initial_variance=initial_variance,
            outlier_variance=outlier_variance,
        ),
        name=f"finalize_map_{width}_{height}",
    )


def dilation_filter_kernel(width, height, dilation_size):
    dilation_filter_kernel = cp.ElementwiseKernel(
        in_params="raw U map, raw U mask",
        out_params="raw U newmap, raw U newmask",
        preamble=string.Template(
            """
            __device__ int get_map_idx(int idx, int layer_n) {
                const int layer = ${width} * ${height};
                return layer * layer_n + idx;
            }

            __device__ int get_relative_map_idx(int idx, int dx, int dy, int layer_n) {
                const int layer = ${width} * ${height};
                const int relative_idx = idx + ${width} * dy + dx;
                return layer * layer_n + relative_idx;
            }
            __device__ bool is_inside(int idx) {
                // Fixed: Row-Major (Row=Y, Col=X)
                // Row index (Y)
                int idx_y = idx / ${width};
                // Column index (X)
                int idx_x = idx % ${width};
                // Check Col bounds (Width)
                if (idx_x <= 0 || idx_x >= ${width} - 1) {
                    return false;
                }
                // Check Row bounds (Height)
                if (idx_y <= 0 || idx_y >= ${height} - 1) {
                    return false;
                }
                return true;
            }
            """
        ).substitute(width=width, height=height),
        operation=string.Template(
            """
            U h = map[get_map_idx(i, 0)];
            U valid = mask[get_map_idx(i, 0)];
            newmap[get_map_idx(i, 0)] = h;
            if (valid < 0.5) {
                U distance = 100;
                U near_value = 0;
                for (int dy = -${dilation_size}; dy <= ${dilation_size}; dy++) {
                    for (int dx = -${dilation_size}; dx <= ${dilation_size}; dx++) {
                        int idx = get_relative_map_idx(i, dx, dy, 0);
                        if (!is_inside(idx)) {continue;}
                        U valid = mask[idx];
                        if(valid > 0.5 && dx + dy < distance) {
                            distance = dx + dy;
                            near_value = map[idx];
                        }
                    }
                }
                if(distance < 100) {
                    newmap[get_map_idx(i, 0)] = near_value;
                    newmask[get_map_idx(i, 0)] = 1.0;
                }
            }
            """
        ).substitute(dilation_size=dilation_size),
        name="dilation_filter_kernel",
    )
    return dilation_filter_kernel

def normal_filter_kernel(width, height, resolution):
    normal_filter_kernel = cp.ElementwiseKernel(
        in_params="raw U map, raw U mask",
        out_params="raw U newmap",
        preamble=string.Template(
            """
            __device__ int get_map_idx(int idx, int layer_n) {
                const int layer = ${width} * ${height};
                return layer * layer_n + idx;
            }

            __device__ int get_relative_map_idx(int idx, int dx, int dy, int layer_n) {
                const int layer = ${width} * ${height};
                const int relative_idx = idx + ${width} * dy + dx;
                return layer * layer_n + relative_idx;
            }
            __device__ bool is_inside(int idx) {
                // Fixed: Row-Major (Row=Y, Col=X)
                // Row index (Y)
                int idx_y = idx / ${width};
                // Column index (X)
                int idx_x = idx % ${width};
                // Check Col bounds (Width)
                if (idx_x <= 0 || idx_x >= ${width} - 1) {
                    return false;
                }
                // Check Row bounds (Height)
                if (idx_y <= 0 || idx_y >= ${height} - 1) {
                    return false;
                }
                return true;
            }
            __device__ float resolution() {
                return ${resolution};
            }
            """
        ).substitute(width=width, height=height, resolution=resolution),
        operation=string.Template(
            """
            U h = map[get_map_idx(i, 0)];
            U valid = mask[get_map_idx(i, 0)];
            if (valid > 0.5) {
                int idx_x = get_relative_map_idx(i, 1, 0, 0);
                int idx_y = get_relative_map_idx(i, 0, 1, 0);
                if (!is_inside(idx_x) || !is_inside(idx_y)) { return; }
                float dzdx = (map[idx_x] - h);
                float dzdy = (map[idx_y] - h);
                // Fixed: Normal = (-dH/dx, -dH/dy, 1)
                float nx = -dzdx / resolution();
                float ny = -dzdy / resolution();
                float nz = 1;
                float norm = sqrt((nx * nx) + (ny * ny) + 1);
                newmap[get_map_idx(i, 0)] = nx / norm;
                newmap[get_map_idx(i, 1)] = ny / norm;
                newmap[get_map_idx(i, 2)] = nz / norm;
            }
            """
        ).substitute(),
        name="normal_filter_kernel",
    )
    return normal_filter_kernel
