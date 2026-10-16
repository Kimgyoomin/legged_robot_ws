# SPDX-License-Identifier: MIT
# RUBI representative-ray selection, 2026-10-15.
"""GPU-only representative-ray selection; all points still enter height fusion.

One original point (lowest input index) per map-aligned 3D voxel is selected.
A bounded hash search falls back to tracing the original point on overflow;
collisions never silently discard rays. Buffers grow and are reused per scan.
"""
import string
import cupy as cp


def voxel_selector_preamble(size):
    """Shared CUDA helper for integrated selection and standalone diagnostics."""
    return string.Template(r'''
        __device__ void select_voxel_ray(int point_index, float x, float y, float z,
                float center_z, unsigned long long mask,
                unsigned long long* keys, int* winners, int* slots) {
                slots[point_index] = -2;  // conservative fallback: keep this original ray
                float vx = floorf(x / ${voxel});
                float vy = floorf(y / ${voxel});
                float vz = floorf((z + center_z) / ${voxel});
                if (!(vx >= -1048576 && vx < 1048576 &&
                      vy >= -1048576 && vy < 1048576 &&
                      vz >= -1048576 && vz < 1048576)) return;
                unsigned long long key =
                    ((unsigned long long)((int)vx + 1048576) << 42) |
                    ((unsigned long long)((int)vy + 1048576) << 21) |
                    (unsigned long long)((int)vz + 1048576);
                unsigned long long h = key;
                h = (h ^ (h >> 30)) * 0xbf58476d1ce4e5b9ULL;
                h = (h ^ (h >> 27)) * 0x94d049bb133111ebULL;
                h ^= h >> 31;
                for (int probe = 0; probe < 32; ++probe) {
                    unsigned long long slot = (h + probe) & mask;
                    unsigned long long old = atomicCAS(
                        (unsigned long long*)&keys[slot], 0xffffffffffffffffULL, key);
                    if (old == 0xffffffffffffffffULL || old == key) {
                        atomicMin(&winners[slot], point_index);
                        slots[point_index] = (int)slot;
                        return;
                    }
                }
        }
    ''').substitute(voxel=f'{size:.17e}f')


class VoxelRaySelector:
    def __init__(self, param):
        from .kernels import map_utils
        size = param.ray_voxel_size or param.resolution
        self.capacity = 0
        self.count = 0
        self.keys = cp.empty(0, cp.uint64)
        self.winners = cp.empty(0, cp.int32)
        self.slots = cp.empty(0, cp.int32)
        self.kernel = cp.ElementwiseKernel(
            'raw U p, raw U R, raw U t, raw U center_z, uint64 mask',
            'raw uint64 keys, raw int32 winners, raw int32 slots',
            preamble=map_utils(param.resolution, param.cell_n, param.cell_n,
                param.sensor_noise_factor, param.min_valid_distance,
                param.max_height_range, param.ramped_height_range_a,
                param.ramped_height_range_b, param.ramped_height_range_c) + voxel_selector_preamble(size),
            operation=r'''
                float x = transform_p(p[3*i], p[3*i+1], p[3*i+2], R[0], R[1], R[2], t[0]);
                float y = transform_p(p[3*i], p[3*i+1], p[3*i+2], R[3], R[4], R[5], t[1]);
                float z = transform_p(p[3*i], p[3*i+1], p[3*i+2], R[6], R[7], R[8], t[2]);
                slots[i] = -1;
                if (is_valid(x,y,z,t[0],t[1],t[2])) {
                    select_voxel_ray((int)i,x,y,z,center_z[0],mask,
                        (unsigned long long*)&keys[0],&winners[0],&slots[0]);
                }
            ''',
            name='rubi_select_voxel_rays')

    def prepare(self, count):
        self.count = int(count)
        needed = 1 << (max(2, 2*self.count)-1).bit_length()
        if needed > self.capacity:
            self.capacity = needed
            self.keys = cp.empty(needed, cp.uint64)
            self.winners = cp.empty(needed, cp.int32)
            self.slots = cp.empty(needed//2, cp.int32)
        self.keys.fill(cp.uint64(0xffffffffffffffff))
        self.winners.fill(cp.int32(0x7fffffff))
        return self.slots, self.winners

    def select(self, points, rotation, translation, center_z):
        """Standalone selector for tests. Production fuses selection into counting."""
        self.prepare(points.shape[0])
        if self.count:
            self.kernel(points, rotation, translation, center_z, cp.uint64(self.capacity-1),
                        self.keys, self.winners, self.slots, size=self.count)
        return self.slots, self.winners

    def selected_mask(self):
        """Diagnostic only: no compaction or host synchronization on fusion path."""
        slots = self.slots[:self.count]
        return (slots == -2) | ((slots >= 0) &
            (self.winners[cp.maximum(slots, 0)] == cp.arange(self.count, dtype=cp.int32)))

    def statistics(self):
        """Explicit diagnostic synchronization; never called by the scan callback."""
        return {'input_points': self.count,
                'selected_rays': int(cp.count_nonzero(self.selected_mask()).get()),
                'fallback_rays': int(cp.count_nonzero(self.slots[:self.count] == -2).get()),
                'allocated_bytes': self.keys.nbytes+self.winners.nbytes+self.slots.nbytes}
