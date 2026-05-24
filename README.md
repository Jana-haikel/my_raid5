# my_raid5 — Custom Linux Kernel RAID5 Driver

A high-performance RAID5 implementation built as a Loadable Kernel Module (LKM), running on top of custom RAM block devices. Designed to outperform Linux's native `md` software RAID on read workloads through minimal I/O path overhead, AVX2-optimized parity computation, and aggressive cache-aware tiling.

---

## Overview

This project implements a complete RAID5 stack in kernel space:

| Layer | Component | Description |
|-------|-----------|-------------|
| **RAM Disks** | `rambuff` driver | 3 lightweight block devices backed by system RAM (`xarray` page tree, RCU-protected lookups) |
| **RAID5 Mapper** | `my_raid5` driver | Custom striping with rotating parity, direct bio remapping, parallel writes |

**Key design choice:** The read path is intentionally minimal — map the sector, swap the bio device pointer, resubmit. No cloning, no caching, no synchronization overhead.

---

## Performance

Benchmarked with `fio` 3.41 (`libaio`, `iodepth=1`, 1GB dataset) against Linux `md` RAID5 on identical underlying RAM disks:

| Metric | Linux `md` | `my_raid5` | Delta |
|--------|-----------|-----------|-------|
| Sequential Read BW | 11.9 GiB/s | **13.3 GiB/s** | **+12%** |
| Random Read IOPS | 787k | **854k** | **+8%** |
| Random Read P99.9 Latency | 2.32 µs | **0.35 µs** | **6.5× better** |

Tail latency improvements are the standout result — the minimal read path eliminates stripe cache contention and cross-CPU synchronization that plague general-purpose RAID implementations.

---

## Building
### Prerequisites
1. Linux kernel headers (kernel-devel or linux-headers)
2. GCC with AVX2 support
3. fio for benchmarking

## Current Status
### ✅ Complete:
- RAM disk driver (rambuff) with dynamic page allocation
- RAID5 mapper with rotating parity
- Full-stripe write fast path
- Read-modify-write (RMW) slow path
- Performance benchmarking vs md
### 🚧 In Progress:
- Write path optimization (RMW overhead reduction)
- Async parity computation
- Discard/TRIM support

## Acknowledgments
Built as a personal deep-dive into Linux storage architecture. The performance gains validate that specialized drivers can beat mature general-purpose implementations when the use case allows for architectural simplification.
