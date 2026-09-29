// Copyright (c) 2020, the YACCLAB contributors, as
// shown by the AUTHORS file. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.
//
// See
// https://iris.unimore.it/bitstream/11380/1179616/1/2018_TPDS_Optimized_Block_Based_Algorithms_to_Label_Connected_Components_on_GPUs.pdf
// for more details.
//
// This file was started from the YACCLAB implementation, but is now quite
// different to match what the april tag detection algorithm wants.  The notable
// changes are:
//  * 4 way connectivity on the background at the same time as the foreground.
//  * 0 and 255 are the only colors considered, 127 means "I'm my own blob".
//  * Blob size is also computed in parallel to aid blob size filtering.
//  * Each 32x32 pixel tile is labeled in shared memory first, and only the
//    links between tiles go through global memory.
//

#include "frc/orin/labeling_allegretti_2019_BKE.h"

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "frc/orin/apriltag_types.h"
#include "frc/orin/cuda.h"

namespace {

//         This is a block-based algorithm.
// Blocks are 2x2 sized, with internal pixels named as:
//                       +---+
//                       |a b|
//                       |c d|
//                       +---+
//
//       Neighbour blocks of block X are named as:
//                      +-+-+-+
//                      |P|Q|R|
//                      +-+-+-+
//                      |S|X|
//                      +-+-+
//
// Each block has up to three nodes: the foreground node, for its 255 pixels,
// at a's index, the left background node, for a and c when they are 0, at c's
// index, and the right background node, for b and d when they are 0, at d's
// index.  The foreground is 8 way connected and the background 4 way, which
// links a node to the P, Q, R and S nodes of the same kind as InitLabeling in
// YACCLAB does.  Every component is labeled with the smallest index of its
// nodes, so the labels don't depend on the order anything runs in.

// A tile is kTileBlocks x kTileBlocks blocks, labeled by one thread per block.
constexpr uint32_t kTileBlocks = 16;
constexpr uint32_t kTilePixels = 2 * kTileBlocks;
constexpr uint32_t kTileThreads = kTileBlocks * kTileBlocks;
// Nodes in a tile are numbered y * kTilePixels + x, which orders them the same
// way as their image indices.
constexpr uint32_t kTileNodes = kTilePixels * kTilePixels;
constexpr uint32_t kLocalBits = 10;
constexpr uint32_t kLocalMask = kTileNodes - 1;
static_assert(kTileNodes == 1u << kLocalBits);

// The node kinds, and their shift into a tile_roots entry.
constexpr uint32_t kForeground = 0;
constexpr uint32_t kLeftBackground = 1;
constexpr uint32_t kRightBackground = 2;

// Returns the image index of node `local` of tile (tile_x, tile_y).
__device__ __forceinline__ uint32_t ImageNode(uint32_t tile_x, uint32_t tile_y,
                                              uint32_t local, uint32_t step) {
  return (tile_y * kTilePixels + local / kTilePixels) * step +
         tile_x * kTilePixels + local % kTilePixels;
}

// Returns the root of n in a union-find forest in shared memory, pointing each
// node on the way at its grandparent (path splitting).  Other threads' unions
// run concurrently, so the reads are volatile to see their links, and the
// shortcuts are atomicMins so a parent only ever decreases.
//
// A concurrent union can relink n after this reads its parent, and the shortcut
// then moves n out of the tree that union just put it in.  No join is lost:
// every node that has been n's parent stays joined to the others, by current
// links and the pairs unions still hold, through nodes smaller than n.  Those
// aren't in n's subtree, which is all a write to parent[n] moves.  When a
// SharedUnion's atomicMin lowers a parent that is no longer a root, the union
// carries on with the new and old parents, and a shortcut's gp has been p's
// parent, while p has been n's, so the same holds one level down.  Labels
// depend on SharedUnion carrying on like that.
__device__ __forceinline__ uint32_t SharedFind(uint32_t *parent, uint32_t n) {
  const volatile uint32_t *links = parent;
  uint32_t p = links[n];
  while (p != n) {
    const uint32_t gp = links[p];
    if (gp != p) {
      atomicMin(parent + n, gp);
    }
    n = p;
    p = gp;
  }
  return n;
}

// Merges the trees of a and b in shared memory, linking the larger root under
// the smaller.
__device__ __forceinline__ void SharedUnion(uint32_t *parent, uint32_t a,
                                            uint32_t b) {
  bool done;
  do {
    a = SharedFind(parent, a);
    b = SharedFind(parent, b);
    if (a < b) {
      const uint32_t old = atomicMin(parent + b, a);
      done = (old == b);
      b = old;
    } else if (b < a) {
      const uint32_t old = atomicMin(parent + a, b);
      done = (old == a);
      a = old;
    } else {
      done = true;
    }
  } while (!done);
}

// Returns the root of n in a union-find forest in global memory.
__device__ __forceinline__ uint32_t GlobalFind(const uint32_t *forest,
                                               uint32_t n) {
  uint32_t p = forest[n];
  while (p != n) {
    n = p;
    p = forest[n];
  }
  return n;
}

// Merges the trees of a and b in global memory, linking the larger root under
// the smaller, and points a and b straight at the root.
__device__ void GlobalUnion(uint32_t *forest, uint32_t a, uint32_t b) {
  const uint32_t a_start = a;
  const uint32_t b_start = b;
  bool done;
  do {
    // The two walks are independent, so they step together.
    uint32_t a_next = forest[a];
    uint32_t b_next = forest[b];
    while (a_next != a || b_next != b) {
      if (a_next != a) {
        a = a_next;
        a_next = forest[a];
      }
      if (b_next != b) {
        b = b_next;
        b_next = forest[b];
      }
    }
    if (a < b) {
      const uint32_t old = atomicMin(forest + b, a);
      done = (old == b);
      b = old;
    } else if (b < a) {
      const uint32_t old = atomicMin(forest + a, b);
      done = (old == a);
      a = old;
    } else {
      done = true;
    }
  } while (!done);
  // Pointing a node at an ancestor keeps it in the same tree.  A concurrent
  // union which relinks it first carries on from its old parent, so that link
  // isn't lost either.
  const uint32_t root = min(a, b);
  if (a_start != root && forest[a_start] != root) {
    forest[a_start] = root;
  }
  if (b_start != root && forest[b_start] != root) {
    forest[b_start] = root;
  }
}

// Spreads the low 16 bits of x to the even bits.
__device__ __forceinline__ uint32_t SpreadBits(uint32_t x) {
  x = (x | (x << 8)) & 0x00ff00ffu;
  x = (x | (x << 4)) & 0x0f0f0f0fu;
  x = (x | (x << 2)) & 0x33333333u;
  x = (x | (x << 1)) & 0x55555555u;
  return x;
}

// Returns the first bit of the run containing bit i, where bit k of left_links
// is set when k is linked to k - 1.  Bit 0 must be clear.
__device__ __forceinline__ uint32_t RunStart(uint32_t left_links, uint32_t i) {
  return 31 - __clz(~left_links & ((2u << i) - 1u));
}

__device__ __forceinline__ bool Bit(uint32_t mask, uint32_t i) {
  return (mask >> i) & 1u;
}

// Labels each tile on its own, one thread per block.  Writes each block's three
// tile-local roots to tile_roots, and for each tile-local root, its own index
// to forest and its pixel count to sizes.  Zeros the sizes an all 127 block's
// pixels are labeled with.
//
// Nodes linked to their left neighbor form runs along each block row, which
// ballots find, and the union-find only joins the runs to the runs above.
__global__ void __launch_bounds__(kTileThreads, 6)
    TileLabel(const GpuImage<uint8_t> image, uint32_t label_step,
              uint32_t *tile_roots, uint32_t *forest, uint32_t *sizes) {
  __shared__ uint8_t pixels[kTilePixels][kTilePixels];
  __shared__ uint32_t parent[kTileNodes];
  // The root of each run start, once the unions are done.
  __shared__ uint32_t run_root[kTileNodes];
  __shared__ uint32_t local_size[kTileNodes];
  // Per block row, which nodes are linked to their left neighbor.  Background
  // bit 2i is block i's left node and 2i + 1 its right node.
  __shared__ uint32_t foreground_left_links[kTileBlocks];
  __shared__ uint32_t background_left_links[kTileBlocks];

  const uint32_t tx = threadIdx.x;
  const uint32_t ty = threadIdx.y;
  const uint32_t bx = blockIdx.x * kTileBlocks + tx;
  const uint32_t by = blockIdx.y * kTileBlocks + ty;
  const uint32_t col = 2 * bx;
  const uint32_t row = 2 * by;
  const bool in_image = row < image.rows && col < image.cols;
  const uint32_t lx = 2 * tx;
  const uint32_t ly = 2 * ty;

  // Pixels off the image are 127, which links to nothing.
  uint16_t top = 0x7f7f;
  uint16_t bottom = 0x7f7f;
  if (in_image) {
    top = *reinterpret_cast<const uint16_t *>(image.data + row * image.step +
                                              col);
    bottom = *reinterpret_cast<const uint16_t *>(image.data +
                                                 (row + 1) * image.step + col);
  }
  *reinterpret_cast<uint16_t *>(&pixels[ly][lx]) = top;
  *reinterpret_cast<uint16_t *>(&pixels[ly + 1][lx]) = bottom;

  const uint32_t fg = ly * kTilePixels + lx;
  const uint32_t bgl = fg + kTilePixels;
  const uint32_t bgr = bgl + 1;
  local_size[fg] = 0;
  local_size[bgl] = 0;
  local_size[bgr] = 0;
  __syncthreads();

  const bool a_fg = (top & 0xff) == 255u;
  const bool b_fg = (top >> 8) == 255u;
  const bool c_fg = (bottom & 0xff) == 255u;
  const bool d_fg = (bottom >> 8) == 255u;
  const bool a_bg = (top & 0xff) == 0u;
  const bool b_bg = (top >> 8) == 0u;
  const bool c_bg = (bottom & 0xff) == 0u;
  const bool d_bg = (bottom >> 8) == 0u;

  // The links to other blocks in this tile.
  const bool up = ty > 0;
  const bool left = tx > 0;
  const bool right = tx + 1 < kTileBlocks;
  const bool fg_p = a_fg && up && left && pixels[ly - 1][lx - 1] == 255u;
  const bool fg_q =
      (a_fg || b_fg) && up &&
      (pixels[ly - 1][lx] == 255u || pixels[ly - 1][lx + 1] == 255u);
  const bool fg_r = b_fg && up && right && pixels[ly - 1][lx + 2] == 255u;
  const bool fg_s =
      (a_fg || c_fg) && left &&
      (pixels[ly][lx - 1] == 255u || pixels[ly + 1][lx - 1] == 255u);
  const bool bgl_q = a_bg && up && pixels[ly - 1][lx] == 0u;
  const bool bgl_s = left && ((a_bg && pixels[ly][lx - 1] == 0u) ||
                              (c_bg && pixels[ly + 1][lx - 1] == 0u));
  const bool bgr_q = b_bg && up && pixels[ly - 1][lx + 1] == 0u;
  const bool bgr_s = (a_bg && b_bg) || (c_bg && d_bg);

  // Each half warp is one block row.
  const uint32_t half = 16 * (ty & 1);
  auto row_ballot = [half](bool value) {
    return (__ballot_sync(~0u, value) >> half) & 0xffffu;
  };
  const uint32_t fg_left_links = row_ballot(fg_s);
  const uint32_t fg_up_links = row_ballot(fg_q);
  const uint32_t bg_left_links =
      SpreadBits(row_ballot(bgl_s)) | (SpreadBits(row_ballot(bgr_s)) << 1);
  const uint32_t bg_up_links =
      SpreadBits(row_ballot(bgl_q)) | (SpreadBits(row_ballot(bgr_q)) << 1);
  if (tx == 0) {
    foreground_left_links[ty] = fg_left_links;
    background_left_links[ty] = bg_left_links;
  }

  // The start of a run is its smallest node, and stands for the whole run in
  // the union-find.
  const uint32_t fg_start = ly * kTilePixels + 2 * RunStart(fg_left_links, tx);
  const uint32_t bg_row = bgl - lx;
  const uint32_t bgl_start = bg_row + RunStart(bg_left_links, lx);
  const uint32_t bgr_start = bg_row + RunStart(bg_left_links, lx + 1);
  if (fg_start == fg) {
    parent[fg] = fg;
  }
  if (bgl_start == bgl) {
    parent[bgl] = bgl;
  }
  if (bgr_start == bgr) {
    parent[bgr] = bgr;
  }
  __syncthreads();

  // Joins the runs to the runs above.  A link up is left out when links to the
  // left of it and the runs in both rows already connect its two nodes.  Those
  // links are further left, or a Q link covering a P or R link, so everything
  // left out is covered by a link which is kept.
  uint32_t pending = 0;
  uint32_t fg_q_to = 0, fg_p_to = 0, fg_r_to = 0, bgl_to = 0, bgr_to = 0;
  if (up) {
    const uint32_t fg_left_links_above = foreground_left_links[ty - 1];
    const uint32_t bg_left_links_above = background_left_links[ty - 1];
    const uint32_t fg_above = (ly - 2) * kTilePixels;
    const bool q_left = left && Bit(fg_up_links, tx - 1);
    const bool q_right = right && Bit(fg_up_links, tx + 1);
    if (fg_q &&
        !(Bit(fg_left_links, tx) && q_left && Bit(fg_left_links_above, tx))) {
      pending |= 1;
      fg_q_to = fg_above + 2 * RunStart(fg_left_links_above, tx);
    }
    if (fg_p && !(Bit(fg_left_links, tx) && q_left) &&
        !(fg_q && Bit(fg_left_links_above, tx))) {
      pending |= 2;
      fg_p_to = fg_above + 2 * RunStart(fg_left_links_above, tx - 1);
    }
    if (fg_r && !(fg_q && Bit(fg_left_links_above, tx + 1)) &&
        !(Bit(fg_left_links, tx + 1) && q_right)) {
      pending |= 4;
      fg_r_to = fg_above + 2 * RunStart(fg_left_links_above, tx + 1);
    }
    // When two neighboring background nodes both link up, the 0 pixels above
    // them are next to each other, so the nodes above are joined too.
    const uint32_t bg_above = bg_row - 2 * kTilePixels;
    if (bgl_q &&
        !(left && Bit(bg_left_links, lx) && Bit(bg_up_links, lx - 1))) {
      pending |= 8;
      bgl_to = bg_above + RunStart(bg_left_links_above, lx);
    }
    if (bgr_q && !(Bit(bg_left_links, lx + 1) && Bit(bg_up_links, lx))) {
      pending |= 16;
      bgr_to = bg_above + RunStart(bg_left_links_above, lx + 1);
    }
  }
  // One loop for all the kinds keeps the warp together.
  while (pending) {
    const int k = __ffs(pending) - 1;
    pending &= pending - 1;
    const uint32_t from = k < 3 ? fg_start : k == 3 ? bgl_start : bgr_start;
    const uint32_t to = k == 0   ? fg_q_to
                        : k == 1 ? fg_p_to
                        : k == 2 ? fg_r_to
                        : k == 3 ? bgl_to
                                 : bgr_to;
    SharedUnion(parent, from, to);
  }
  __syncthreads();

  // Find each run start's root.  The three walks are independent, so they step
  // together.
  {
    const uint32_t *walk = parent;
    uint32_t f = fg, l = bgl, r = bgr;
    uint32_t f_next = fg_start == fg ? walk[f] : f;
    uint32_t l_next = bgl_start == bgl ? walk[l] : l;
    uint32_t r_next = bgr_start == bgr ? walk[r] : r;
    while (f_next != f || l_next != l || r_next != r) {
      if (f_next != f) {
        f = f_next;
        f_next = walk[f];
      }
      if (l_next != l) {
        l = l_next;
        l_next = walk[l];
      }
      if (r_next != r) {
        r = r_next;
        r_next = walk[r];
      }
    }
    if (fg_start == fg) {
      run_root[fg] = f;
    }
    if (bgl_start == bgl) {
      run_root[bgl] = l;
    }
    if (bgr_start == bgr) {
      run_root[bgr] = r;
    }
  }
  __syncthreads();

  const uint32_t fg_root = run_root[fg_start];
  const uint32_t bgl_root = run_root[bgl_start];
  const uint32_t bgr_root = run_root[bgr_start];
  const uint32_t fg_count = a_fg + b_fg + c_fg + d_fg;
  const uint32_t bgl_count = a_bg + c_bg;
  const uint32_t bgr_count = b_bg + d_bg;
  if (fg_count) {
    atomicAdd(&local_size[fg_root], fg_count);
  }
  if (bgl_count && bgr_count && bgl_root == bgr_root) {
    atomicAdd(&local_size[bgl_root], bgl_count + bgr_count);
  } else {
    if (bgl_count) {
      atomicAdd(&local_size[bgl_root], bgl_count);
    }
    if (bgr_count) {
      atomicAdd(&local_size[bgr_root], bgr_count);
    }
  }
  __syncthreads();

  if (!in_image) {
    return;
  }

  tile_roots[by * (image.cols / 2) + bx] =
      fg_root | (bgl_root << (kLocalBits * kLeftBackground)) |
      (bgr_root << (kLocalBits * kRightBackground));

  auto write_root = [&](uint32_t local) {
    const uint32_t node = ImageNode(blockIdx.x, blockIdx.y, local, label_step);
    forest[node] = node;
    sizes[node] = local_size[local];
  };
  if (fg_count && fg_root == fg) {
    write_root(fg);
  }
  if (bgl_count && bgl_root == bgl) {
    write_root(bgl);
  }
  if (bgr_count && bgr_root == bgr) {
    write_root(bgr);
  }
  if (fg_count + bgl_count + bgr_count == 0) {
    // FinalLabel labels each pixel of an all 127 block with its own index.
    *reinterpret_cast<uint64_t *>(sizes + row * label_step + col) = 0;
    *reinterpret_cast<uint64_t *>(sizes + (row + 1) * label_step + col) = 0;
  } else if (bx == 0 && by == 0 && fg_count == 0) {
    // And the 127 pixels of other blocks with 0, which is otherwise only the
    // top left block's foreground node.
    sizes[0] = 0;
  }
}

// Returns the image index of the tile-local root of block (bx, by)'s node of
// kind `kind`.
__device__ __forceinline__ uint32_t LocalRoot(const uint32_t *tile_roots,
                                              uint32_t block_cols,
                                              uint32_t label_step, uint32_t bx,
                                              uint32_t by, uint32_t kind) {
  const uint32_t local =
      (tile_roots[by * block_cols + bx] >> (kLocalBits * kind)) & kLocalMask;
  return ImageNode(bx / kTileBlocks, by / kTileBlocks, local, label_step);
}

// Joins the tile-local components across tile edges.  One thread per block on
// a tile's top row or left column.  Each makes the links across the tile edges
// from its block which TileLabel left out, and the left column's also make the
// R links into it from the tile on the left.
//
// As in TileLabel, a link is left out when others already connect its nodes.
// Along a horizontal edge those are the Q links to the left of it and the links
// within the rows on either side, and along a vertical edge the S links above
// it and the links within the columns on either side.  Links within a row or
// column only count when they are inside a tile.
__global__ void BorderMerge(const GpuImage<uint8_t> image, uint32_t label_step,
                            const uint32_t *tile_roots, uint32_t *forest) {
  const uint32_t block_cols = image.cols / 2;
  const uint32_t block_rows = image.rows / 2;
  const uint32_t tiles_x = (block_cols + kTileBlocks - 1) / kTileBlocks;
  const uint32_t tiles_y = (block_rows + kTileBlocks - 1) / kTileBlocks;
  const uint32_t num_horizontal = (tiles_y - 1) * block_cols;
  const uint32_t i = blockIdx.x * blockDim.x + threadIdx.x;
  uint32_t bx;
  uint32_t by;
  if (i < num_horizontal) {
    by = (i / block_cols + 1) * kTileBlocks;
    bx = i % block_cols;
  } else {
    const uint32_t j = i - num_horizontal;
    if (j >= (tiles_x - 1) * block_rows) {
      return;
    }
    by = j % block_rows;
    bx = (j / block_rows + 1) * kTileBlocks;
    if (by % kTileBlocks == 0 && by > 0) {
      // The top row thread has this block.
      return;
    }
  }
  const bool top = by % kTileBlocks == 0 && by > 0;
  const bool left_edge = bx % kTileBlocks == 0 && bx > 0;

  // The pixels from 2 left of the block to 2 right of it, and 2 above it to
  // its bottom, all loaded at once.  Pixels off the image are 127, which links
  // to nothing.
  const int col = 2 * bx;
  const int row = 2 * by;
  uint16_t window[4][3];
#pragma unroll
  for (int y = 0; y < 4; ++y) {
#pragma unroll
    for (int x = 0; x < 3; ++x) {
      const int image_x = col - 2 + 2 * x;
      const int image_y = row - 2 + y;
      window[y][x] = image_x >= 0 && image_y >= 0 &&
                             image_x < static_cast<int>(image.cols) &&
                             image_y < static_cast<int>(image.rows)
                         ? *reinterpret_cast<const uint16_t *>(
                               image.data + image_y * image.step + image_x)
                         : 0x7f7f;
    }
  }
  // The pixel at (col + dx, row + dy).
  auto pixel = [&](int dx, int dy) -> uint32_t {
    return (window[dy + 2][(dx + 2) / 2] >> (8 * ((dx + 2) % 2))) & 0xff;
  };
  auto fg = [&](int dx, int dy) { return pixel(dx, dy) == 255u; };
  auto bg = [&](int dx, int dy) { return pixel(dx, dy) == 0u; };
  // The foreground Q and S links of the block dx blocks right and dy down.
  auto fg_q = [&](int dx) {
    return (fg(2 * dx, 0) || fg(2 * dx + 1, 0)) &&
           (fg(2 * dx, -1) || fg(2 * dx + 1, -1));
  };
  auto fg_s = [&](int dx, int dy) {
    return (fg(2 * dx, 2 * dy) || fg(2 * dx, 2 * dy + 1)) &&
           (fg(2 * dx - 1, 2 * dy) || fg(2 * dx - 1, 2 * dy + 1));
  };
  auto root = [&](uint32_t x, uint32_t y, uint32_t kind) {
    return LocalRoot(tile_roots, block_cols, label_step, x, y, kind);
  };

  if (top) {
    // Whether the links to the blocks on either side are inside a tile.
    const bool left_inside = bx % kTileBlocks != 0;
    const bool right_inside =
        bx % kTileBlocks != kTileBlocks - 1 && bx + 1 < block_cols;
    const bool q = fg_q(0);
    const bool q_left = left_inside && fg_q(-1);
    const bool q_right = right_inside && fg_q(1);
    const bool s = left_inside && fg_s(0, 0);
    const bool s_right = right_inside && fg_s(1, 0);
    const bool s_above = left_inside && fg_s(0, -1);
    const bool s_above_right = right_inside && fg_s(1, -1);
    if (fg(0, 0) && fg(-1, -1) && !(s && q_left) && !(q && s_above)) {
      GlobalUnion(forest, root(bx, by, kForeground),
                  root(bx - 1, by - 1, kForeground));
    }
    if (q && !(s && q_left && s_above)) {
      GlobalUnion(forest, root(bx, by, kForeground),
                  root(bx, by - 1, kForeground));
    }
    if (fg(1, 0) && fg(2, -1) && !(q && s_above_right) &&
        !(s_right && q_right)) {
      GlobalUnion(forest, root(bx, by, kForeground),
                  root(bx + 1, by - 1, kForeground));
    }
    // As in TileLabel, the background nodes above two neighboring links up are
    // joined, so only the links within this row matter.
    const bool bgl_q = bg(0, 0) && bg(0, -1);
    const bool bgr_q = bg(1, 0) && bg(1, -1);
    const bool bgl_s =
        left_inside && ((bg(0, 0) && bg(-1, 0)) || (bg(0, 1) && bg(-1, 1)));
    const bool bgr_q_left = left_inside && bg(-1, 0) && bg(-1, -1);
    const bool bgr_s = (bg(0, 0) && bg(1, 0)) || (bg(0, 1) && bg(1, 1));
    if (bgl_q && !(bgl_s && bgr_q_left)) {
      GlobalUnion(forest, root(bx, by, kLeftBackground),
                  root(bx, by - 1, kLeftBackground));
    }
    if (bgr_q && !(bgr_s && bgl_q)) {
      GlobalUnion(forest, root(bx, by, kRightBackground),
                  root(bx, by - 1, kRightBackground));
    }
  }

  if (left_edge) {
    // Whether the links to the blocks above are inside a tile.
    const bool up_inside = by % kTileBlocks != 0;
    const bool s = fg_s(0, 0);
    const bool q = up_inside && fg_q(0);
    const bool q_left = up_inside && fg_q(-1);
    const bool s_above = up_inside && fg_s(0, -1);
    if (!top && by > 0) {
      if (fg(0, 0) && fg(-1, -1) && !(q && s_above) && !(s && q_left)) {
        GlobalUnion(forest, root(bx, by, kForeground),
                    root(bx - 1, by - 1, kForeground));
      }
      // The left block's R link, into this tile.
      if (fg(-1, 0) && fg(0, -1) && !(q_left && s_above) && !(s && q)) {
        GlobalUnion(forest, root(bx - 1, by, kForeground),
                    root(bx, by - 1, kForeground));
      }
    }
    if (s && !(q && s_above && q_left)) {
      GlobalUnion(forest, root(bx, by, kForeground),
                  root(bx - 1, by, kForeground));
    }
    // The link up from the left background node, and the one from the right
    // background node on the left, join this link to the one above.
    const bool bgl_s = (bg(0, 0) && bg(-1, 0)) || (bg(0, 1) && bg(-1, 1));
    const bool bgl_q = up_inside && bg(0, 0) && bg(0, -1);
    const bool bgr_q_left = up_inside && bg(-1, 0) && bg(-1, -1);
    if (bgl_s && !(bgl_q && bgr_q_left)) {
      GlobalUnion(forest, root(bx, by, kLeftBackground),
                  root(bx - 1, by, kRightBackground));
    }
  }
}

// Labels every pixel with its component's root, and adds each tile-local
// component's size into its root's.  One thread per block, a tile per thread
// block, so only the tile's local roots look up their roots.
__global__ void __launch_bounds__(kTileThreads)
    FinalLabel(const GpuImage<uint8_t> image, GpuImage<uint32_t> labels,
               const uint32_t *tile_roots, const uint32_t *forest,
               uint32_t *sizes) {
  __shared__ uint32_t roots[kTileNodes];

  const uint32_t tx = threadIdx.x;
  const uint32_t ty = threadIdx.y;
  const uint32_t bx = blockIdx.x * kTileBlocks + tx;
  const uint32_t by = blockIdx.y * kTileBlocks + ty;
  const uint32_t col = 2 * bx;
  const uint32_t row = 2 * by;
  const bool in_image = row < image.rows && col < image.cols;
  const uint32_t fg = 2 * ty * kTilePixels + 2 * tx;
  const uint32_t bgl = fg + kTilePixels;
  const uint32_t bgr = bgl + 1;

  uint16_t top = 0x7f7f;
  uint16_t bottom = 0x7f7f;
  uint32_t local_roots = 0;
  if (in_image) {
    top = *reinterpret_cast<const uint16_t *>(image.data + row * image.step +
                                              col);
    bottom = *reinterpret_cast<const uint16_t *>(image.data +
                                                 (row + 1) * image.step + col);
    local_roots = tile_roots[by * (image.cols / 2) + bx];
  }
  const bool a_fg = (top & 0xff) == 255u;
  const bool b_fg = (top >> 8) == 255u;
  const bool c_fg = (bottom & 0xff) == 255u;
  const bool d_fg = (bottom >> 8) == 255u;
  const bool a_bg = (top & 0xff) == 0u;
  const bool b_bg = (top >> 8) == 0u;
  const bool c_bg = (bottom & 0xff) == 0u;
  const bool d_bg = (bottom >> 8) == 0u;
  const bool has_fg = a_fg || b_fg || c_fg || d_fg;
  const bool has_bgl = a_bg || c_bg;
  const bool has_bgr = b_bg || d_bg;
  const uint32_t fg_root = local_roots & kLocalMask;
  const uint32_t bgl_root =
      (local_roots >> (kLocalBits * kLeftBackground)) & kLocalMask;
  const uint32_t bgr_root =
      (local_roots >> (kLocalBits * kRightBackground)) & kLocalMask;

  auto resolve = [&](uint32_t local) {
    const uint32_t node = ImageNode(blockIdx.x, blockIdx.y, local, labels.step);
    const uint32_t root = GlobalFind(forest, node);
    roots[local] = root;
    if (root != node) {
      atomicAdd(sizes + root, sizes[node]);
    }
  };
  if (has_fg && fg_root == fg) {
    resolve(fg);
  }
  if (has_bgl && bgl_root == bgl) {
    resolve(bgl);
  }
  if (has_bgr && bgr_root == bgr) {
    resolve(bgr);
  }
  __syncthreads();

  if (!in_image) {
    return;
  }
  uint32_t a_label;
  uint32_t b_label;
  uint32_t c_label;
  uint32_t d_label;
  if (!has_fg && !has_bgl && !has_bgr) {
    a_label = row * labels.step + col;
    b_label = a_label + 1;
    c_label = a_label + labels.step;
    d_label = c_label + 1;
  } else {
    const uint32_t f = has_fg ? roots[fg_root] : 0;
    const uint32_t l = has_bgl ? roots[bgl_root] : 0;
    const uint32_t r = has_bgr ? roots[bgr_root] : 0;
    a_label = a_fg ? f : a_bg ? l : 0;
    b_label = b_fg ? f : b_bg ? r : 0;
    c_label = c_fg ? f : c_bg ? l : 0;
    d_label = d_fg ? f : d_bg ? r : 0;
  }
  *reinterpret_cast<uint64_t *>(labels.data + row * labels.step + col) =
      (static_cast<uint64_t>(b_label) << 32) | a_label;
  *reinterpret_cast<uint64_t *>(labels.data + (row + 1) * labels.step + col) =
      (static_cast<uint64_t>(d_label) << 32) | c_label;
}

}  // namespace

void LabelImage(const GpuImage<uint8_t> input, GpuImage<uint32_t> output,
                GpuImage<uint32_t> union_markers_size_device,
                uint32_t *tile_roots, uint32_t *forest, cudaStream_t stream,
                std::span<frc::apriltag::CudaEvent> kernel_events) {
  ABSL_CHECK(kernel_events.empty() || kernel_events.size() == 3u);
  auto record = [&](size_t index) {
    if (!kernel_events.empty()) {
      CHECK_CUDA(cudaEventRecord(kernel_events[index].get(), stream));
    }
  };

  ABSL_CHECK_NE(input.rows, 1u);
  ABSL_CHECK_NE(input.cols, 1u);

  // Need an even number of rows and colums, we don't need to solve the actual
  // hard problems...
  ABSL_CHECK_EQ(input.rows % 2, 0u);
  ABSL_CHECK_EQ(input.cols % 2, 0u);
  ABSL_CHECK_EQ(output.rows, input.rows);
  ABSL_CHECK_EQ(output.cols, input.cols);
  // Labels index the sizes.
  ABSL_CHECK_EQ(union_markers_size_device.step, output.step);

  const uint32_t block_cols = input.cols / 2;
  const uint32_t block_rows = input.rows / 2;
  const dim3 tiles((block_cols + kTileBlocks - 1) / kTileBlocks,
                   (block_rows + kTileBlocks - 1) / kTileBlocks, 1);
  const dim3 tile_threads(kTileBlocks, kTileBlocks, 1);

  TileLabel<<<tiles, tile_threads, 0, stream>>>(
      input, output.step, tile_roots, forest, union_markers_size_device.data);
  record(0);

  const uint32_t border_blocks =
      (tiles.y - 1) * block_cols + (tiles.x - 1) * block_rows;
  if (border_blocks > 0) {
    constexpr uint32_t kBorderThreads = 128;
    BorderMerge<<<(border_blocks + kBorderThreads - 1) / kBorderThreads,
                  kBorderThreads, 0, stream>>>(input, output.step, tile_roots,
                                               forest);
  }
  record(1);

  FinalLabel<<<tiles, tile_threads, 0, stream>>>(
      input, output, tile_roots, forest, union_markers_size_device.data);
  record(2);
}
