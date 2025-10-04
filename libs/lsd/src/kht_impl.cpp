/*
 * Copyright (C) 2008 Leandro A. F. Fernandes and Manuel M. Oliveira
 *
 * author   : Fernandes, Leandro A. F.
 * e-mail   : laffernandes@gmail.com
 * home page: http://www.inf.ufrgs.br/~laffernandes
 *
 *
 * The complete description of the implemented techinique can be found at
 *
 *      Leandro A. F. Fernandes, Manuel M. Oliveira
 *      Real-time line detection through an improved Hough transform voting scheme
 *      Pattern Recognition (PR), Elsevier, 41:1, 2008, 299-314.
 *      DOI: http://dx.doi.org/10.1016/j.patcog.2007.04.003
 *      Project Page: http://www.inf.ufrgs.br/~laffernandes/kht.html
 *
 * If you use this implementation, please reference the above paper.
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <lsd/impl/kht.h>

#include <algorithm>
#include <cstdlib>
#include <limits>
#include <memory.h>

// pi value.
static const double pi = 3.14159265358979323846;

// Kernel-based Hough transform (KHT) for detecting straight lines in images.
void kht(lines_list_t& lines,
         unsigned char* binary_image,
         const size_t image_width,
         const size_t image_height,
         const size_t cluster_min_size,
         const double cluster_min_deviation,
         const double delta,
         const double kernel_min_height,
         const double n_sigmas) {
  static strings_list_t strings;
  static clusters_list_t clusters;
  static accumulator_t accumulator;

  // Group feature pixels from an input binary into clusters of approximately collinear pixels.
  find_strings(strings, binary_image, image_width, image_height, cluster_min_size);
  find_clusters(clusters, strings, cluster_min_deviation, cluster_min_size);

  // Perform the proposed Hough transform voting scheme.
  accumulator.init(image_width, image_height, delta);
  voting(accumulator, clusters, kernel_min_height, n_sigmas);

  // Retrieve the most significant straight lines from the resulting voting map.
  peak_detection(lines, accumulator);
}

// Allocates 2D memory blocks.
void* malloc_2d(const size_t size1, const size_t size2, const size_t data_size) {
  const size_t pointers_size = size1 * sizeof(void*);
  const size_t items_size = size1 * size2 * data_size;

  void* buffer = malloc(pointers_size + items_size);

  void** pointers = static_cast<void**>(buffer);
  char* items = &(static_cast<char*>(buffer))[pointers_size];

  for (size_t i = 0, j = 0, j_inc = size2 * data_size; i != size1; ++i, j += j_inc) {
    pointers[i] = &items[j];
  }

  return buffer;
}

// Sets 2D buffers to a specified character.
void* memset_2d(void* dest, const int c, const size_t size1, const size_t size2, const size_t data_size) {
  if (dest) {
    const size_t pointers_size = size1 * sizeof(void*);
    const size_t items_size = size1 * size2 * data_size;

    char* buffer = static_cast<char*>(dest);

    memset(&buffer[pointers_size], c, items_size);
  }

  return dest;
}

// Reallocate 2D memory blocks.
void* realloc_2d(void* memblock, const size_t size1, const size_t size2, const size_t data_size) {
  const size_t pointers_size = size1 * sizeof(void*);
  const size_t items_size = size1 * size2 * data_size;

  memblock = realloc(memblock, pointers_size + items_size);

  void** pointers = static_cast<void**>(memblock);
  char* items = &(static_cast<char*>(memblock))[pointers_size];

  for (size_t i = 0, j = 0, j_inc = size2 * data_size; i != size1; ++i, j += j_inc) {
    pointers[i] = &items[j];
  }

  return memblock;
}

/* Eigenvectors and Eigenvalues
 * Code from "The Better C Eigenvector Source Code Page"
 * http://www.nauticom.net/www/jdtaft/CEigenBetter.htm
 */
inline void tri_diagonalize(const double* Cxd, double* d, double* e, double* A, int L, double tol) {
  int i, j, k, l;
  double f, g, h, hh;
  for (i = 0; i < L; i++) {
    for (j = 0; j <= i; j++) {
      A[i * L + j] = Cxd[i * L + j];
    }
  }
  for (i = L - 1; i > 0; i--) {
    l = i - 2;
    f = A[i * L + i - 1];
    g = 0.0;
    for (k = 0; k <= l; k++) {
      g += A[i * L + k] * A[i * L + k];
    }
    h = g + f * f;
    if (g <= tol) {
      e[i] = f;
      h = 0.0;
      d[i] = h;
      continue;
    }
    l++;
    g = sqrt(h);
    if (f >= 0.0) g = -g;
    e[i] = g;
    h = h - f * g;
    A[i * L + i - 1] = f - g;
    f = 0.0;
    for (j = 0; j <= l; j++) {
      A[j * L + i] = A[i * L + j] / h;
      g = 0.0;
      for (k = 0; k <= j; k++) {
        g += A[j * L + k] * A[i * L + k];
      }
      for (k = j + 1; k <= l; k++) {
        g += A[k * L + j] * A[i * L + k];
      }
      e[j] = g / h;
      f += g * A[j * L + i];
    }
    hh = f / (h + h);
    for (j = 0; j <= l; j++) {
      f = A[i * L + j];
      g = e[j] - hh * f;
      e[j] = g;
      for (k = 0; k <= j; k++) {
        A[j * L + k] = A[j * L + k] - f * e[k] - g * A[i * L + k];
      }
    }
    d[i] = h;
  }
  d[0] = e[0] = 0.0;
  for (i = 0; i < L; i++) {
    l = i - 1;
    if (d[i] != 0.0) {
      for (j = 0; j <= l; j++) {
        g = 0.0;
        for (k = 0; k <= l; k++) {
          g += A[i * L + k] * A[k * L + j];
        }
        for (k = 0; k <= l; k++) {
          A[k * L + j] = A[k * L + j] - g * A[k * L + i];
        }
      }
    }
    d[i] = A[i * L + i];
    A[i * L + i] = 1.0;
    for (j = 0; j <= l; j++) {
      A[i * L + j] = A[j * L + i] = 0.0;
    }
  }
}

/* Eigenvectors and Eigenvalues
 * Code from "The Better C Eigenvector Source Code Page"
 * http://www.nauticom.net/www/jdtaft/CEigenBetter.htm
 */
inline static int calc_eigenstructure(double* d, double* e, double* A, int L, double macheps) {
  int i, j, k, l, m;
  double b, c, f, g, h, p, r, s;

  for (i = 1; i < L; i++) e[i - 1] = e[i];
  e[L - 1] = b = f = 0.0;
  for (l = 0; l < L; l++) {
    h = macheps * (fabs(d[l]) + fabs(e[l]));
    if (b < h) b = h;
    for (m = l; m < L; m++) {
      if (fabs(e[m]) <= b) break;
    }
    j = 0;
    if (m != l) do {
        if (j++ == 30) return -1;
        p = (d[l + 1] - d[l]) / (2.0 * e[l]);
        r = sqrt(p * p + 1);
        h = d[l] - e[l] / (p + (p < 0.0 ? -r : r));
        for (i = l; i < L; i++) d[i] = d[i] - h;
        f += h;
        p = d[m];
        c = 1.0;
        s = 0.0;
        for (i = m - 1; i >= l; i--) {
          g = c * e[i];
          h = c * p;
          if (fabs(p) >= fabs(e[i])) {
            c = e[i] / p;
            r = sqrt(c * c + 1);
            e[i + 1] = s * p * r;
            s = c / r;
            c = 1.0 / r;
          } else {
            c = p / e[i];
            r = sqrt(c * c + 1);
            e[i + 1] = s * e[i] * r;
            s = 1.0 / r;
            c = c / r;
          }
          p = c * d[i] - s * g;
          d[i + 1] = h + s * (c * g + s * d[i]);
          for (k = 0; k < L; k++) {
            h = A[k * L + i + 1];
            A[k * L + i + 1] = s * A[k * L + i] + c * h;
            A[k * L + i] = c * A[k * L + i] - s * h;
          }
        }
        e[l] = s * p;
        d[l] = c * p;
      } while (fabs(e[l]) > b);
    d[l] = d[l] + f;
  }

  /* order the eigenvectors  */
  for (i = 0; i < L; i++) {
    k = i;
    p = d[i];
    for (j = i + 1; j < L; j++) {
      if (d[j] > p) {
        k = j;
        p = d[j];
      }
    }
    if (k != i) {
      d[k] = d[i];
      d[i] = p;
      for (j = 0; j < L; j++) {
        p = A[j * L + i];
        A[j * L + i] = A[j * L + k];
        A[j * L + k] = p;
      }
    }
  }
  return 0;
}

// Computes the decomposition of a matrix into matrices composed of its eigenvectors and eigenvalues.
void eigen(matrix_t& vectors, matrix_t& values, const matrix_t& matrix) {
  double temp[2];

  tri_diagonalize(matrix, values, temp, vectors, 2, 1.0e-6);
  calc_eigenstructure(values, temp, vectors, 2, 1.0e-16);

  values[3] = values[1];
  values[1] = values[2] = 0.0;
}

// An elliptical-Gaussian kernel.
struct kernel_t {
  const cluster_t* pcluster;

  double rho;
  double theta;

  matrix_t lambda;  // [sigma^2_rho sigma_rhotheta; sigma_rhotheta sigma^2_theta]

  size_t rho_index;    // [1,rho_size] range
  size_t theta_index;  // [1,theta_size] range

  double height;
};

// Specifies a list of Gaussian kernels.
typedef list<kernel_t, 1000> kernels_list_t;

// Specifies a list of pointers to Gaussian kernels.
typedef list<kernel_t*, 1000> pkernels_list_t;

// Bi-variated Gaussian distribution.
inline double gauss(const double rho,
                    const double theta,
                    const double sigma2_rho,
                    const double sigma2_theta,
                    const double sigma_rho_sigma_theta,
                    const double two_r,
                    const double a,
                    const double b) {
  double z =
      ((rho * rho) / sigma2_rho) - ((two_r * rho * theta) / sigma_rho_sigma_theta) + ((theta * theta) / sigma2_theta);
  return a * exp(-z * b);
}

// Bi-variated Gaussian distribution.
inline double gauss(const double rho,
                    const double theta,
                    const double sigma2_rho,
                    const double sigma2_theta,
                    const double sigma_rho_theta) {
  /* Leandro A. F. Fernandes, Manuel M. Oliveira
   * Real-time line detection through an improved Hough transform voting scheme
   * Pattern Recognition (PR), Elsevier, 41:1, 2008, 299-314.
   *
   * Equation 15
   */

  const double sigma_rho_sigma_theta = sqrt(sigma2_rho) * sqrt(sigma2_theta);
  const double r = (sigma_rho_theta / sigma_rho_sigma_theta), two_r = 2.0 * r;
  const double a = 1.0 / (2.0 * pi * sigma_rho_sigma_theta * sqrt(1.0 - (r * r)));
  const double b = 1.0 / (2.0 * (1.0 - (r * r)));
  return gauss(rho, theta, sigma2_rho, sigma2_theta, sigma_rho_sigma_theta, two_r, a, b);
}

// Solves the uncertainty propagation.
inline void solve(matrix_t& result, const matrix_t& nabla, const matrix_t& lambda) {
  matrix_t result1 = {}, result2 = {};

  for (size_t i = 0, i_line = 0; i < 2; ++i, i_line += 2) {
    for (size_t j = 0; j < 2; ++j) {
      for (size_t k = 0; k < 2; ++k) {
        result1[i_line + j] += nabla[i_line + k] * lambda[(k * 2) + j];
      }
    }
  }

  for (size_t i = 0, i_line = 0; i < 2; ++i, i_line += 2) {
    for (size_t j = 0; j < 2; ++j) {
      for (size_t k = 0; k < 2; ++k) {
        result2[i_line + j] += result1[i_line + k] * nabla[(j * 2) + k];
      }
    }
  }

  memcpy(result, result2, 4 * sizeof(double));
}

// This function complements the linking procedure.
inline bool next(
    int& x_seed, int& y_seed, const unsigned char* binary_image, const size_t image_width, const size_t image_height) {
  /* Leandro A. F. Fernandes, Manuel M. Oliveira
   * Real-time line detection through an improved Hough transform voting scheme
   * Pattern Recognition (PR), Elsevier, 41:1, 2008, 299-314.
   *
   * Algorithm 6
   */

  int x, y;
  static const int X_OFFSET[8] = {0, 1, 0, -1, 1, -1, -1, 1};
  static const int Y_OFFSET[8] = {1, 0, -1, 0, 1, 1, -1, -1};

  for (size_t i = 0; i != 8; ++i) {
    x = x_seed + X_OFFSET[i];
    if (x >= 0 && static_cast<size_t>(x) < image_width) {
      y = y_seed + Y_OFFSET[i];
      if (y >= 0 && static_cast<size_t>(y) < image_height) {
        const size_t xi = static_cast<size_t>(x);
        const size_t yi = static_cast<size_t>(y);
        const size_t idx = yi * image_width + xi;
        if (binary_image[idx]) {
          x_seed = x;
          y_seed = y;
          return true;
        }
      }
    }
  }
  return false;
}

// Creates a string of neighboring edge pixels.
inline void linking_procedure(string_t& string,
                              unsigned char* binary_image,
                              const size_t image_width,
                              const size_t image_height,
                              const int x_ref,
                              const int y_ref,
                              const double half_width,
                              const double half_height) {
  /* Leandro A. F. Fernandes, Manuel M. Oliveira
   * Real-time line detection through an improved Hough transform voting scheme
   * Pattern Recognition (PR), Elsevier, 41:1, 2008, 299-314.
   *
   * Algorithm 5
   */

  int x, y;

  string.clear();

  // Find and add feature pixels to the end of the string.
  x = x_ref;
  y = y_ref;
  do {
    pixel_t& p = string.push_back();

    p.x_index = x;
    p.y_index = y;

    p.x = x - half_width;
    p.y = y - half_height;

    const size_t xi = static_cast<size_t>(x);
    const size_t yi = static_cast<size_t>(y);
    const size_t idx = yi * image_width + xi;
    binary_image[idx] = 0;
  } while (next(x, y, binary_image, image_width, image_height));

  pixel_t temp;
  for (size_t i = 0, j = string.size() - 1; i < j; ++i, --j) {
    temp = string[i];
    string[i] = string[j];
    string[j] = temp;
  }

  // Find and add feature pixels to the begin of the string.
  x = x_ref;
  y = y_ref;
  if (next(x, y, binary_image, image_width, image_height)) {
    do {
      pixel_t& p = string.push_back();

      p.x_index = x;
      p.y_index = y;

      p.x = x - half_width;
      p.y = y - half_height;

      const size_t xi = static_cast<size_t>(x);
      const size_t yi = static_cast<size_t>(y);
      const size_t idx = yi * image_width + xi;
      binary_image[idx] = 0;
    } while (next(x, y, binary_image, image_width, image_height));
  }
}

// Creates a list of strings of neighboring edge pixels.
void find_strings(strings_list_t& strings,
                  unsigned char* binary_image,
                  const size_t image_width,
                  const size_t image_height,
                  const size_t min_size) {
  const double half_width = 0.5 * static_cast<double>(image_width);
  const double half_height = 0.5 * static_cast<double>(image_height);

  strings.clear();

  for (size_t y = 1, y_end = image_height - 1; y != y_end; ++y) {
    for (size_t x = 1, x_end = image_width - 1; x != x_end; ++x) {
      const size_t idx = y * image_width + x;
      if (binary_image[idx]) {
        string_t& string = strings.push_back();

        linking_procedure(string, binary_image, image_width, image_height, static_cast<int>(x), static_cast<int>(y),
                          half_width, half_height);

        if (string.size() < min_size) {
          strings.pop_back();
        }
      }
    }
  }
}

// Subdivides the string of feature pixels into sets of most perceptually significant straight line segments.
inline double subdivision_procedure(clusters_list_t& clusters,
                                    const string_t& string,
                                    const size_t first_index,
                                    const size_t last_index,
                                    const double min_deviation,
                                    const size_t min_size) {
  /* D. G. Lowe
   * Three-dimensional object recognition from single two-dimensional images
   * Artificial Intelligence, Elsevier, 31, 1987, 355�395.
   *
   * Section 4.6
   */

  size_t clusters_count = clusters.size();

  const pixel_t& first = string[first_index];
  const pixel_t& last = string[last_index];

  // Compute the length of the straight line segment defined by the endpoints of the cluster.
  int x = first.x_index - last.x_index;
  int y = first.y_index - last.y_index;
  double length = sqrt(static_cast<double>((x * x) + (y * y)));

  // Find the pixels with maximum deviation from the line segment in order to subdivide the cluster.
  size_t max_pixel_index = 0;
  double deviation, max_deviation = -1.0;

  for (size_t i = first_index, count = string.size(); i != last_index; i = (i + 1) % count) {
    const pixel_t& current = string[i];

    deviation = static_cast<double>(abs(((current.x_index - first.x_index) * (first.y_index - last.y_index)) +
                                        ((current.y_index - first.y_index) * (last.x_index - first.x_index))));

    if (deviation > max_deviation) {
      max_pixel_index = i;
      max_deviation = deviation;
    }
  }
  max_deviation /= length;

  // Compute the ratio between the length of the segment and the maximum deviation.
  double ratio = length / std::max(max_deviation, min_deviation);

  // Test the number of pixels of the sub-clusters.
  if (((max_pixel_index - first_index + 1) >= min_size) && ((last_index - max_pixel_index + 1) >= min_size)) {
    double ratio1 = subdivision_procedure(clusters, string, first_index, max_pixel_index, min_deviation, min_size);
    double ratio2 = subdivision_procedure(clusters, string, max_pixel_index, last_index, min_deviation, min_size);

    // Test the quality of the sub-clusters against the quality of the current cluster.
    if ((ratio1 > ratio) || (ratio2 > ratio)) {
      return std::max(ratio1, ratio2);
    }
  }

  // Remove the sub-clusters from the list of clusters.
  clusters.resize(clusters_count);

  // Keep current cluster
  cluster_t& cluster = clusters.push_back();

  cluster.pixels = &first;
  cluster.size = (last_index - first_index) + 1;

  return ratio;
}

// Creates a list of clusters of approximately collinear feature pixels.
void find_clusters(clusters_list_t& clusters,
                   const strings_list_t& strings,
                   const double min_deviation,
                   const size_t min_size) {
  clusters.clear();

  for (size_t i = 0, end = strings.size(); i != end; ++i) {
    const string_t& string = strings[i];
    subdivision_procedure(clusters, string, 0, string.size() - 1, min_deviation, min_size);
  }
}

// This function complements the proposed voting process.
inline void vote(accumulator_t& accumulator,
                 size_t rho_start_index,
                 const size_t theta_start_index,
                 const double rho_start,
                 const double theta_start,
                 int inc_rho_index,
                 const int inc_theta_index,
                 const double sigma2_rho,
                 const double sigma2_theta,
                 const double sigma_rho_theta,
                 const double scale) {
  /* Leandro A. F. Fernandes, Manuel M. Oliveira
   * Real-time line detection through an improved Hough transform voting scheme
   * Pattern Recognition (PR), Elsevier, 41:1, 2008, 299-314.
   *
   * Algorithm 4
   */

  int** bins = accumulator.bins();

  const size_t rho_size = accumulator.width(), theta_size = accumulator.height();
  const int rho_size_int = static_cast<int>(rho_size);
  const int theta_size_int = static_cast<int>(theta_size);
  const double delta = accumulator.delta();
  const double inc_rho = delta * inc_rho_index, inc_theta = delta * inc_theta_index;

  const double sigma_rho_sigma_theta = sqrt(sigma2_rho) * sqrt(sigma2_theta);
  const double r = (sigma_rho_theta / sigma_rho_sigma_theta), two_r = 2.0 * r;
  const double a = 1.0 / (2.0 * pi * sigma_rho_sigma_theta * sqrt(1.0 - (r * r)));
  const double b = 1.0 / (2.0 * (1.0 - (r * r)));

  bool theta_voted;
  double rho, theta;
  int votes, theta_not_voted = 0;
  int rho_index, theta_index, theta_count = 0;
  int rho_start_idx = static_cast<int>(rho_start_index);
  int theta_start_idx = static_cast<int>(theta_start_index);

  // Loop for the theta coordinates of the parameter space.
  theta_index = theta_start_idx;
  theta = theta_start;
  do {
    // Test if the kernel exceeds the parameter space limits.
    if ((theta_index == 0) || (theta_index == (theta_size_int + 1))) {
      rho_start_idx = rho_size_int - rho_start_idx + 1;
      theta_index = (theta_index == 0) ? theta_size_int : 1;
      inc_rho_index = -inc_rho_index;
    }

    // Loop for the rho coordinates of the parameter space.
    theta_voted = false;

    rho_index = rho_start_idx;
    rho = rho_start;
    while (
        ((votes = static_cast<int>(
              (gauss(rho, theta, sigma2_rho, sigma2_theta, sigma_rho_sigma_theta, two_r, a, b) * scale) + 0.5)) > 0) &&
        (rho_index >= 1) && (rho_index <= rho_size_int)) {
      bins[static_cast<size_t>(theta_index)][static_cast<size_t>(rho_index)] += votes;
      theta_voted = true;

      rho_index += inc_rho_index;
      rho += inc_rho;
    }

    if (!theta_voted) {
      theta_not_voted++;
    }

    theta_index += inc_theta_index;
    theta += inc_theta;
    theta_count++;
  } while ((theta_not_voted != 2) && (theta_count < theta_size_int));
}

// Performs the proposed Hough transform voting scheme.
void voting(accumulator_t& accumulator,
            const clusters_list_t& clusters,
            const double kernel_min_height,
            const double n_sigmas) {
  /* Leandro A. F. Fernandes, Manuel M. Oliveira
   * Real-time line detection through an improved Hough transform voting scheme
   * Pattern Recognition (PR), Elsevier, 41:1, 2008, 299-314.
   *
   * Algorithm 2
   */
  static kernels_list_t kernels;
  static pkernels_list_t used_kernels;

  kernels.resize(clusters.size());
  used_kernels.resize(clusters.size());

  matrix_t M, V, S;
  point_t mean, u, v;
  double x, y, Sxx, Syy, Sxy, aux;

  static const double rad_to_deg = 180.0 / pi;
  const double delta = accumulator.delta();
  const double one_div_delta = 1.0 / delta;
  const double n_sigmas2 = n_sigmas * n_sigmas;
  const double rho_max = accumulator.rho_bounds().upper;

  for (size_t k = 0, end = clusters.size(); k != end; ++k) {
    const cluster_t& cluster = clusters[k];
    kernel_t& kernel = kernels[k];

    kernel.pcluster = &cluster;

    // Alternative reference system definition.
    mean.x = mean.y = 0.0;
    for (size_t i = 0; i != cluster.size; ++i) {
      mean.x += cluster.pixels[i].x;
      mean.y += cluster.pixels[i].y;
    }
    mean.x /= static_cast<double>(cluster.size);
    mean.y /= static_cast<double>(cluster.size);

    Sxx = Syy = Sxy = 0.0;
    for (size_t i = 0; i != cluster.size; ++i) {
      x = cluster.pixels[i].x - mean.x;
      y = cluster.pixels[i].y - mean.y;

      Sxx += (x * x);
      Syy += (y * y);
      Sxy += (x * y);
    }

    M[0] = Sxx;
    M[3] = Syy;
    M[1] = M[2] = Sxy;
    eigen(V, S, M);

    u.x = V[0];
    u.y = V[2];

    v.x = V[1];
    v.y = V[3];

    // y_v >= 0 condition verification.
    if (v.y < 0.0) {
      v.x *= -1.0;
      v.y *= -1.0;
    }

    // Normal equation parameters computation (Eq. 3).
    kernel.rho = (v.x * mean.x) + (v.y * mean.y);
    kernel.theta = acos(v.x) * rad_to_deg;

    kernel.rho_index = static_cast<size_t>(std::abs((kernel.rho + rho_max) * one_div_delta)) + 1;
    kernel.theta_index = static_cast<size_t>(std::abs(kernel.theta * one_div_delta)) + 1;

    // sigma^2_m' and sigma^2_b' computation, substituting Eq. 5 in Eq. 10.
    aux = sqrt(1.0 - (v.x * v.x));
    matrix_t nabla = {-((u.x * mean.x) + (u.y * mean.y)), 1.0, (aux != 0.0) ? ((u.x / aux) * rad_to_deg) : 0.0, 0.0};

    aux = 0.0;
    for (size_t i = 0; i != cluster.size; ++i) {
      x = (u.x * (cluster.pixels[i].x - mean.x)) + (u.y * (cluster.pixels[i].y - mean.y));
      aux += (x * x);
    }

    matrix_t lambda = {1.0 / aux, 0.0, 0.0, 1.0 / static_cast<double>(cluster.size)};

    // Uncertainty from sigma^2_m' and sigma^2_b' to sigma^2_rho,  sigma^2_theta and sigma_rho_theta.
    solve(kernel.lambda, nabla, lambda);

    if (kernel.lambda[3] == 0.0) {
      kernel.lambda[3] = 0.1;
    }

    kernel.lambda[0] *= n_sigmas2;
    kernel.lambda[3] *= n_sigmas2;

    // Compute the height of the kernel.
    kernel.height = gauss(0.0, 0.0, kernel.lambda[0], kernel.lambda[3], kernel.lambda[1]);
  }

  /* Leandro A. F. Fernandes, Manuel M. Oliveira
   * Real-time line detection through an improved Hough transform voting scheme
   * Pattern Recognition (PR), Elsevier, 41:1, 2008, 299-314.
   *
   * Algorithm 3
   */

  // Discard groups with very short kernels.
  double norm = std::numeric_limits<double>::min();

  for (size_t k = 0, end = kernels.size(); k != end; ++k) {
    kernel_t& kernel = kernels[k];

    if (norm < kernel.height) {
      norm = kernel.height;
    }
    used_kernels[k] = &kernel;
  }
  norm = 1.0 / norm;

  size_t i = 0;
  for (size_t k = 0, end = used_kernels.size(); k != end; ++k) {
    if ((used_kernels[k]->height * norm) >= kernel_min_height) {
      if (i != k) {
        kernel_t* temp = used_kernels[i];
        used_kernels[i] = used_kernels[k];
        used_kernels[k] = temp;
      }
      i++;
    }
  }
  used_kernels.resize(i);

  // Find the g_min threshold and compute the scale factor for integer votes.
  double radius, scale;
  double kernels_scale = std::numeric_limits<double>::min();

  for (size_t k = 0, end = used_kernels.size(); k != end; ++k) {
    kernel_t& kernel = *used_kernels[k];

    eigen(V, S, kernel.lambda);
    radius = sqrt(S[3]);

    scale = gauss(V[1] * radius, V[3] * radius, kernel.lambda[0], kernel.lambda[3], kernel.lambda[1]);
    scale = (scale < 1.0) ? (1.0 / scale) : 1.0;

    if (kernels_scale < scale) {
      kernels_scale = scale;
    }
  }

  // Vote for each selected kernel.
  for (size_t k = 0, end = used_kernels.size(); k != end; ++k) {
    kernel_t& kernel = *used_kernels[k];

    vote(accumulator, kernel.rho_index, kernel.theta_index, 0.0, 0.0, 1, 1, kernel.lambda[0], kernel.lambda[3],
         kernel.lambda[1], kernels_scale);
    vote(accumulator, kernel.rho_index, kernel.theta_index - 1, 0.0, -delta, 1, -1, kernel.lambda[0], kernel.lambda[3],
         kernel.lambda[1], kernels_scale);
    vote(accumulator, kernel.rho_index - 1, kernel.theta_index, -delta, 0.0, -1, 1, kernel.lambda[0], kernel.lambda[3],
         kernel.lambda[1], kernels_scale);
    vote(accumulator, kernel.rho_index - 1, kernel.theta_index - 1, -delta, -delta, -1, -1, kernel.lambda[0],
         kernel.lambda[3], kernel.lambda[1], kernels_scale);
  }
}

// The coordinates of a bin of the accumulator.
struct bin_t {
  size_t rho_index;    // [1,rho_size] range.
  size_t theta_index;  // [1,theta_size] range.

  int votes;
};

// Specifies a list of accumulator bins.
typedef list<bin_t, 1000> bins_list_t;

// An auxiliar data structure that identifies which accumulator bin was visited by the peak detection procedure.
class visited_map_t {
 private:
  // The map of flags ([1,theta_size][1,rho_size] range).
  bool** m_map;

  // Specifies the size of allocated storage for the map (rho dimention).
  size_t m_rho_capacity;

  // Specifies the size of allocated storage for the map (theta dimention).
  size_t m_theta_capacity;

 public:
  // Initializes the map.
  inline void init(const size_t accumulator_width, const size_t accumulator_height) {
    if ((m_rho_capacity < (accumulator_width + 2)) || (m_theta_capacity < (accumulator_height + 2))) {
      m_rho_capacity = accumulator_width + 2;
      m_theta_capacity = accumulator_height + 2;

      m_map = static_cast<bool**>(realloc_2d(m_map, m_theta_capacity, m_rho_capacity, sizeof(bool)));
    }

    memset_2d(m_map, 0, m_theta_capacity, m_rho_capacity, sizeof(bool));
  }

  // Sets a given accumulator bin as visited.
  inline void set_visited(const size_t rho_index, size_t theta_index) { m_map[theta_index][rho_index] = true; }

  // Class constructor.
  visited_map_t() : m_map(0), m_rho_capacity(0), m_theta_capacity(0) {}

  visited_map_t(const visited_map_t&) = delete;
  visited_map_t& operator=(const visited_map_t&) = delete;
  visited_map_t(visited_map_t&&) = delete;
  visited_map_t& operator=(visited_map_t&&) =
      delete;  // If sharing is needed later, migrate to std::vector to manage lifetime safely.

  // Class destructor()
  ~visited_map_t() { free(m_map); }

  // Returns whether a neighbour bin was visited already.
  inline bool visited_neighbour(const size_t rho_index, const size_t theta_index) const {
    return m_map[theta_index - 1][rho_index - 1] || m_map[theta_index - 1][rho_index] ||
           m_map[theta_index - 1][rho_index + 1] || m_map[theta_index][rho_index - 1] ||
           m_map[theta_index][rho_index + 1] || m_map[theta_index + 1][rho_index - 1] ||
           m_map[theta_index + 1][rho_index] || m_map[theta_index + 1][rho_index + 1];
  }
};

inline bool compare_bins(const bin_t& bin1, const bin_t& bin2) { return bin1.votes > bin2.votes; }

// Computes the convolution of the given cell with a (discrete) 3x3 Gaussian kernel.
inline int convolution(const int** bins, const int rho_index, const int theta_index) {
  return bins[theta_index - 1][rho_index - 1] + bins[theta_index - 1][rho_index + 1] +
         bins[theta_index + 1][rho_index - 1] + bins[theta_index + 1][rho_index + 1] +
         bins[theta_index - 1][rho_index] + bins[theta_index - 1][rho_index] + bins[theta_index][rho_index - 1] +
         bins[theta_index][rho_index - 1] + bins[theta_index][rho_index + 1] + bins[theta_index][rho_index + 1] +
         bins[theta_index + 1][rho_index] + bins[theta_index + 1][rho_index] + bins[theta_index][rho_index] +
         bins[theta_index][rho_index] + bins[theta_index][rho_index] + bins[theta_index][rho_index];
}

// Identify the peaks of votes (most significant straight lines) in the accmulator.
void peak_detection(lines_list_t& lines, const accumulator_t& accumulator) {
  /* Leandro A. F. Fernandes, Manuel M. Oliveira
   * Real-time line detection through an improved Hough transform voting scheme
   * Pattern Recognition (PR), Elsevier, 41:1, 2008, 299-314.
   *
   * Section 3.4
   */

  const int** bins = accumulator.bins();
  const double* rho = accumulator.rho();
  const double* theta = accumulator.theta();

  // Create a list with all cells that receive at least one vote.
  static bins_list_t used_bins;

  size_t used_bins_count = 0;
  for (size_t theta_index = 1, theta_end = accumulator.height() + 1; theta_index != theta_end; ++theta_index) {
    for (size_t rho_index = 1, rho_end = accumulator.width() + 1; rho_index != rho_end; ++rho_index) {
      if (bins[theta_index][rho_index]) {
        used_bins_count++;
      }
    }
  }
  used_bins.resize(used_bins_count);

  for (size_t theta_index = 1, i = 0, theta_end = accumulator.height() + 1; theta_index != theta_end; ++theta_index) {
    for (size_t rho_index = 1, rho_end = accumulator.width() + 1; rho_index != rho_end; ++rho_index) {
      if (bins[theta_index][rho_index]) {
        bin_t& bin = used_bins[i];

        bin.rho_index = rho_index;
        bin.theta_index = theta_index;
        bin.votes = convolution(bins, static_cast<int>(rho_index),
                                static_cast<int>(theta_index));  // Convolution of the cells with a 3x3 Gaussian kernel

        i++;
      }
    }
  }

  // Sort the list in descending order according to the result of the convolution.
  bin_t* const used_bins_begin = used_bins.items();
  std::sort(used_bins_begin, used_bins_begin + static_cast<std::ptrdiff_t>(used_bins_count), compare_bins);

  // Use a sweep plane that visits each cell of the list.
  static visited_map_t visited;
  visited.init(accumulator.width(), accumulator.height());

  lines.clear();
  lines.reserve(used_bins_count);

  for (size_t i = 0; i != used_bins_count; ++i) {
    bin_t& bin = used_bins[i];

    if (!visited.visited_neighbour(bin.rho_index, bin.theta_index)) {
      line_t& line = lines.push_back();

      line.rho = rho[bin.rho_index];
      line.theta = theta[bin.theta_index];
    }
    visited.set_visited(bin.rho_index, bin.theta_index);
  }
}
