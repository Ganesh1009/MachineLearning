#ifndef BRIDGE_NODE_DONGLE_MATRIX_H
#define BRIDGE_NODE_DONGLE_MATRIX_H

#define INP_MAT_ROW_SIZE 1
#define INP_MAT_COL_SIZE 3
#define B_MAT_ROW_SIZE 1
#define B_MAT_COL_SIZE 3
#define KINV_MAT_ROW_SIZE 3
#define KINV_MAT_COL_SIZE 3

#include <stdbool.h>
#include "bsn_imu_data.h"

typedef struct {
  float **data;
  int cols,rows;
}Matrix;

Matrix* getMatrix(int, int);
Matrix* getMemory(void);
void freeMatrix(Matrix *);

void mat_add(Matrix *, Matrix *);
void mat_mul(Matrix *, Matrix *, Matrix *);
void mat_sub(Matrix *, Matrix *);
void mat_divide(Matrix *, float *);
void transpose(Matrix *, Matrix *);
void inverse(Matrix *, Matrix *);

float norm(Matrix *);
void normalize(Matrix *, float);
void mat_copy(Matrix *, Matrix *);

void vector_cross_product(Matrix *, Matrix *, Matrix *);
void vector_dot_product(Matrix *, Matrix *);
bool val_mat_dim(int, int);

void quatFromRotation(Matrix *, imu_orientation_t *, float);
void quatMultiplication(imu_orientation_t *, imu_orientation_t *, imu_orientation_t *);
void quatCopy(imu_orientation_t *, imu_orientation_t *);
void mat_to_quat_copy(Matrix *, imu_orientation_t *);
void quat_to_mat_copy(imu_orientation_t *, Matrix *);
void quat_inverse(imu_orientation_t *);

#endif /*BRIDGE_NODE_DONGLE_MATRIX_H*/
