#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include "matrix.h"

Matrix* getMemory() {
  return (Matrix *) malloc(sizeof(Matrix));
}

void freeMatrix(Matrix *matrix){
  for(int i=0;i<matrix->rows;i++)
    free(matrix->data[i]);

  free(matrix);
}

Matrix* getMatrix(int rows, int cols){
  Matrix *temp = getMemory();
  temp->rows = rows;
  temp->cols = cols;
  temp->data = (float **) malloc(sizeof(float *) * temp->rows);
  for (size_t i = 0; i < rows; i++)
    temp->data[i] = (float *) malloc(sizeof(float) * temp->cols);

  return temp;
}

void mat_mul(Matrix *array1, Matrix *array2, Matrix *result) {
    float temp_sum;
    if(val_mat_dim(array1->cols,array2->rows)){
      for(int row=0; row<array1->rows; row++) {
        for(int col=0; col<array2->cols; col++) {
          for(int k=0; k<array2->rows; k++) {
            temp_sum += array1->data[row][k] * array2->data[k][col];
            // nrf_cli_fprintf(&m_cli_cdc_acm, 2,"S=%d,B=%d,R=%d",
            //   (int)array1->data[row][k],(int)array2->data[k][col],(int)temp_sum);
          }
          result->data[row][col] = temp_sum;
          temp_sum = 0;
          // nrf_cli_fprintf(&m_cli_cdc_acm, 2,"Result=%d",(int)resultant_matrix->data[row][col]);
        }
      }
    }
}

void mat_sub(Matrix *array1, Matrix *array2) {
    if(val_mat_dim(array1->cols, array2->cols)) {
      for(int row=0; row<array1->rows; row++)
      for(int col=0; col< array1->cols; col++)
        array1->data[row][col] -= array2->data[row][col];
    }
}

float norm(Matrix *matrix){

  float sum = 0.0;
  for(int row=0; row<matrix->rows; row++)
  for(int col=0; col<matrix->cols; col++)
    sum += pow(*(*(matrix->data+row)+col),2);

  return sqrt(sum);
}

void normalize(Matrix *matrix, float normalized_factor) {

  if(normalized_factor == 0.0)
    normalized_factor = norm(matrix);

  if(normalized_factor != 0.0){
    for(int row=0; row<matrix->rows; row++)
    for(int col=0; col<matrix->cols; col++)
      matrix->data[row][col] /= normalized_factor;
  }
}

void quatFromRotation(Matrix *matrix, imu_orientation_t *quaternion, float angle){

  float half_angle = angle / 2.;
  float sin_half_angle = sin(half_angle);
  float cos_half_angle = cos (half_angle);

  quaternion->w = cos_half_angle;
  quaternion->x = (sin_half_angle)*(matrix->data[0][0]);
  quaternion->y = (sin_half_angle)*(matrix->data[1][0]);
  quaternion->z = (sin_half_angle)*(matrix->data[2][0]);

}

void quatMultiplication(imu_orientation_t *predicted_state,
  imu_orientation_t *previous_state, imu_orientation_t *quaternion) {

    float aw = previous_state->w;
    float ax = previous_state->x;
    float ay = previous_state->y;
    float az = previous_state->z;

    float bw = quaternion->w;
    float bx = quaternion->x;
    float by = quaternion->y;
    float bz = quaternion->z;

    predicted_state->w = aw*bw - ax*bx - ay*by - az*bz;
    predicted_state->x = ay*bz - az*by + aw*bx + ax*bw;
    predicted_state->y = az*bx - ax*bz + aw*by + ay*bw;
    predicted_state->z = ax*by - ay*bx + aw*bz + az*bw;
}

void transpose(Matrix *matrix, Matrix *resultant_matrix) {

  for (int col=0; col<matrix->cols; col++) {

      for(int row=0; row<matrix->rows; row++) {

          *(*(resultant_matrix->data+col)+row) = *(*(matrix->data+row)+col);
      }
  }
}

void mat_add(Matrix *matrix1, Matrix *matrix2) {

  for(int row=0; row<matrix1->rows; row++) {

    for(int col=0; col<matrix1->cols; col++) {

        *(*(matrix1->data + row) + col)  += *(*(matrix2->data + row) + col);
    }
  }
}

void mat_copy(Matrix *source, Matrix *dest){

  for(int row=0; row<source->rows; row++)
  for(int col=0; col<source->cols; col++)
    *(*(dest->data + row) + col)  = *(*(source->data + row) + col);
}

void quatCopy(imu_orientation_t *source, imu_orientation_t *dest){
  dest->x = source->x;
  dest->y = source->y;
  dest->z = source->z;
  dest->w = source->w;
}

void mat_to_quat_copy(Matrix *source, imu_orientation_t *dest){
  dest->w = source->data[0][0];
  dest->x = source->data[1][0];
  dest->y = source->data[2][0];
  dest->z = source->data[3][0];
}

void quat_to_mat_copy(imu_orientation_t *source, Matrix *dest){
  dest->data[0][0] = source->w;
  dest->data[1][0] = source->x;
  dest->data[2][0] = source->y;
  dest->data[3][0] = source->z;
}

void inverse(Matrix *array1, Matrix *inverseMatrix) {

    float determinant = 0.0;

    for (int row=0; row<array1->rows; row++) {
        determinant += *(*(array1->data + 0)+row) *
          ( ((*(*(array1->data+1)+((row+1)%3))) * (*(*(array1->data+2)+((row+2)%3))))
          - ((*(*(array1->data+1)+((row+2)%3))) * (*(*(array1->data+2)+((row+1)%3)))));

    }

    if(determinant != 0.0) {
      for(int row=0; row<array1->rows; row++)
      for(int col=0; col<array1->cols; col++)
        *(*(inverseMatrix->data+row)+col) = (((*(*(array1->data+((col+1)%3))+((row+1)%3))) *
          (*(*(array1->data+((col +2)%3)) + ((row+2)%3))) ) - ( (*(*(array1->data+((col+1)%3))+
          ((row+2)%3))) * (*(*(array1->data+((col+2)%3))+((row+1)%3)))))/determinant;
    }

}

void vector_cross_product(Matrix *mat1, Matrix *mat2, Matrix *resultant_matrix) {
  resultant_matrix->data[0][0] = ((mat1->data[1][0] * mat2->data[2][0]) -
    (mat1->data[2][0] * mat2->data[1][0]));

  resultant_matrix->data[1][0] = ((mat1->data[2][0] * mat2->data[0][0]) -
    (mat1->data[0][0] * mat2->data[2][0]));

  resultant_matrix->data[2][0] = ((mat1->data[0][0] * mat2->data[1][0]) -
    (mat1->data[1][0] * mat2->data[0][0]));
}

float quat_norm(imu_orientation_t *quaternion) {
  return sqrt(pow(quaternion->w,2)+pow(quaternion->x,2)+pow(quaternion->y,2)+
    pow(quaternion->z,2));
}

void quat_inverse(imu_orientation_t *quaternion) {
  float squared_norm = pow ( quat_norm(quaternion), 2);
  quaternion->w /= squared_norm;
  quaternion->x /= squared_norm;
  quaternion->y /= squared_norm;
  quaternion->z /= squared_norm;
}

// void quatAddition(imu_orientation_t *quat1, imu_orientation_t *quat2) {
//   quat1->x += quat2->x;
//   quat1->y += quat2->y;
//   quat1->z += quat2->z;
//   quat1->w += quat2->w;
// }





// void mat_divide(Matrix *array1, float scalar) {
//
//     resultant_matrix = (float **) malloc(sizeof(float *) * cols);
//
//     for (int row=0; row<rows; row++) {
//
//         *(resultant_matrix + row) = (float *) malloc(sizeof(float) * rows);
//
//         for(int col=0; col<cols; col++) {
//
//             *(*(resultant_matrix+row)+col) = *(*(mat1 + row)+col) / scalar;
//         }
//     }
// }
//
// void vector_dot_product(Matrix *array1, Matrix *array1) {
//
//     resultant_matrix = (float **)(malloc(sizeof(float*)));
//     *(resultant_matrix) = (float *)(malloc(sizeof(float)*cols));
//
//     for(int i=0; i<3; i++) {
//
//         *(*(resultant_matrix)+i) = (*(*(vector1)+i)) * (*(*(vector2)+i));
//     }
// }

bool val_mat_dim(int dim1, int dim2) {
    return (dim1 == dim2)?true:false;
}
