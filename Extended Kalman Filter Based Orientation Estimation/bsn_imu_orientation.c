
/*
 * bsn_imu_orientation.c
 *
 *  Created on: 5 Dec 2018
 *      Author: musahl
 */

#define TIMESTAMP 100
#define INIT_SAMPLES 100
#define QUAT_DIM 4
#define NORM_DIM 3
#define GRAVITY_CONSTANT 9.8

#include "bsn_imu_data.h"
#include "bsn_imu_orientation.h"
#include "matrix.h"
#include<stdlib.h>
#include<math.h>

typedef struct bsn_imu_orientation_store {
  //whatever else you need for the ekf goes here
  float counter,temp_mag_sum, temp_acc_sum,gravity_avg,mag_avg,gyr_l2_norm,
    acc_l2_norm,mag_l2_norm,variance,difference,observed_data_matrix_l2_norm;
  float sinp,siny,sinr,cosr,cosy,roll_new,pitch_new,yaw_old,c1,c2,c3,s1,s2,s3,
    observed_mag_l2_norm;
  float q1,q2,q3,q4;
  imu_orientation_t quaternion,predicted_state,previous_state,temp_quat;
  Matrix *tempGyro, *tempAcc, *tempMag,*updated_pred_state,*temp_quat_mat1, *temp_quat_mat2;
  Matrix *pred_state_cov_mat, *prev_state_cov_mat, *adaption_mat_F;
  Matrix *adaption_mat_F_trans, *noise_cov_mat,*updated_state_cov_mat;
  Matrix *lnr_appx_pred_stat_mat, *observed_data_matrix, *kalman_gain;
  Matrix *R, *pred_cov_transformation_mat,*pred_cov_transformation_trans;
  Matrix *pred_cov_plus_R_inverse,*pred_cov_plus_R_mat,*intrim_pred_cov_mat;
  Matrix *pred_state,*kg_transformation_mat,*kg_transformation_idn, *mzl;
  Matrix *intrimResult,*x_vector,*y_vector,*z_vector,*mag_z_mat,*rotation_mat;
} bsn_imu_orientation_store_t;

bsn_imu_orientation_store_t *t;

void bsn_imu_orientation_initialize() {
  // init your local variables etc here

  t->counter = 1.0;
  t->temp_mag_sum = 0.0;
  t->temp_acc_sum = 0.0;
  t->gravity_avg = 0.0;
  t->mag_avg = 0.0;
  t = (bsn_imu_orientation_store_t *)malloc(sizeof(bsn_imu_orientation_store_t));

  Matrix *temp = getMatrix(NORM_DIM,1);
  t->intrimResult = getMatrix(QUAT_DIM,QUAT_DIM);
  t->temp_quat_mat1 = getMatrix(QUAT_DIM,1);
  t->temp_quat_mat2 = getMatrix(QUAT_DIM,1);
  for(int i=0;i<t->temp_quat_mat1->rows;i++){
    t->temp_quat_mat1->data[i][0] = (i == 0)?1:0;
  }

  t->tempGyro = getMatrix(NORM_DIM,1);
  t->tempAcc = getMatrix(NORM_DIM,1);
  t->tempMag = getMatrix(NORM_DIM,1);
  t->R = getMatrix(NORM_DIM,NORM_DIM);
  t->rotation_mat = getMatrix(NORM_DIM,NORM_DIM);
  t->mag_z_mat = getMatrix(NORM_DIM,1);
  t->mzl = getMatrix(NORM_DIM,1);
  for (size_t i = 0; i < t->mag_z_mat->rows; i++)
  for (size_t j = 0; j < t->mag_z_mat->cols; j++)
    t->mag_z_mat->data[i][j] = (i == j)?1:0;

  for(int i=0;i<temp->rows;i++){
    temp->data[i][0] = (i == 2)?1:0;
  }

  quatFromRotation(temp,&(t->previous_state),0.0);

  t->noise_cov_mat = getMatrix(QUAT_DIM,QUAT_DIM);
  t->adaption_mat_F = getMatrix(QUAT_DIM,QUAT_DIM);
  t->prev_state_cov_mat = getMatrix(QUAT_DIM,QUAT_DIM);
  t->adaption_mat_F_trans = getMatrix(QUAT_DIM,QUAT_DIM);
  t->pred_state_cov_mat = getMatrix(QUAT_DIM,QUAT_DIM);

  t->pred_cov_transformation_mat = getMatrix(NORM_DIM,QUAT_DIM);
  t->pred_cov_transformation_trans = getMatrix(QUAT_DIM,NORM_DIM);

  t->pred_cov_plus_R_inverse = getMatrix(NORM_DIM,NORM_DIM);
  t->pred_cov_plus_R_mat = getMatrix(NORM_DIM,NORM_DIM);
  t->intrim_pred_cov_mat = getMatrix(QUAT_DIM,NORM_DIM);

  for(int i=0; i<t->adaption_mat_F->rows; i++){
    for (size_t j = 0; j < t->adaption_mat_F->cols; j++) {
      t->noise_cov_mat->data[i][j] = (i == j)?0.01:0;
      t->adaption_mat_F->data[i][j] = (i == j)?0.1:0;
      t->prev_state_cov_mat->data[i][j] = (i == j)?1:0;
    }
  }

  t->lnr_appx_pred_stat_mat = getMatrix(NORM_DIM,1);
  t->x_vector = getMatrix(NORM_DIM,1);
  t->y_vector = getMatrix(NORM_DIM,1);
  t->z_vector = getMatrix(NORM_DIM,1);
  t->observed_data_matrix = getMatrix(NORM_DIM,1);
  t->kalman_gain = getMatrix(QUAT_DIM,NORM_DIM);
  t->updated_pred_state = getMatrix(QUAT_DIM,1);
  t->pred_state = getMatrix(QUAT_DIM,1);
  t->kg_transformation_mat = getMatrix(QUAT_DIM,QUAT_DIM);
  t->kg_transformation_idn = getMatrix(QUAT_DIM,QUAT_DIM);
  t->updated_state_cov_mat = getMatrix(QUAT_DIM,QUAT_DIM);


}


void bsn_imu_orientation_free() {
  freeMatrix(t->adaption_mat_F_trans);
  freeMatrix(t->adaption_mat_F);
  freeMatrix(t->noise_cov_mat);
  free(t);
}

void bsn_imu_orientation_add_sample(imu_calibrated_t* calibrated_imu_data) {
  // the magic happens here, this updates the ekf
  if(t->counter <= INIT_SAMPLES+1)
    t->counter++;

  for(int i=0; i<t->tempGyro->rows; i++)
    t->tempGyro->data[i][0] = calibrated_imu_data->gyr[i];

  for(int i=0; i<t->tempAcc->rows; i++)
    t->tempAcc->data[i][0] = calibrated_imu_data->acc[i];

  for(int i=0; i<t->tempMag->rows; i++)
    t->tempMag->data[i][0] = calibrated_imu_data->mag[i];

  t->gyr_l2_norm = norm(t->tempGyro);

  t->acc_l2_norm = norm(t->tempAcc);
  normalize(t->tempAcc,t->acc_l2_norm);

  t->mag_l2_norm = norm(t->tempMag);
  normalize(t->tempMag,t->mag_l2_norm);

  if (t->counter < INIT_SAMPLES-70) {

    t->temp_mag_sum += t->mag_l2_norm;
    t->mag_avg = t->temp_mag_sum/t->counter;

    t->temp_acc_sum += t->acc_l2_norm;
    t->gravity_avg = t->temp_acc_sum/t->counter;

  }

  const bool PREDECTION = true;
  const bool MEASUREMENT_ACC = true;
  const bool MEASUREMENT_MAG = true;

  if(PREDECTION) {

    normalize(t->tempGyro,t->gyr_l2_norm);
    t->gyr_l2_norm /= TIMESTAMP;
    // x_k_p = A*x_k_1
    quatFromRotation(t->tempGyro, &(t->quaternion), t->gyr_l2_norm);
    //Parameter definition of quatMultiplication is : quatMultiplication(RESULT,MATRIX_1,MATRIX_2)
    quatMultiplication(&(t->predicted_state), &(t->previous_state),
      &(t->quaternion));

    //p_k_p = (A*p_k_1*ATranspose) + q_k
    mat_mul(t->adaption_mat_F,t->prev_state_cov_mat,t->intrimResult);
    transpose(t->adaption_mat_F, t->adaption_mat_F_trans);
    mat_mul(t->intrimResult,t->adaption_mat_F_trans,t->pred_state_cov_mat);
    mat_add(t->pred_state_cov_mat,t->noise_cov_mat);
    // nrf_cli_fprintf(&m_cli_cdc_acm, 2,"Predicted state matrix:");
    // nrf_cli_fprintf(&m_cli_cdc_acm, 2,"%d,%d,%d,%d\n",(int)((t->predicted_state.w)*1000),
    //   (int)((t->predicted_state.x)*1000),(int)((t->predicted_state.y)*1000),
    //   (int)((t->predicted_state.z)*1000));
    // nrf_cli_fprintf(&m_cli_cdc_acm, 2,"Predicted state cov matrix:");
    // printMatrix(t->pred_state_cov_mat);
  }
  else{
    quatCopy(&(t->previous_state),&(t->predicted_state));
    mat_copy(t->prev_state_cov_mat,t->pred_state_cov_mat);
    mat_add(t->pred_state_cov_mat,t->noise_cov_mat);
  }

  if (MEASUREMENT_ACC) {
    normalize(t->tempAcc,t->acc_l2_norm);
    // Pre steps for UPDATING THE PREDICTED STATE (Y - h(x_k_p)) = Z
    t->lnr_appx_pred_stat_mat->data[0][0] =
     2*((t->predicted_state.x * t->predicted_state.z) -
        (t->predicted_state.w * t->predicted_state.y));

    t->lnr_appx_pred_stat_mat->data[1][0] =
     2*((t->predicted_state.y * t->predicted_state.z) +
        (t->predicted_state.w * t->predicted_state.x));

    t->lnr_appx_pred_stat_mat->data[2][0] =
     (pow(t->predicted_state.w,2) - pow(t->predicted_state.x,2) -
      pow(t->predicted_state.y,2) + pow(t->predicted_state.z,2));

    normalize(t->lnr_appx_pred_stat_mat,norm(t->lnr_appx_pred_stat_mat));
    mat_copy(t->tempAcc, t->observed_data_matrix);
    mat_sub(t->observed_data_matrix,t->lnr_appx_pred_stat_mat);
    // Pre steps for UPDATING THE PREDICTED STATE done=======================

    // KALMAN GAIN kg = (p_k_p * Htranspose) / ((H * p_k_p * Htrasnpose) + R)
    t->variance = 1.0;
    t->difference = fabs(t->acc_l2_norm - t->gravity_avg);

    if (t->counter <= INIT_SAMPLES) {
        t->variance += fmax((t->difference - 0.02),0.0) * 2;
    } else {
      t->observed_data_matrix_l2_norm = fabs(norm(t->observed_data_matrix));
      t->variance += fmax(t->observed_data_matrix_l2_norm,0.0) * 10;
    }


    for (size_t i = 0; i < t->R->rows; i++)
    for (size_t j = 0; j < t->R->cols; j++) {
      t->R->data[i][j] = (i == j)?t->variance:0;
    }

    t->pred_cov_transformation_mat->data[0][0] = -2*(t->predicted_state.y);
    t->pred_cov_transformation_mat->data[0][1] = 2*(t->predicted_state.z);
    t->pred_cov_transformation_mat->data[0][2] = -2*(t->predicted_state.w);
    t->pred_cov_transformation_mat->data[0][3] = 2*(t->predicted_state.x);

    t->pred_cov_transformation_mat->data[1][0] = 2*(t->predicted_state.x);
    t->pred_cov_transformation_mat->data[1][1] = 2*(t->predicted_state.w);
    t->pred_cov_transformation_mat->data[1][2] = 2*(t->predicted_state.z);
    t->pred_cov_transformation_mat->data[1][3] = 2*(t->predicted_state.y);

    t->pred_cov_transformation_mat->data[2][0] = 2*(t->predicted_state.w);
    t->pred_cov_transformation_mat->data[2][1] = -2*(t->predicted_state.x);
    t->pred_cov_transformation_mat->data[2][2] = -2*(t->predicted_state.y);
    t->pred_cov_transformation_mat->data[2][3] = 2*(t->predicted_state.z);
    transpose(t->pred_cov_transformation_mat,t->pred_cov_transformation_trans);

    // nrf_cli_fprintf(&m_cli_cdc_acm, 2,"\nH matrix:");
    // printMatrix(t->pred_cov_transformation_mat);
    //
    // nrf_cli_fprintf(&m_cli_cdc_acm, 2,"\nHtranspose matrix:");
    // printMatrix(t->pred_cov_transformation_trans);
    mat_mul(t->pred_state_cov_mat,t->pred_cov_transformation_trans,
      t->intrim_pred_cov_mat); //(4x4) * (4x3) = (4x3)
    // nrf_cli_fprintf(&m_cli_cdc_acm, 2,"\nINTER MAT:");
    // printMatrix(t->intrim_pred_cov_mat);
    mat_mul(t->pred_cov_transformation_mat,t->intrim_pred_cov_mat,
      t->pred_cov_plus_R_mat);  //(3x4) * (4x3) = (3x3)
    mat_add(t->pred_cov_plus_R_mat,t->R);
    inverse(t->pred_cov_plus_R_mat,t->pred_cov_plus_R_inverse);
    // nrf_cli_fprintf(&m_cli_cdc_acm, 2,"\nInverse matrix:");
    // printMatrix(t->pred_cov_plus_R_inverse);
    // nrf_cli_fprintf(&m_cli_cdc_acm, 2,"\nOf:");
    // printMatrix(t->pred_cov_plus_R_mat);
    mat_mul(t->intrim_pred_cov_mat,t->pred_cov_plus_R_inverse,
      t->kalman_gain);
    // nrf_cli_fprintf(&m_cli_cdc_acm, 2,"\nKALMAN GAIN:");
    // printMatrix(t->kalman_gain);
    // ==============KALMAN GAIN COMPUTATION DONE=========================

    // x_k = x_k_p + K*(Z) UPDATING THE PREDICTED STATE from pre steps========
    // nrf_cli_fprintf(&m_cli_cdc_acm, 2,"\nLINEAR APPROX:");
    // printMatrix(t->lnr_appx_pred_stat_mat);
    mat_mul(t->kalman_gain,t->observed_data_matrix,t->updated_pred_state);
    quat_to_mat_copy(&(t->predicted_state),t->updated_pred_state);

    mat_add(t->updated_pred_state,t->pred_state);
    normalize(t->updated_pred_state,0.0);
    // nrf_cli_fprintf(&m_cli_cdc_acm, 2,"\nUPDATED PRED STATE:");
    // printMatrix(t->updated_pred_state);
    // ==========================UPDATING PREDICTED STATE DONE==============

    // p_k = (I - K*H)*p_k_p   UPDATING THE PREDICTED_STATE_COVARIANCE_MATRIX
    mat_mul(t->kalman_gain,t->pred_cov_transformation_mat,
      t->kg_transformation_mat);

    for (size_t i = 0; i < t->kg_transformation_idn->rows; i++)
    for (size_t j = 0; j < t->kg_transformation_idn->cols; j++)
      t->kg_transformation_idn->data[i][j] = (i==j)?1:0;

    mat_sub(t->kg_transformation_idn,t->kg_transformation_mat);
    mat_mul(t->kg_transformation_idn,t->pred_state_cov_mat,
      t->updated_state_cov_mat);
    //===========PREDICTED_STATE_COVARIANCE_MATRIX UPDATE DONE===============
    // nrf_cli_fprintf(&m_cli_cdc_acm, 2,"\nUPDATED COV STATE:");
    // printMatrix(t->updated_state_cov_mat);
    mat_to_quat_copy(t->updated_pred_state,&(t->predicted_state));
    mat_copy(t->updated_state_cov_mat,t->pred_state_cov_mat);
  }

  if (MEASUREMENT_MAG) {
    normalize(t->tempGyro,t->mag_l2_norm);

    // Pre steps for UPDATING THE PREDICTED STATE (Y - h(x_k_p))=Z ===========

    t->z_vector->data[0][0] =
     2*((t->predicted_state.x * t->predicted_state.z) -
        (t->predicted_state.w * t->predicted_state.y));
    t->z_vector->data[1][0] =
     2*((t->predicted_state.y * t->predicted_state.z) +
        (t->predicted_state.w * t->predicted_state.x));
    t->z_vector->data[2][0] =
     (pow(t->predicted_state.w,2) - pow(t->predicted_state.x,2) -
      pow(t->predicted_state.y,2) + pow(t->predicted_state.z,2));
    normalize(t->z_vector,0.0);

    vector_cross_product(t->z_vector,t->tempGyro,t->y_vector);
    normalize(t->y_vector,0.0);

    vector_cross_product(t->y_vector,t->z_vector,t->x_vector);
    normalize(t->x_vector,0.0);

    t->rotation_mat->data[0][0] =
     (pow(t->predicted_state.w,2) + pow(t->predicted_state.x,2) -
      pow(t->predicted_state.y,2) - pow(t->predicted_state.z,2));
    t->rotation_mat->data[0][1] =
     (2*(t->predicted_state.x * t->predicted_state.y) +
      2*(t->predicted_state.w * t->predicted_state.z));
    t->rotation_mat->data[0][2] =
     (2*(t->predicted_state.x * t->predicted_state.z) +
      2*(t->predicted_state.w * t->predicted_state.y));

    t->rotation_mat->data[1][0] =
     (2*(t->predicted_state.x * t->predicted_state.y) -
      2*(t->predicted_state.w * t->predicted_state.z));

    t->rotation_mat->data[1][1] =
     (pow(t->predicted_state.w,2) - pow(t->predicted_state.x,2) +
      pow(t->predicted_state.y,2) - pow(t->predicted_state.z,2));

    t->rotation_mat->data[1][2] =
     (2*(t->predicted_state.y * t->predicted_state.z) +
      2*(t->predicted_state.w * t->predicted_state.x));

    t->rotation_mat->data[2][0] =
     (2*(t->predicted_state.x * t->predicted_state.z) +
      2*(t->predicted_state.w * t->predicted_state.y));
    t->rotation_mat->data[2][1] =
     (2*(t->predicted_state.y * t->predicted_state.z) -
      2*(t->predicted_state.w * t->predicted_state.x));
    t->rotation_mat->data[2][2] =
     (pow(t->predicted_state.w,2) -pow(t->predicted_state.x,2) -
      pow(t->predicted_state.y,2)+pow(t->predicted_state.z,2));

    mat_mul(t->rotation_mat,t->mag_z_mat,t->lnr_appx_pred_stat_mat);
    normalize(t->lnr_appx_pred_stat_mat,0.0);
    mat_copy(t->x_vector,t->observed_data_matrix);
    mat_sub(t->observed_data_matrix,t->lnr_appx_pred_stat_mat);
    //===========Pre steps for UPDATING THE PREDICTED STATE done===============

    // KALMAN GAIN kg = (p_k_p * Htranspose) / ((H * p_k_p * Htrasnpose) + R)
    t->pred_cov_transformation_mat->data[0][0] =
      (2*t->mag_z_mat->data[0][0]*(t->predicted_state.w) +
       2*t->mag_z_mat->data[1][0]*(t->predicted_state.z) -
       2*t->mag_z_mat->data[2][0]*(t->predicted_state.y));
    t->pred_cov_transformation_mat->data[0][1] =
      (2*t->mag_z_mat->data[0][0]*(t->predicted_state.x) +
       2*t->mag_z_mat->data[1][0]*(t->predicted_state.y) +
       2*t->mag_z_mat->data[2][0]*(t->predicted_state.z));
    t->pred_cov_transformation_mat->data[0][2] =
      (-2*t->mag_z_mat->data[0][0]*(t->predicted_state.y) +
        2*t->mag_z_mat->data[1][0]*(t->predicted_state.x) -
        2*t->mag_z_mat->data[2][0]*(t->predicted_state.w));
    t->pred_cov_transformation_mat->data[0][3] =
      (-2*t->mag_z_mat->data[0][0]*(t->predicted_state.z) +
        2*t->mag_z_mat->data[1][0]*(t->predicted_state.w) +
        2*t->mag_z_mat->data[2][0]*(t->predicted_state.x));

    t->pred_cov_transformation_mat->data[1][0] =
      (-2*t->mag_z_mat->data[0][0]*(t->predicted_state.z) +
        2*t->mag_z_mat->data[1][0]*(t->predicted_state.w) +
        2*t->mag_z_mat->data[2][0]*(t->predicted_state.x));
    t->pred_cov_transformation_mat->data[1][1] =
      (2*t->mag_z_mat->data[0][0]*(t->predicted_state.y) -
       2*t->mag_z_mat->data[1][0]*(t->predicted_state.x) +
       2*t->mag_z_mat->data[2][0]*(t->predicted_state.w));
    t->pred_cov_transformation_mat->data[1][2] =
      (2*t->mag_z_mat->data[0][0]*(t->predicted_state.x) +
       2*t->mag_z_mat->data[1][0]*(t->predicted_state.y) +
       2*t->mag_z_mat->data[2][0]*(t->predicted_state.z));
    t->pred_cov_transformation_mat->data[1][3] =
      (-2*t->mag_z_mat->data[0][0]*(t->predicted_state.w) -
       2*t->mag_z_mat->data[1][0]*(t->predicted_state.z) -
       2*t->mag_z_mat->data[2][0]*(t->predicted_state.y));

    t->pred_cov_transformation_mat->data[2][0] =
      (2*t->mag_z_mat->data[0][0]*(t->predicted_state.y) -
       2*t->mag_z_mat->data[1][0]*(t->predicted_state.x) +
       2*t->mag_z_mat->data[2][0]*(t->predicted_state.w));
    t->pred_cov_transformation_mat->data[2][1] =
      (2*t->mag_z_mat->data[0][0]*(t->predicted_state.z) -
       2*t->mag_z_mat->data[1][0]*(t->predicted_state.w) -
       2*t->mag_z_mat->data[2][0]*(t->predicted_state.x));
    t->pred_cov_transformation_mat->data[2][2] =
      (2*t->mag_z_mat->data[0][0]*(t->predicted_state.w) +
       2*t->mag_z_mat->data[1][0]*(t->predicted_state.z) -
       2*t->mag_z_mat->data[2][0]*(t->predicted_state.y));
    t->pred_cov_transformation_mat->data[2][3] =
      (2*t->mag_z_mat->data[0][0]*(t->predicted_state.x) +
       2*t->mag_z_mat->data[1][0]*(t->predicted_state.y) +
       2*t->mag_z_mat->data[2][0]*(t->predicted_state.z));
    transpose(t->pred_cov_transformation_mat,t->pred_cov_transformation_trans);

    // nrf_cli_fprintf(&m_cli_cdc_acm, 2,"\nH matrix:");
    // printMatrix(t->pred_cov_transformation_mat);
    //
    // nrf_cli_fprintf(&m_cli_cdc_acm, 2,"\nHtranspose matrix:");
    // printMatrix(t->pred_cov_transformation_trans);
    mat_mul(t->pred_state_cov_mat,t->pred_cov_transformation_trans,
      t->intrim_pred_cov_mat); //(4x4) * (4x3) = (4x3)
    // nrf_cli_fprintf(&m_cli_cdc_acm, 2,"\nINTER MAT:");
    // printMatrix(t->intrim_pred_cov_mat);
    mat_mul(t->pred_cov_transformation_mat,t->intrim_pred_cov_mat,
      t->pred_cov_plus_R_mat);  //(3x4) * (4x3) = (3x3)

    t->variance = 1.3;
    t->difference = fabs(t->mag_l2_norm - t->mag_avg);
    t->observed_mag_l2_norm = fabs(norm(t->observed_data_matrix));
    if (t->counter <= INIT_SAMPLES) {
      t->variance += fmax((t->difference - 0.02),0.0) * 2;
    } else if (t->counter > INIT_SAMPLES && (t->gyr_l2_norm > 0.02
       || t->difference > 1.5)) {
         t->variance += fmax(t->observed_mag_l2_norm - 0.3,0) * 10000;
    } else {
      t->variance += fmax(t->observed_mag_l2_norm,0.0) * 10;
    }
    for (size_t i = 0; i < t->R->rows; i++)
    for (size_t j = 0; j < t->R->cols; j++)
      t->R->data[i][j] = (i==j)?t->variance:0;

    mat_add(t->pred_cov_plus_R_mat,t->R);
    inverse(t->pred_cov_plus_R_mat,t->pred_cov_plus_R_inverse);
    // nrf_cli_fprintf(&m_cli_cdc_acm, 2,"\nInverse matrix:");
    // printMatrix(t->pred_cov_plus_R_inverse);
    // nrf_cli_fprintf(&m_cli_cdc_acm, 2,"\nOf:");
    // printMatrix(t->pred_cov_plus_R_mat);
    mat_mul(t->intrim_pred_cov_mat,t->pred_cov_plus_R_inverse,
      t->kalman_gain);
    //================= KALMAN GAIN COMPUTATION DONE=======================

    // x_k = x_k_p + K*(Z) UPDATING PREDICTED STATE from pre steps=============
    mat_mul(t->kalman_gain,t->observed_data_matrix,t->updated_pred_state);
    quat_to_mat_copy(&(t->predicted_state),t->updated_pred_state);

    mat_add(t->updated_pred_state,t->pred_state);
    normalize(t->updated_pred_state,0.0);
    //====================UPDATING PREDICTION DONE========================

    // p_k = (I - K*H)*p_k_p UPDATING PREDICTED_STATE_COVARIANCE_MATRIX
    mat_mul(t->kalman_gain,t->pred_cov_transformation_mat,
      t->kg_transformation_mat);

    for (size_t i = 0; i < t->kg_transformation_idn->rows; i++)
    for (size_t j = 0; j < t->kg_transformation_idn->cols; j++)
      t->kg_transformation_idn->data[i][j] = (i==j)?1:0;

    mat_sub(t->kg_transformation_idn,t->kg_transformation_mat);
    mat_mul(t->kg_transformation_idn,t->pred_state_cov_mat,
      t->updated_state_cov_mat);
    // nrf_cli_fprintf(&m_cli_cdc_acm, 2,"\nUPDATED COV STATE:");
    // printMatrix(t->updated_state_cov_mat);
    //==========UPDATING PREDICTED_STATE_COVARIANCE_MATRIX DONE===============

    t->rotation_mat->data[0][0] =
     (pow(t->updated_pred_state->data[0][0],2) +
      pow(t->updated_pred_state->data[1][0],2) -
      pow(t->updated_pred_state->data[2][0],2) -
      pow(t->updated_pred_state->data[3][0],2));
    t->rotation_mat->data[0][1] =
     (2*(t->updated_pred_state->data[1][0] * t->updated_pred_state->data[2][0]) +
      2*(t->updated_pred_state->data[0][0] * t->updated_pred_state->data[3][0]));
    t->rotation_mat->data[0][2] =
     (2*(t->updated_pred_state->data[1][0] * t->updated_pred_state->data[3][0]) +
      2*(t->updated_pred_state->data[0][0] * t->updated_pred_state->data[2][0]));

    t->rotation_mat->data[1][0] =
     (2*(t->updated_pred_state->data[1][0] * t->updated_pred_state->data[2][0]) -
      2*(t->updated_pred_state->data[0][0] * t->updated_pred_state->data[3][0]));
    t->rotation_mat->data[1][1] =
     (pow(t->updated_pred_state->data[0][0],2) -
      pow(t->updated_pred_state->data[1][0],2) +
      pow(t->updated_pred_state->data[2][0],2) -
      pow(t->updated_pred_state->data[3][0],2));
    t->rotation_mat->data[1][2] =
     (2*(t->updated_pred_state->data[2][0] * t->updated_pred_state->data[3][0]) +
      2*(t->updated_pred_state->data[0][0] * t->updated_pred_state->data[1][0]));

    t->rotation_mat->data[2][0] =
     (2*(t->updated_pred_state->data[1][0] * t->updated_pred_state->data[3][0]) +
      2*(t->updated_pred_state->data[0][0] * t->updated_pred_state->data[2][0]));
    t->rotation_mat->data[2][1] =
     (2*(t->updated_pred_state->data[2][0] * t->updated_pred_state->data[3][0]) -
      2*(t->updated_pred_state->data[0][0] * t->updated_pred_state->data[1][0]));
    t->rotation_mat->data[2][2] =
     (pow(t->updated_pred_state->data[0][0],2) -
      pow(t->updated_pred_state->data[1][0],2) -
      pow(t->updated_pred_state->data[2][0],2)+
      pow(t->updated_pred_state->data[3][0],2));

    inverse(t->rotation_mat,t->R);
    mat_mul(t->R,t->x_vector,t->mzl);
    if(t->counter > INIT_SAMPLES){
      mat_copy(t->mzl,t->mag_z_mat);
    }
  }

  if(t->counter > INIT_SAMPLES) {
    quatCopy(&(t->previous_state),&(t->predicted_state));
    quat_inverse(&(t->predicted_state));

    mat_to_quat_copy(t->updated_pred_state,&(t->temp_quat));
    quatMultiplication(&(t->predicted_state),&(t->temp_quat),&(t->quaternion));

    quatCopy(&(t->quaternion),&(t->predicted_state));

    quat_to_mat_copy(&(t->predicted_state),t->temp_quat_mat2);
    mat_sub(t->temp_quat_mat2,t->temp_quat_mat1);
    mat_to_quat_copy(t->temp_quat_mat2,&(t->predicted_state));

    // if(t->gyr_l2_norm < 0.02) {
    //   mat_copy(t->temp_quat_mat1,t->updated_pred_state);
    //   t->sinp =
    //    2 * ((t->temp_quat_mat1->data[0][0] * t->temp_quat_mat1->data[2][0]) -
    //         (t->temp_quat_mat1->data[3][0] * t->temp_quat_mat1->data[1][0]));
    //   t->roll_new = asin(t->sinp);
    //
    //   quat_to_mat_copy(&(t->previous_state),t->temp_quat_mat1);
    //   t->siny =
    //    2 * ((t->temp_quat_mat1->data[0][0] * t->temp_quat_mat1->data[3][0]) +
    //         (t->temp_quat_mat1->data[1][0] * t->temp_quat_mat1->data[2][0]));
    //   t->cosy =
    //    (pow(t->temp_quat_mat1->data[0][0],2) + pow(t->temp_quat_mat1->data[1][0],2) -
    //     pow(t->temp_quat_mat1->data[2][0],2) - pow(t->temp_quat_mat1->data[3][0],2));
    //   t->yaw_old = atan2(t->siny,t->cosy);
    //
    //   t->sinr =
    //    2 * ((t->temp_quat_mat1->data[0][0] * t->temp_quat_mat1->data[1][0]) +
    //         (t->temp_quat_mat1->data[2][0] * t->temp_quat_mat1->data[3][0]));
    //   t->cosr =
    //    (pow(t->temp_quat_mat1->data[0][0],2) - pow(t->temp_quat_mat1->data[1][0],2) -
    //     pow(t->temp_quat_mat1->data[2][0],2) + pow(t->temp_quat_mat1->data[3][0],2));
    //   t->pitch_new = atan2(t->sinr,t->cosr);
    //
    //   t->c1 = cos(t->yaw_old * 0.5);
    //   t->s1 = sin(t->yaw_old * 0.5);
    //   t->c2 = cos(t->roll_new * 0.5);
    //   t->s2 = sin(t->roll_new * 0.5);
    //   t->c3 = cos(t->pitch_new * 0.5);
    //   t->s3 = sin(t->pitch_new * 0.5);
    //
    //   t->updated_pred_state->data[0][0] =
    //     ((t->c1 * t->c2 * t->c3) + (t->s1 * t->s2 * t->s3));
    //   t->updated_pred_state->data[1][0] =
    //    ((t->c1 * t->c2 * t->s3) - (t->s1 * t->s2 * t->c3));
    //   t->updated_pred_state->data[2][0] =
    //    ((t->c1 * t->s2 * t->c3) + (t->s1 * t->c2 * t->s3));
    //   t->updated_pred_state->data[3][0] =
    //    ((t->s1 * t->c2 * t->c3) - (t->c1 * t->s2 * t->s3));
    // }
  }

  mat_to_quat_copy(t->updated_pred_state,&(t->previous_state));
  mat_copy(t->updated_state_cov_mat,t->prev_state_cov_mat);
  mat_to_quat_copy(t->updated_pred_state,&(t->quaternion));
}

imu_orientation_t bsn_imu_orientation_get_quaternion() {
  //just return your local quaternion data here
  return (t->quaternion);
}
