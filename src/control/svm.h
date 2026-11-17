/**
 * @file svm.h
 * @brief Space vector modulation (min/max zero-sequence injection).
 */
#ifndef SVM_H
#define SVM_H

/**
 * @brief Convert a stationary-frame voltage vector to three duty cycles.
 * Linear up to |v| = vdc / sqrt(3). Duties are clamped to
 * [INV_DUTY_MIN, INV_DUTY_MAX] to respect minimum pulse / bootstrap needs.
 */
void svm_calc(float v_alpha, float v_beta, float vdc, float duty[3]);

#endif /* SVM_H */
