#ifndef QUATERNION_H
#define QUATERNION_H

#define RAD_TO_DEG 57.29577951f
#define DEG_TO_RAD 0.01745329f

typedef struct 
{
    float w, x, y, z;
} quat_t;

typedef struct
{
    float x, y, z;
} vector_t;

vector_t cross_product(vector_t *vec1, vector_t *vec2);
void get_quat_deriv(quat_t *q_ptr, vector_t *vect, quat_t *q_dot_ptr);
void norm_vector(vector_t *vector_ptr);
void norm_quat(quat_t *quat_ptr);
quat_t quat_conj(quat_t *q_ptr);
void quat_to_euler(quat_t *q_ptr, vector_t *att);
void get_attitude_from_accel(vector_t *vec, quat_t *q_result);
void get_heading_from_mag(vector_t *vec, quat_t *q_result);
quat_t get_quat_product(quat_t *q1, quat_t *q2);

#endif /*QUATERNION_H*/