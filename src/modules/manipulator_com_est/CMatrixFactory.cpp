#include "CMatrixFactory.h"

Matrix4f H_transls(float theta, float a, float d, float alpha) {
	Matrix4f mat_tr;

	//First row
	float r11 = cosf(theta);
	float r12 = -sinf(theta) * cosf(alpha);
	float r13 = sinf(theta) * sinf(alpha);
	float r14 = a * cosf(theta);
	//Second row
	float r21 = sinf(theta);
	float r22 = cosf(theta) * cosf(alpha);
	float r23 = -cosf(theta) * sinf(alpha);
	float r24 = a * sinf(theta);
	//Third row
	float r31 = 0.0f;
	float r32 = sinf(alpha);
	float r33 = cosf(alpha);
	float r34 = d;
	//Forth row
	float r41 = 0.0f;
	float r42 = 0.0f;
	float r43 = 0.0f;
	float r44 = 1.0f;
	mat_tr(0, 0) = r11;
	mat_tr(0, 1) = r12;
	mat_tr(0, 2) = r13;
	mat_tr(0, 3) = r14;

	mat_tr(1, 0) = r21;
	mat_tr(1, 1) = r22;
	mat_tr(1, 2) = r23;
	mat_tr(1, 3) = r24;

	mat_tr(2, 0) = r31;
	mat_tr(2, 1) = r32;
	mat_tr(2, 2) = r33;
	mat_tr(2, 3) = r34;

	mat_tr(3, 0) = r41;
	mat_tr(3, 1) = r42;
	mat_tr(3, 2) = r43;
	mat_tr(3, 3) = r44;

	return mat_tr;
}

Matrix4f trotz(float alpha) {
	Matrix4f _temp;

	//First row
	float r11 = cosf(alpha);
	float r12 = -sinf(alpha);
	float r13 = 0.0f;
	float r14 = 0.0f;
	//Second row
	float r21 = sinf(alpha);
	float r22 = cosf(alpha);
	float r23 = 0.0f;
	float r24 = 0.0f;
	//Third row
	float r31 = 0.0f;
	float r32 = 0.0f;
	float r33 = 1.0f;
	float r34 = 0.0f;
	//Forth row
	float r41 = 0.0f;
	float r42 = 0.0f;
	float r43 = 0.0f;
	float r44 = 0.0f;

	_temp(0, 0) = r11;
	_temp(0, 1) = r12;
	_temp(0, 2) = r13;
	_temp(0, 3) = r14;

	_temp(1, 0) = r21;
	_temp(1, 1) = r22;
	_temp(1, 2) = r23;
	_temp(1, 3) = r24;

	_temp(2, 0) = r31;
	_temp(2, 1) = r32;
	_temp(2, 2) = r33;
	_temp(2, 3) = r34;

	_temp(3, 0) = r41;
	_temp(3, 1) = r42;
	_temp(3, 2) = r43;
	_temp(3, 3) = r44;

	return _temp;
}

Matrix4f troty(float beta) {
	Matrix4f _temp;

	//First row
	float r11 = cosf(beta);
	float r12 = 0.0f;
	float r13 = sinf(beta);
	float r14 = 0.0f;
	//Second row
	float r21 = 0.0f;
	float r22 = 1.0f;
	float r23 = 0.0f;
	float r24 = 0.0f;
	//Third row
	float r31 = -sinf(beta);
	float r32 = 0.0f;
	float r33 = cosf(beta);
	float r34 = 0.0f;
	//Forth row
	float r41 = 0.0f;
	float r42 = 0.0f;
	float r43 = 0.0f;
	float r44 = 0.0f;

	_temp(0, 0) = r11;
	_temp(0, 1) = r12;
	_temp(0, 2) = r13;
	_temp(0, 3) = r14;

	_temp(1, 0) = r21;
	_temp(1, 1) = r22;
	_temp(1, 2) = r23;
	_temp(1, 3) = r24;

	_temp(2, 0) = r31;
	_temp(2, 1) = r32;
	_temp(2, 2) = r33;
	_temp(2, 3) = r34;

	_temp(3, 0) = r41;
	_temp(3, 1) = r42;
	_temp(3, 2) = r43;
	_temp(3, 3) = r44;

	return _temp;
}

Matrix4f trotx(float gamma) {
	Matrix4f _temp;

	//First row
	float r11 = 1.0f;
	float r12 = 0.0f;
	float r13 = 0.0f;
	float r14 = 0.0f;
	//Second row
	float r21 = 0.0f;
	float r22 = cosf(gamma);
	float r23 = -sinf(gamma);
	float r24 = 0.0f;
	//Third row
	float r31 = 0.0f;
	float r32 = sinf(gamma);
	float r33 = cosf(gamma);
	float r34 = 0.0f;
	//Forth row
	float r41 = 0.0f;
	float r42 = 0.0f;
	float r43 = 0.0f;
	float r44 = 0.0f;

	_temp(0, 0) = r11;
	_temp(0, 1) = r12;
	_temp(0, 2) = r13;
	_temp(0, 3) = r14;

	_temp(1, 0) = r21;
	_temp(1, 1) = r22;
	_temp(1, 2) = r23;
	_temp(1, 3) = r24;

	_temp(2, 0) = r31;
	_temp(2, 1) = r32;
	_temp(2, 2) = r33;
	_temp(2, 3) = r34;

	_temp(3, 0) = r41;
	_temp(3, 1) = r42;
	_temp(3, 2) = r43;
	_temp(3, 3) = r44;

	return _temp;
}

Matrix4f translx(float dx) {
	Matrix4f _temp;
	_temp.identity();
	_temp(0, 3) = dx;

	return _temp;
}

Matrix4f transly(float dy) {
	Matrix4f _temp;
	_temp.identity();

	_temp(1, 3) = dy;

	return _temp;
}

Matrix4f translz(float dz) {
	Matrix4f _temp;
	_temp.identity();

	_temp(2, 3) = dz;

	return _temp;
}
Matrix4f transl_vector(Vector3f &vec) {
	Matrix4f _temp;
	_temp.identity();

	_temp(0, 3) = vec(0);
	_temp(1, 3) = vec(1);
	_temp(2, 3) = vec(2);

	return _temp;
}

Matrix3f ext_rotation(Matrix4f & hm) {
	Matrix3f _temp;
	_temp(0, 0) = hm(0, 0);
	_temp(0, 1) = hm(0, 1);
	_temp(0, 2) = hm(0, 2);

	_temp(1, 0) = hm(1, 0);
	_temp(1, 1) = hm(1, 1);
	_temp(1, 2) = hm(1, 2);

	_temp(2, 0) = hm(2, 0);
	_temp(2, 1) = hm(2, 1);
	_temp(2, 2) = hm(2, 2);

	return _temp;
}

Vector3f ext_translvector(Matrix4f & hm) {
	Vector3f vec;

	vec(0) = hm(0, 3);
	vec(1) = hm(1, 3);
	vec(2) = hm(2, 3);

	return vec;
}

