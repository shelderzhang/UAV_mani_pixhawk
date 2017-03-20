/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file angular_acc_ekf_param.c
 *
 * Angular rate estimator (ekf based)
 *
 * @author bdai <bdai1412@gmail.com>
 */

#include <systemlib/param/param.h>

/**
 * progress noise of angular acceleration
 *
 * @group Angular acceleration and acceleration estimator
 * @min 0.001
 * @max 10
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(AAE_PN_ANGACC, 0.02f);

/**
 * progress noise of angular rate
 *
 * @group Angular acceleration and acceleration estimator
 * @min 0.001
 * @max 10
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(AAE_PN_ANGRATE, 0.02f);

/**
 * progress noise of acceleration
 *
 * @group Angular acceleration and acceleration estimator
 * @min 0.001
 * @max 10
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(AAE_PN_ACC, 0.02f);

/**
 * progress noise of velocity
 *
 * @group Angular acceleration and acceleration estimator
 * @min 0.001
 * @max 10
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(AAE_PN_VEL, 0.05f);

/**
 * angular rate standard deviation
 *
 * @group Angular acceleration and acceleration estimator
 * @min 0.001
 * @max 10
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(AAE_STD_ANGRAT, 0.001f);

/**
 * angular acceleration standard deviation
 *
 * @group Angular acceleration and acceleration estimator
 * @min 0.001
 * @max 10
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(AAE_STD_ANGACC, 0.1f);

/**
 * acceleration standard deviation
 *
 * @group Angular acceleration and acceleration estimator
 * @min 0.001
 * @max 10
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(AAE_STD_ACC, 0.01f);

/**
 * velocity standard deviation
 *
 * @group Angular acceleration and acceleration estimator
 * @min 0.001
 * @max 10
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(AAE_STD_VEL, 0.01f);

/**
 * Cut frequency
 *
 * @group Angular acceleration and acceleration estimator
 * @min 5
 * @max 100
 * @decimal 0
 */
PARAM_DEFINE_FLOAT(AAE_X_ANG_LP, 20.0f);

/**
 * Cut frequency for state publication
 *
 * @group Angular acceleration and acceleration estimator
 * @min 5
 * @max 100
 * @decimal 0
 */
PARAM_DEFINE_FLOAT(AAE_X_ACC_LP, 10.0f);
