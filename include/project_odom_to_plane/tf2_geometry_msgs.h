#ifndef TF2_GEOMETRY_MSGS_H_2
#define TF2_GEOMETRY_MSGS_H_2

/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Additional code which is not yet merged in kinetic (and probably will never be).
 * ######## REMOVE THIS WHEN CODE IS AVAILABLE DOWNSTREAM OR USING NEWER DISTRO ###########
 */

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace tf2
{

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Pose type.
* This function is a specialization of the doTransform template defined in tf2/convert.h.
* \param t_in The pose to transform, as a Pose3 message.
* \param t_out The transformed pose, as a Pose3 message.
* \param transform The timestamped transform to apply, as a TransformStamped message.
*/
template <>
inline
void doTransform(const geometry_msgs::Pose& t_in, geometry_msgs::Pose& t_out, const geometry_msgs::TransformStamped& transform)
{
  tf2::Vector3 v;
  fromMsg(t_in.position, v);
  tf2::Quaternion r;
  fromMsg(t_in.orientation, r);

  tf2::Transform t;
  fromMsg(transform.transform, t);
  tf2::Transform v_out = t * tf2::Transform(r, v);
  toMsg(v_out, t_out);
}

/** \brief Transform the covariance matrix of a PoseWithCovarianceStamped message to a new frame.
* \param t_in The covariance matrix to transform.
* \param transform The timestamped transform to apply, as a TransformStamped message.
* \return The transformed covariance matrix.
*/
inline
geometry_msgs::PoseWithCovariance::_covariance_type transformCovariance(const geometry_msgs::PoseWithCovariance::_covariance_type& cov_in, const tf2::Transform& transform)
{
  /**
   * To transform a covariance matrix:
   *
   * [R 0] COVARIANCE [R' 0 ]
   * [0 R]            [0  R']
   *
   * Where:
   * 	R is the rotation matrix (3x3).
   * 	R' is the transpose of the rotation matrix.
   * 	COVARIANCE is the 6x6 covariance matrix to be transformed.
   */

  // get rotation matrix transpose
  const tf2::Matrix3x3  R_transpose = transform.getBasis().transpose();

  // convert the covariance matrix into four 3x3 blocks
  const tf2::Matrix3x3 cov_11(cov_in[0], cov_in[1], cov_in[2],
            cov_in[6], cov_in[7], cov_in[8],
            cov_in[12], cov_in[13], cov_in[14]);
  const tf2::Matrix3x3 cov_12(cov_in[3], cov_in[4], cov_in[5],
            cov_in[9], cov_in[10], cov_in[11],
            cov_in[15], cov_in[16], cov_in[17]);
  const tf2::Matrix3x3 cov_21(cov_in[18], cov_in[19], cov_in[20],
            cov_in[24], cov_in[25], cov_in[26],
            cov_in[30], cov_in[31], cov_in[32]);
  const tf2::Matrix3x3 cov_22(cov_in[21], cov_in[22], cov_in[23],
            cov_in[27], cov_in[28], cov_in[29],
            cov_in[33], cov_in[34], cov_in[35]);

  // perform blockwise matrix multiplication
  const tf2::Matrix3x3 result_11 = transform.getBasis()*cov_11*R_transpose;
  const tf2::Matrix3x3 result_12 = transform.getBasis()*cov_12*R_transpose;
  const tf2::Matrix3x3 result_21 = transform.getBasis()*cov_21*R_transpose;
  const tf2::Matrix3x3 result_22 = transform.getBasis()*cov_22*R_transpose;

  // form the output
  geometry_msgs::PoseWithCovariance::_covariance_type output;
  output[0] = result_11[0][0];
  output[1] = result_11[0][1];
  output[2] = result_11[0][2];
  output[6] = result_11[1][0];
  output[7] = result_11[1][1];
  output[8] = result_11[1][2];
  output[12] = result_11[2][0];
  output[13] = result_11[2][1];
  output[14] = result_11[2][2];

  output[3] = result_12[0][0];
  output[4] = result_12[0][1];
  output[5] = result_12[0][2];
  output[9] = result_12[1][0];
  output[10] = result_12[1][1];
  output[11] = result_12[1][2];
  output[15] = result_12[2][0];
  output[16] = result_12[2][1];
  output[17] = result_12[2][2];

  output[18] = result_21[0][0];
  output[19] = result_21[0][1];
  output[20] = result_21[0][2];
  output[24] = result_21[1][0];
  output[25] = result_21[1][1];
  output[26] = result_21[1][2];
  output[30] = result_21[2][0];
  output[31] = result_21[2][1];
  output[32] = result_21[2][2];

  output[21] = result_22[0][0];
  output[22] = result_22[0][1];
  output[23] = result_22[0][2];
  output[27] = result_22[1][0];
  output[28] = result_22[1][1];
  output[29] = result_22[1][2];
  output[33] = result_22[2][0];
  output[34] = result_22[2][1];
  output[35] = result_22[2][2];

  return output;
}


/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs PoseWithCovarianceStamped type.
* This function is a specialization of the doTransform template defined in tf2/convert.h.
* \param t_in The pose to transform, as a timestamped PoseWithCovarianceStamped message.
* \param t_out The transformed pose, as a timestamped PoseWithCovarianceStamped message.
* \param transform The timestamped transform to apply, as a TransformStamped message.
*/
template <>
inline
void doTransform(const geometry_msgs::PoseWithCovarianceStamped& t_in, geometry_msgs::PoseWithCovarianceStamped& t_out, const geometry_msgs::TransformStamped& transform)
{
  tf2::Vector3 v;
  fromMsg(t_in.pose.pose.position, v);
  tf2::Quaternion r;
  fromMsg(t_in.pose.pose.orientation, r);

  tf2::Transform t;
  fromMsg(transform.transform, t);
  tf2::Transform v_out = t * tf2::Transform(r, v);
  toMsg(v_out, t_out.pose.pose);
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;

  t_out.pose.covariance = transformCovariance(t_in.pose.covariance, t);
}

} // namespace

#endif // TF2_GEOMETRY_MSGS_H_2
