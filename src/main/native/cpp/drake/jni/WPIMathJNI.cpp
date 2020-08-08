/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <jni.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <unsupported/Eigen/MatrixFunctions>

#include "drake/math/discrete_algebraic_riccati_equation.h"
#include "edu_wpi_first_math_WPIMathJNI.h"

bool check_stabilizable(const Eigen::Ref<const Eigen::MatrixXd>& A,
                        const Eigen::Ref<const Eigen::MatrixXd>& B) {
  // This function checks if (A,B) is a stabilizable pair.
  // (A,B) is stabilizable if and only if the uncontrollable eigenvalues of
  // A, if any, have absolute values less than one, where an eigenvalue is
  // uncontrollable if Rank[lambda * I - A, B] < n.
  int n = B.rows(), m = B.cols();
  Eigen::EigenSolver<Eigen::MatrixXd> es(A);
  for (int i = 0; i < n; i++) {
    if (es.eigenvalues()[i].real() * es.eigenvalues()[i].real() +
            es.eigenvalues()[i].imag() * es.eigenvalues()[i].imag() <
        1)
      continue;

    Eigen::MatrixXcd E(n, n + m);
    E << es.eigenvalues()[i] * Eigen::MatrixXcd::Identity(n, n) - A, B;
    Eigen::ColPivHouseholderQR<Eigen::MatrixXcd> qr(E);
    if (qr.rank() != n) {
      return false;
    }
  }

  return true;
}

extern "C" {

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void* reserved) {
  JNIEnv* env;
  if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK)
    return JNI_ERR;

  return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL JNI_OnUnload(JavaVM* vm, void* reserved) {}

/*
 * Class:     edu_wpi_first_math_WPIMathJNI
 * Method:    exp
 * Signature: ([DI[D)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_math_WPIMathJNI_exp
  (JNIEnv* env, jclass, jdoubleArray src, jint rows, jdoubleArray dst)
{
  jdouble* arrayBody = env->GetDoubleArrayElements(src, nullptr);

  Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
      Amat{arrayBody, rows, rows};
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Aexp =
      Amat.exp();

  env->ReleaseDoubleArrayElements(src, arrayBody, 0);
  env->SetDoubleArrayRegion(dst, 0, rows * rows, Aexp.data());
}

/*
 * Class:     edu_wpi_first_math_WPIMathJNI
 * Method:    isStabilizable
 * Signature: (II[D[D)Z
 */
JNIEXPORT jboolean JNICALL
Java_edu_wpi_first_math_WPIMathJNI_isStabilizable
  (JNIEnv* env, jclass, jint states, jint inputs, jdoubleArray aSrc,
   jdoubleArray bSrc)
{
  jdouble* nativeA = env->GetDoubleArrayElements(aSrc, nullptr);
  jdouble* nativeB = env->GetDoubleArrayElements(bSrc, nullptr);

  Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
      A{nativeA, states, states};

  Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
      B{nativeB, states, inputs};

  bool isStabilizable = check_stabilizable(A, B);

  env->ReleaseDoubleArrayElements(aSrc, nativeA, 0);
  env->ReleaseDoubleArrayElements(bSrc, nativeB, 0);

  return isStabilizable;
}

}  // extern "C"
