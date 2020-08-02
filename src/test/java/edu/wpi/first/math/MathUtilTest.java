/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.math;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class MathUtilTest {
  @Test
  void testAngleNormalize() {
    double x = 5 * Math.PI;
    double y = -5 * Math.PI;
    double a = Math.PI / 2;
    double b = -Math.PI / 2;

    assertEquals(MathUtil.normalizeAngle(x), Math.PI);
    assertEquals(MathUtil.normalizeAngle(y), Math.PI);
    assertEquals(MathUtil.normalizeAngle(a), a);
    assertEquals(MathUtil.normalizeAngle(b), b);
  }
}
