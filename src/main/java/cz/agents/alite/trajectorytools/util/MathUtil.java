package cz.agents.alite.trajectorytools.util;

public class MathUtil {
      public static double clamp (double i, double low, double high) {
        return java.lang.Math.max (java.lang.Math.min (i, high), low);
      }


      public static float clamp (float i, float low, float high) {
        return java.lang.Math.max (java.lang.Math.min (i, high), low);
      }


      public static int clamp (int i, int low, int high) {
        return java.lang.Math.max (java.lang.Math.min (i, high), low);
      }


      public static long clamp (long i, long low, long high) {
        return java.lang.Math.max (java.lang.Math.min (i, high), low);
      }

      /**
       * Returns true if two doubles are considered equal. Tests if the absolute
       * difference between the two doubles has a difference less then a given
       * double (epsilon). Determining the given epsilon is highly dependant on the
       * precision of the doubles that are being compared.
       *
       * @param a double to compare.
       * @param b double to compare
       * @param epsilon double which is compared to the absolute difference of two
       * doubles to determine if they are equal.
       * @return true if a is considered equal to b.
       */
      public static boolean equals(double a, double b, double epsilon){
          return a == b ? true : Math.abs(a - b) < epsilon;
      }
}
