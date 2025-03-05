/**
 * Copyright (C) 2015 by Liangliang Nan (liangliang.nan@gmail.com)
 * https://3d.bk.tudelft.nl/liangliang/
 *
 * This file is part of Easy3D. If it is useful in your research/work,
 * I would be grateful if you show your appreciation by citing it:
 * ------------------------------------------------------------------
 *      Liangliang Nan.
 *      Easy3D: a lightweight, easy-to-use, and efficient C++
 *      library for processing and rendering 3D data. 2018.
 * ------------------------------------------------------------------
 * Easy3D is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 3
 * as published by the Free Software Foundation.
 *
 * Easy3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "calibration.h"
#include "matrix_algo.h"



using namespace easy3d;



/**
 * TODO: Finish this function for calibrating a camera from the corresponding 3D-2D point pairs.
 *       You may define a few functions for some sub-tasks.
 * @return True on success, otherwise false. On success, the camera parameters are returned by fx, fy, cx, cy, skew, R, and t).
 */
bool Calibration::calibration(
        const std::vector<Vector3D>& points_3d, /// input: An array of 3D points.
        const std::vector<Vector2D>& points_2d, /// input: An array of 2D image points.
        double& fx,  /// output: focal length (i.e., K[0][0]).
        double& fy,  /// output: focal length (i.e., K[1][1]).
        double& cx,  /// output: x component of the principal point (i.e., K[0][2]).
        double& cy,  /// output: y component of the principal point (i.e., K[1][2]).
        double& s,   /// output: skew factor (i.e., K[0][1]), which is s = -alpha * cot(theta).
        Matrix33& R, /// output: the 3x3 rotation matrix encoding camera rotation.
        Vector3D& t) /// output：a 3D vector encoding camera translation.
{
    std::cout << "\nTODO: implement the 'calibration()' function in the file 'Calibration/calibration_method.cpp'\n\n";

    std::cout << "[Liangliang]:\n"
                 "\tIn this assignment, two essential data structures, 'Matrix' and 'Vector', are provided for the\n"
                 "\tmanipulation and storage of matrices and vectors. These data structures are defined in:\n"
                 "\t    - Calibration/matrix.h: handles matrices of arbitrary dimensions and related functions.\n"
                 "\t    - Calibration/vector.h: manages vectors of arbitrary sizes and related functions.\n"
                 "\tCamera calibration requires computing the SVD and inverse of matrices. These functions, along\n"
                 "\twith several other relevant ones, are provided in:\n"
                 "\t    - Calibration/matrix_algo.h: contains functions for determinant, inverse, SVD, linear least-squares...\n"
                 "\tIn the 'Calibration::calibration(...)' function, code snippets are provided for your reference.\n"
                 "\tFor more details about these data structures and a complete list of related functions, please\n"
                 "\trefer to the header files mentioned above.\n\n"
                 "\tFor your final submission, adhere to the following guidelines:\n"
                 "\t    - submit ONLY the 'Calibration/calibration_method.cpp' file.\n"
                 "\t    - remove ALL unrelated test code, debugging code, and comments.\n"
                 "\t    - ensure that your code compiles and can reproduce your results WITHOUT ANY modification.\n\n" << std::flush;


    // TODO: the above code just demonstrates some useful data structures and APIs. Please remove all above code in your
    //       final submission.

    //--------------------------------------------------------------------------------------------------------------
    // implementation starts ...

    std::cout << "\n[Liangliang]:\n"
                 "\tThis function takes two arrays as input parameters:\n"
                 "\t\t- points_3d: An array of 3D points representing the scene\n"
                 "\t\t- points_2d: An array of 2D image points corresponding to the 3D points\n"
                 "\tThe function should return either 'true' upon successful calibration or 'false' otherwise.\n"
                 "\tUpon success, the following parameters must be stored in the specified variables:\n"
                 "\t\t- fx and fy: focal lengths along the x and y axes, respectively\n"
                 "\t\t- cx and cy: coordinates of the principal point\n"
                 "\t\t- s: the skew factor, i.e., s = -alpha * cot(theta)\n"
                 "\t\t- R: the 3x3 rotation matrix encoding camera orientation\n"
                 "\t\t- t: a 3D vector encoding camera location.\n"
                 "\tIMPORTANT: don't forget to write your recovered parameters to the above variables." << std::endl;

    // TODO: check if input is valid (e.g., number of correspondences >= 6, sizes of 2D/3D points must match)
    if (points_2d.size() != points_3d.size()){
        std::cout << "Incorrect number of points in the array." << std::endl;
        return false;
    }
    if (points_3d.size() < 6) {
        std::cout << "At least 6 number of points are required for calibration." << std::endl;
        return false;
    }


    // TODO: construct the P matrix (so P * m = 0).
    int point_num = points_3d.size();
    Matrix P(2*point_num,12,0.0);
    for (int i = 0; i < point_num; i++) {
        // read every pair of 2d and 3d point
        const Vector3D& X = points_3d[i];
        const Vector2D& x = points_2d[i];

        // add rows
        P.set_row(2*i, {X[0], X[1], X[2], 1, 0, 0, 0, 0, -x[0]*X[0], -x[0]*X[1], -x[0]*X[2],-x[0]});
        P.set_row(2*i+1, {0, 0, 0, 0, X[0], X[1], X[2], 1, -x[1]*X[0], -x[1]*X[1], -x[1]*X[2],-x[1]});
    };


    // TODO: solve for M (the whole projection matrix, i.e., M = K * [R, t]) using SVD decomposition.
    //   Optional: you can check if your M is correct by applying M on the 3D points. If correct, the projected point
    //             should be very close to your input images points.

    // perform SVD decomposition on P
    Matrix U(2*(point_num),2*point_num, 0.0);
    Matrix D(2*point_num, 12, 0.0);
    Matrix V(12, 12, 0.0);
    svd_decompose(P, U, D, V);
    // M is the last column of V
    Vector M = V.get_column(V.cols() - 1);



    // TODO: extract intrinsic parameters from M.
        
    // 1) Rearrange the 12-dimensional vector M into a 3×4 matrix M34 (3 rows, 4 columns)
    Matrix34 M34(
        M[0], M[1], M[2], M[3],
        M[4], M[5], M[6], M[7],
        M[8], M[9], M[10], M[11]
    );

    // 2) Split: A = first 3×3 of M34, b = last column
    Matrix33 A(
        M34(0, 0), M34(0, 1), M34(0, 2),
        M34(1, 0), M34(1, 1), M34(1, 2),
        M34(2, 0), M34(2, 1), M34(2, 2)
    );
    Vector3D b(M34(0, 3), M34(1, 3), M34(2, 3));

    // 3) Define some auxiliary vectors: a1, a2, a3 are the 3 rows of A respectively
    Vector3D a1(A(0, 0), A(0, 1), A(0, 2));
    Vector3D a2(A(1, 0), A(1, 1), A(1, 2));
    Vector3D a3(A(2, 0), A(2, 1), A(2, 2));

    // 4) Calculate ρ = 1 / ||a3||
    double norm_a3 = a3.length();
    if (norm_a3 < 1e-12) {
        std::cerr << "[Error] norm(a3) too small.\n";
        return false; // Unable to calibrate
    }
    double rho = 1.0 / norm_a3;
    double rho2 = rho * rho; 

    // 5) Calculate cx, cy
    //    cx = ρ^2 * (a1 · a3),  cy = ρ^2 * (a2 · a3)
    double a1dot_a3 = dot(a1, a3);
    double a2dot_a3 = dot(a2, a3);
    cx = rho2 * a1dot_a3;
    cy = rho2 * a2dot_a3;

    // 6) Calculate cosθ、sinθ
    Vector cross_a1_a3 = cross(a1, a3);
    Vector cross_a2_a3 = cross(a2, a3);
    double norm_a1_a3 = cross_a1_a3.length();
    double norm_a2_a3 = cross_a2_a3.length();

    if (norm_a1_a3 < 1e-12 || norm_a2_a3 < 1e-12) {
        std::cerr << "[Error] cross() degenerate.\n";
        return false;
    }
    double cos_theta = -dot(cross_a1_a3, cross_a2_a3) / (norm_a1_a3 * norm_a2_a3);

    double sin_theta = std::sqrt(std::max(0.0, 1.0 - cos_theta * cos_theta));

    // 7) Calculate fx, fy:   fx = ρ² * ||a1×a3|| * sinθ;   fy = ρ² * ||a2×a3|| * sinθ
    fx = rho2 * norm_a1_a3 * sin_theta;
    fy = rho2 * norm_a2_a3 * sin_theta;

    // 8) Calculate skew s
    //   s = - fx * cotθ = - fx * (cosθ / sinθ)
    if (std::abs(sin_theta) < 1e-12) {
        s = 0.0; // when theta->0,skew->0
    }
    else {
        s = -fx * (cos_theta / sin_theta);
    }

        
    // TODO: extract extrinsic parameters from M.
    Vector r1 = cross_a2_a3 / norm_a2_a3;
    Vector r3 = rho * a3;
    Vector r2 = cross(r3, r1);
    // rotation matrix
    R.set_row(0, r1);
    R.set_row(1, r2);
    R.set_row(2, r3);

    Matrix33 K(fx, s, cx,
        0, fy, cy,
        0, 0, 1);
    Matrix33 K_inverse;
    inverse(K, K_inverse);
    t = rho * K_inverse * b;

    // TODO: make sure the recovered parameters are passed to the corresponding variables (fx, fy, cx, cy, s, R, and t)
    std::cout << "fx: " << fx << ", fy: " << fy << std::endl;
    std::cout << "cx: " << cx << ", cy: " << cy << std::endl;
    std::cout << "skew: " << s << std::endl;
    std::cout << "R: " << R << std::endl;
    std::cout << "t: " << t << std::endl;
    std::cout << "\n\tTODO: After you implement this function, please return 'true' - this will trigger the viewer to\n"
                 "\t\tupdate the rendering using your recovered camera parameters. This can help you to visually check\n"
                 "\t\tif your calibration is successful or not.\n\n" << std::flush;
    return true;
}

















