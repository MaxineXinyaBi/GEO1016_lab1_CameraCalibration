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

    /// Below are a few examples showing some useful data structures and functions.

    // // This is a 1D array of 'double' values. Alternatively, you can use 'double mat[25]' but you cannot change it
    // // length. With 'std::vector', you can append/delete/insert elements, and much more. The 'std::vector' can store
    // // not only 'double', but also any other types of objects. In case you may want to learn more about 'std::vector'
    // // check here: https://en.cppreference.com/w/cpp/container/vector
    // std::vector<double> array = {1, 3, 3, 4, 7, 6, 2, 8, 2, 8, 3, 2, 4, 9, 1, 7, 3, 23, 2, 3, 5, 2, 1, 5, 8, 9, 22};
    // array.push_back(5); // append 5 to the array (so the size will increase by 1).
    // array.insert(array.end(), 10, 3);  // append ten 3 (so the size will grow by 10).
    //
    // /// To access the value of an element.
    // double a = array[2];
    //
    // /// define a 2D vector/point
    // Vector2D b(1.1, 2.2);
    //
    // /// define a 3D vector/point
    // Vector3D c(1.1, 2.2, 3.3);
    //
    // /// get the Cartesian coordinates of a (a is treated as Homogeneous coordinates)
    // Vector2D p = c.cartesian();
    //
    // /// get the Homogeneous coordinates of p
    // Vector3D q = p.homogeneous();
    //
    // /// the length of a vector
    // double len = p.length();
    // /// the squared length of a vector
    // double sqr_len = p.length2();
    //
    // /// the dot product of two vectors
    // double dot_prod = dot(p, q);
    //
    // /// the cross product of two vectors
    // Vector cross_prod = cross(c, q);
    //
    // /// normalize this vector
    // cross_prod.normalize();
    //
    // // Define an m-by-n double valued matrix.
    // // Here I use the above array to initialize it. You can also use A(i, j) to initialize/modify/access its elements.
    // const int m = 6, n = 5;
    // Matrix A(m, n, array.data());    // 'array.data()' returns a pointer to the array.
    // std::cout << "M: \n" << A << std::endl;
    //
    // /// define a 3 by 4 matrix (and all elements initialized to 0.0)
    // Matrix M(3, 4, 0.0);
    //
    // /// set first row by a vector
    // M.set_row(0, Vector4D(1.1, 2.2, 3.3, 4.4));
    //
    // /// set second column by a vector
    // M.set_column(1, Vector3D(5.5, 5.5, 5.5));
    //
    // /// define a 3 by 3 matrix (and all elements initialized to 0.0)
    // Matrix33 B;
    //
    // /// define and initialize a 3 by 3 matrix
    // Matrix33 T(1.1, 2.2, 3.3,
    //            0, 2.2, 3.3,
    //            0, 0, 1);
    //
    // /// define and initialize a 3 by 4 matrix
    // Matrix34 P(1.1, 2.2, 3.3, 0,
    //            0, 2.2, 3.3, 1,
    //            0, 0, 1, 1);
    //
    // /// define a 15 by 9 matrix (and all elements initialized to 0.0)
    // Matrix W(15, 9, 0.0);
    // /// set the first row by a 9-dimensional vector
    // W.set_row(0, {0, 1, 2, 3, 4, 5, 6, 7, 8}); // {....} is equivalent to a std::vector<double>
    //
    // /// get the number of rows.
    // int num_rows = W.rows();
    //
    // /// get the number of columns.
    // int num_cols = W.cols();
    //
    // /// get the the element at row 1 and column 2
    // double value = W(1, 2);
    //
    // /// get the last column of a matrix
    // Vector last_column = W.get_column(W.cols() - 1);
    //
    // /// define a 3 by 3 identity matrix
    // Matrix33 I = Matrix::identity(3, 3, 1.0);
    //
    // /// matrix-vector product
    // Vector3D v = M * Vector4D(1, 2, 3, 4); // M is 3 by 4
    //
    // Matrix U(m, m, 0.0);   // initialized with 0s
    // Matrix S(m, n, 0.0);   // initialized with 0s
    // Matrix V(n, n, 0.0);   // initialized with 0s
    //
    // // Compute the SVD decomposition of A
    // svd_decompose(A, U, S, V);
    //
    // // Now let's check if the SVD result is correct
    //
    // // Check 1: U is orthogonal, so U * U^T must be identity
    // std::cout << "U*U^T: \n" << U * transpose(U) << std::endl;
    //
    // // Check 2: V is orthogonal, so V * V^T must be identity
    // std::cout << "V*V^T: \n" << V * transpose(V) << std::endl;
    //
    // // Check 3: S must be a diagonal matrix
    // std::cout << "S: \n" << S << std::endl;
    //
    // // Check 4: according to the definition, A = U * S * V^T
    // std::cout << "M - U * S * V^T: \n" << A - U * S * transpose(V) << std::endl;
    //
    // // Compute the inverse of a matrix
    // Matrix invT;
    // inverse(T, invT);
    // // Let's check if the inverse is correct
    // std::cout << "T * invT: \n" << T * invT << std::endl;

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
    
    // 1. Calculate r₃ = ρ * a3
    Vector3D r3 = a3 * rho;

    // 2. Calculate r₁ = (a₂ × a₃) / ||a₂ × a₃||
    Vector3D r1 = cross(a2, a3);
    double norm_r1 = r1.length();
    if (norm_r1 < 1e-12) {
        std::cerr << "[Error] r1 degenerate.\n";
        return false;
    }
    r1 = r1 / norm_r1;

    // 3. Calculate r₂ = r₃ × r₁
    Vector3D r2 = cross(r3, r1);

    // 4. Obtain R: use r₁,r₂,r₃ as its row(R = [r₁ᵀ; r₂ᵀ; r₃ᵀ])
    Matrix33 R_extr;
    for (int j = 0; j < 3; j++) {
        R_extr(0, j) = r1[j];
        R_extr(1, j) = r2[j];
        R_extr(2, j) = r3[j];
    }

    // 5. Calculate K and K-1(using fx, fy, cx, cy, s)
    Matrix33 K_extr;
    K_extr(0, 0) = fx;  K_extr(0, 1) = s;   K_extr(0, 2) = cx;
    K_extr(1, 0) = 0;   K_extr(1, 1) = fy;  K_extr(1, 2) = cy;
    K_extr(2, 0) = 0;   K_extr(2, 1) = 0;   K_extr(2, 2) = 1;

    // As K is uptriangle matrix，its inverse:
    Matrix33 K_inv;
    K_inv(0, 0) = 1.0 / fx;
    K_inv(0, 1) = -s / (fx * fy);
    K_inv(0, 2) = (s * cy - cx * fy) / (fx * fy);
    K_inv(1, 0) = 0;
    K_inv(1, 1) = 1.0 / fy;
    K_inv(1, 2) = -cy / fy;
    K_inv(2, 0) = 0;
    K_inv(2, 1) = 0;
    K_inv(2, 2) = 1;

    // 6. Calculate t = ρ * K_inv * b
    Vector3D t_extr = rho * K_inv * b;

    R = R_extr;
    t = t_extr;
    

    // TODO: make sure the recovered parameters are passed to the corresponding variables (fx, fy, cx, cy, s, R, and t)

    std::cout << "\n\tTODO: After you implement this function, please return 'true' - this will trigger the viewer to\n"
                 "\t\tupdate the rendering using your recovered camera parameters. This can help you to visually check\n"
                 "\t\tif your calibration is successful or not.\n\n" << std::flush;
    return true;
}

