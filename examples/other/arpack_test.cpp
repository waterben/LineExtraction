#include <iostream>
#include "lsmatrxa.h"
#include <arpack++/arlsmat.h>
#include <arpack++/arlssym.h>
#include "lsymsol.h"

/*#if (_MSC_VER >= 1900)
#pragma comment(lib, "legacy_stdio_definitions.lib") // add legacy_stdio_definitions.lib for VS2015 because of define changes!!!
#endif*/


int main()
{

    // Defining variables;

    int     nx;
    int     n;          // Dimension of the problem.
    int     nnz;        // Number of nonzero elements in A.
    int*    irow;       // pointer to an array that stores the row
    // indices of the nonzeros in A.
    int*    pcol;       // pointer to an array of pointers to the
    // beginning of each column of A in vector A.
    double* A;          // pointer to an array that stores the
    // nonzero elements of A.

    // Creating a 100x100 matrix.

    nx = 10;
    SymmetricMatrixA(nx, n, nnz, A, irow, pcol);
    ARluSymMatrix<double> matrix(n, nnz, A, irow, pcol);

    // Defining what we need: the four eigenvectors of A with smallest magnitude.

    ARluSymStdEig<double> dprob(4, matrix, "SM");

    // Finding eigenvalues and eigenvectors.

    dprob.FindEigenvectors();

    // Printing solution.

    Solution(matrix, dprob);

    int in;
    std::cin >> in;
    return 0;

} // main.
