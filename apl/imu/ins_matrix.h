#ifndef __INS_MATRIX_H__
#define __INS_MATRIX_H__


#ifdef __cplusplus
extern "C" {
#endif


#include "ins_baseparams.h"

	/* new matrix ------------------------------------------------------------------
	* allocate memory of matrix 
	* args   : int    n,m       I   number of rows and columns of matrix
	* return : matrix pointer (if n<=0 or m<=0, return NULL)
	*-----------------------------------------------------------------------------*/
	double *ins_mat(int n, int m);

	/* new integer matrix ----------------------------------------------------------
	* allocate memory of integer matrix 
	* args   : int    n,m       I   number of rows and columns of matrix
	* return : matrix pointer (if n<=0 or m<=0, return NULL)
	*-----------------------------------------------------------------------------*/
	int *ins_imat(int n, int m);

	/* zero matrix -----------------------------------------------------------------
	* generate new zero matrix
	* args   : int    n,m       I   number of rows and columns of matrix
	* return : matrix pointer (if n<=0 or m<=0, return NULL)
	*-----------------------------------------------------------------------------*/
	double *ins_zeros(int n, int m);

	/* identity matrix -------------------------------------------------------------
	* generate new identity matrix
	* args   : int    n         I   number of rows and columns of matrix
	* return : matrix pointer (if n<=0, return NULL)
	*-----------------------------------------------------------------------------*/
	double *ins_mat_eye(int n);

	/* inner product ---------------------------------------------------------------
	* inner product of vectors
	* args   : double *a,*b     I   vector a,b (n x 1)
	*          int    n         I   size of vector a,b
	* return : a'*b
	*-----------------------------------------------------------------------------*/
	double ins_mat_dot(const double *a, const double *b, int n);

	/* euclid norm -----------------------------------------------------------------
	* euclid norm of vector
	* args   : double *a        I   vector a (n x 1)
	*          int    n         I   size of vector a
	* return : || a ||
	*-----------------------------------------------------------------------------*/
	double ins_mat_norm(const double *a, int n);

	/* outer product of 3d vectors -------------------------------------------------
	* outer product of 3d vectors 
	* args   : double *a,*b     I   vector a,b (3 x 1)
	*          double *c        O   outer product (a x b) (3 x 1)
	* return : none
	*-----------------------------------------------------------------------------*/
	void ins_mat_cross3(const double *a, const double *b, double *c);

	/* normalize 3d vector ---------------------------------------------------------
	* normalize 3d vector
	* args   : double *a        I   vector a (3 x 1)
	*          double *b        O   normlized vector (3 x 1) || b || = 1
	* return : status (1:ok,0:error)
	*-----------------------------------------------------------------------------*/
	int ins_mat_normv3(const double *a, double *b);

	/* copy matrix -----------------------------------------------------------------
	* copy matrix
	* args   : double *A        O   destination matrix A (n x m)
	*          double *B        I   source matrix B (n x m)
	*          int    n,m       I   number of rows and columns of matrix
	* return : none
	*-----------------------------------------------------------------------------*/
	void ins_matcpy(double *A, const double *B, int n, int m);


	/* multiply matrix (wrapper of blas dgemm) -------------------------------------
	* multiply matrix by matrix (C=alpha*A*B+beta*C)
	* args   : char   *tr       I  transpose flags ("N":normal,"T":transpose)
	*          int    n,k,m     I  size of (transposed) matrix A,B
	*          double alpha     I  alpha
	*          double *A,*B     I  (transposed) matrix A (n x m), B (m x k)
	*          double beta      I  beta
	*          double *C        IO matrix C (n x k)
	* return : none
	*-----------------------------------------------------------------------------*/
	void ins_matmul(const char *tr, int n, int k, int m, double alpha,
		const double *A, const double *B, double beta, double *C);

	/* inverse of matrix -----------------------------------------------------------
	* inverse of matrix (A=A^-1)
	* args   : double *A        IO  matrix (n x n)
	*          int    n         I   size of matrix A
	* return : status (0:ok,0>:error)
	*-----------------------------------------------------------------------------*/
	int ins_matinv(double *A, int n);

	/* solve linear equation -------------------------------------------------------
	* solve linear equation (X=A\Y or X=A'\Y)
	* args   : char   *tr       I   transpose flag ("N":normal,"T":transpose)
	*          double *A        I   input matrix A (n x n)
	*          double *Y        I   input matrix Y (n x m)
	*          int    n,m       I   size of matrix A,Y
	*          double *X        O   X=A\Y or X=A'\Y (n x m)
	* return : status (0:ok,0>:error)
	* notes  : matirix stored by column-major order (fortran convention)
	*          X can be same as Y
	*-----------------------------------------------------------------------------*/
	extern int ins_matsolve(const char *tr, const double *A, const double *Y, int n,
		int m, double *X);

	/* least square estimation -----------------------------------------------------
	* least square estimation by solving normal equation (x=(A*A')^-1*A*y)
	* args   : double *A        I   transpose of (weighted) design matrix (n x m)
	*          double *y        I   (weighted) measurements (m x 1)
	*          int    n,m       I   number of parameters and measurements (n<=m)
	*          double *x        O   estmated parameters (n x 1)
	*          double *Q        O   esimated parameters covariance matrix (n x n)
	* return : status (0:ok,0>:error)
	* notes  : for weighted least square, replace A and y by A*w and w*y (w=W^(1/2))
	*          matirix stored by column-major order (fortran convention)
	*-----------------------------------------------------------------------------*/
	int ins_mat_lsq(const double *A, const double *y, int n, int m, double *x,
		double *Q);

	/* kalman filter ---------------------------------------------------------------
	* kalman filter state update as follows:
	*
	*   K=P*H*(H'*P*H+R)^-1, xp=x+K*v, Pp=(I-K*H')*P
	*
	* args   : double *x        I   states vector (n x 1)
	*          double *P        I   covariance matrix of states (n x n)
	*          double *H        I   transpose of design matrix (n x m)
	*          double *v        I   innovation (measurement - model) (m x 1)
	*          double *R        I   covariance matrix of measurement error (m x m)
	*          int    n,m       I   number of states and measurements
	*          double *xp       O   states vector after update (n x 1)
	*          double *Pp       O   covariance matrix of states after update (n x n)
	* return : status (0:ok,<0:error)
	* notes  : matirix stored by column-major order (fortran convention)
	*          if state x[i]==0.0, not updates state x[i]/P[i+i*n]
	*-----------------------------------------------------------------------------*/
	static int ins_mat_filter_(const double *x, const double *P, const double *H,
		const double *v, const double *R, int n, int m,
		double *xp, double *Pp);

	int ins_mat_filter(double *x, double *P, const double *H, const double *v,
		const double *R, int n, int m);

	/* smoother --------------------------------------------------------------------
	* combine forward and backward filters by fixed-interval smoother as follows:
	*
	*   xs=Qs*(Qf^-1*xf+Qb^-1*xb), Qs=(Qf^-1+Qb^-1)^-1)
	*
	* args   : double *xf       I   forward solutions (n x 1)
	* args   : double *Qf       I   forward solutions covariance matrix (n x n)
	*          double *xb       I   backward solutions (n x 1)
	*          double *Qb       I   backward solutions covariance matrix (n x n)
	*          int    n         I   number of solutions
	*          double *xs       O   smoothed solutions (n x 1)
	*          double *Qs       O   smoothed solutions covariance matrix (n x n)
	* return : status (0:ok,0>:error)
	* notes  : see reference [4] 5.2
	*          matirix stored by column-major order (fortran convention)
	*-----------------------------------------------------------------------------*/
	int ins_mat_smoother(const double *xf, const double *Qf, const double *xb,
		const double *Qb, int n, double *xs, double *Qs);

	/* 
	* matrix stored by row-major order, change to col-major order
	*/
	void insMat_Rowmjr_to_Colmjr(double* A, int n, int m, double* TA);

    /* 
	* matrix stored by col-major order, change to row-major order
	*/
	void insMat_Colmjr_to_Rowmjr(double* A, int n, int m, double* TA);


	/*
	* the matrix A (n*m) stored in row major, transpose as TA also stored in row major.
	*/
	void insMat_Rowmjr_Transpose(double* A, int n, int m, double* TA);

	/*
	* the matrix A (n*m) stored in col major, transpose as TA also stored in col major.
	*/
	void insMat_Colmjr_Transpose(double* A, int n, int m, double* TA);

    /*
	* get item [i,j] of matrix A, stored in row major;
	*/
	double ins_getMatItem_RowMjr(double* A, int n, int m, int i, int j);

	/*
	* get item [i,j] of matrix A, stored in col major;
	*/
	double ins_getMatItem_ColMjr(double* A, int n, int m, int i, int j);



#ifdef __cplusplus
}
#endif

#endif
