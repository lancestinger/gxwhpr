#include "ins_matrix.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>


static int ludcmp(double *A, int n, int *indx, double *d);
static void lubksb(const double *A, int n, const int *indx, double *b);

static void ins_fatalerr(const char *format, ...);

/* new matrix ------------------------------------------------------------------
* allocate memory of matrix 
* args   : int    n,m       I   number of rows and columns of matrix
* return : matrix pointer (if n<=0 or m<=0, return NULL)
*-----------------------------------------------------------------------------*/
double *ins_mat(int n, int m)
{
	double *p;

	if (n<=0||m<=0) return NULL;
	if (!(p=(double *)malloc(sizeof(double)*n*m))) {
		ins_fatalerr("matrix memory allocation error: n=%d,m=%d\n",n,m);
	}
	return p;
}

/* new integer matrix ----------------------------------------------------------
* allocate memory of integer matrix 
* args   : int    n,m       I   number of rows and columns of matrix
* return : matrix pointer (if n<=0 or m<=0, return NULL)
*-----------------------------------------------------------------------------*/
int *ins_imat(int n, int m)
{
	int *p;

	if (n<=0||m<=0) return NULL;
	if (!(p=(int *)malloc(sizeof(int)*n*m))) {
		ins_fatalerr("integer matrix memory allocation error: n=%d,m=%d\n",n,m);
	}
	return p;
}

/* zero matrix -----------------------------------------------------------------
* generate new zero matrix
* args   : int    n,m       I   number of rows and columns of matrix
* return : matrix pointer (if n<=0 or m<=0, return NULL)
*-----------------------------------------------------------------------------*/
double *ins_zeros(int n, int m)
{
	double *p;

#ifdef NOCALLOC
	if ((p=ins_mat(n,m))) for (n=n*m-1;n>=0;n--) p[n]=0.0;
#else
	if (n<=0||m<=0) return NULL;
	if (!(p=(double *)calloc(sizeof(double),n*m))) {
		ins_fatalerr("matrix memory allocation error: n=%d,m=%d\n",n,m);
	}
#endif
	return p;
}

/* identity matrix -------------------------------------------------------------
* generate new identity matrix
* args   : int    n         I   number of rows and columns of matrix
* return : matrix pointer (if n<=0, return NULL)
*-----------------------------------------------------------------------------*/
double *ins_mat_eye(int n)
{
	double *p;
	int i;

	if ((p=ins_zeros(n,n))) for (i=0;i<n;i++) p[i+i*n]=1.0;
	return p;
}

/* inner product ---------------------------------------------------------------
* inner product of vectors
* args   : double *a,*b     I   vector a,b (n x 1)
*          int    n         I   size of vector a,b
* return : a'*b
*-----------------------------------------------------------------------------*/
double ins_mat_dot(const double *a, const double *b, int n)
{
	double c=0.0;

	while (--n>=0) c+=a[n]*b[n];
	return c;
}

/* euclid norm -----------------------------------------------------------------
* euclid norm of vector
* args   : double *a        I   vector a (n x 1)
*          int    n         I   size of vector a
* return : || a ||
*-----------------------------------------------------------------------------*/
double ins_mat_norm(const double *a, int n)
{
	return sqrt(ins_mat_dot(a,a,n));
}

/* outer product of 3d vectors -------------------------------------------------
* outer product of 3d vectors 
* args   : double *a,*b     I   vector a,b (3 x 1)
*          double *c        O   outer product (a x b) (3 x 1)
* return : none
*-----------------------------------------------------------------------------*/
void ins_mat_cross3(const double *a, const double *b, double *c)
{
	c[0]=a[1]*b[2]-a[2]*b[1];
	c[1]=a[2]*b[0]-a[0]*b[2];
	c[2]=a[0]*b[1]-a[1]*b[0];
}

/* normalize 3d vector ---------------------------------------------------------
* normalize 3d vector
* args   : double *a        I   vector a (3 x 1)
*          double *b        O   normlized vector (3 x 1) || b || = 1
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
int ins_mat_normv3(const double *a, double *b)
{
	double r;
	if ((r=ins_mat_norm(a,3))<=0.0) return 0;
	b[0]=a[0]/r;
	b[1]=a[1]/r;
	b[2]=a[2]/r;
	return 1;
}

/* copy matrix -----------------------------------------------------------------
* copy matrix
* args   : double *A        O   destination matrix A (n x m)
*          double *B        I   source matrix B (n x m)
*          int    n,m       I   number of rows and columns of matrix
* return : none
*-----------------------------------------------------------------------------*/
void ins_matcpy(double *A, const double *B, int n, int m)
{
	memcpy(A,B,sizeof(double)*n*m);
}


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
				   const double *A, const double *B, double beta, double *C)
{
	double d;
	int i,j,x,f=tr[0]=='N'?(tr[1]=='N'?1:2):(tr[1]=='N'?3:4);

	for (i=0;i<n;i++) for (j=0;j<k;j++) {
		d=0.0;
		switch (f) {
			case 1: for (x=0;x<m;x++) d+=A[i+x*n]*B[x+j*m]; break;
			case 2: for (x=0;x<m;x++) d+=A[i+x*n]*B[j+x*k]; break;
			case 3: for (x=0;x<m;x++) d+=A[x+i*m]*B[x+j*m]; break;
			case 4: for (x=0;x<m;x++) d+=A[x+i*m]*B[j+x*k]; break;
		}
		if (beta==0.0 || fabs(beta)<1.0e-6) C[i+j*n]=alpha*d; else C[i+j*n]=alpha*d+beta*C[i+j*n];
	}
}

/* inverse of matrix -----------------------------------------------------------
* inverse of matrix (A=A^-1)
* args   : double *A        IO  matrix (n x n)
*          int    n         I   size of matrix A
* return : status (0:ok,0>:error)
*-----------------------------------------------------------------------------*/
int ins_matinv(double *A, int n)
{
	double d,*B;
	int i,j,*indx;

	indx=ins_imat(n,1); B=ins_mat(n,n); ins_matcpy(B,A,n,n);
	if (ludcmp(B,n,indx,&d)) {free(indx); free(B); return -1;}
	for (j=0;j<n;j++) {
		for (i=0;i<n;i++) A[i+j*n]=0.0;
		A[j+j*n]=1.0;
		lubksb(B,n,indx,A+j*n);
	}
	free(indx); free(B);
	return 0;
}

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
int ins_matsolve(const char *tr, const double *A, const double *Y, int n,
				 int m, double *X)
{
	double *B=ins_mat(n,n);
	int info;

	ins_matcpy(B,A,n,n);
	if (!(info=ins_matinv(B,n))) ins_matmul(tr[0]=='N'?"NN":"TN",n,m,n,1.0,B,Y,0.0,X);
	free(B);
	return info;
}

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
			   double *Q)
{
	double *Ay;
	int info;

	if (m<n) return -1;
	Ay=ins_mat(n,1);
	ins_matmul("NN",n,1,m,1.0,A,y,0.0,Ay); /* Ay=A*y */
	ins_matmul("NT",n,n,m,1.0,A,A,0.0,Q);  /* Q=A*A' */
	if (!(info=ins_matinv(Q,n))) ins_matmul("NN",n,1,n,1.0,Q,Ay,0.0,x); /* x=Q^-1*Ay */
	free(Ay);
	return info;
}
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
int ins_mat_filter_(const double *x, const double *P, const double *H,
				   const double *v, const double *R, int n, int m,
				   double *xp, double *Pp)
{
	double *F=ins_mat(n,m),*Q=ins_mat(m,m),*K=ins_mat(n,m),*I=ins_mat_eye(n);
	int info;

	ins_matcpy(Q,R,m,m);
	ins_matcpy(xp,x,n,1);
	ins_matmul("NN",n,m,n,1.0,P,H,0.0,F);       /* Q=H'*P*H+R */
	ins_matmul("TN",m,m,n,1.0,H,F,1.0,Q);
	if (!(info=ins_matinv(Q,m))) {
		ins_matmul("NN",n,m,m,1.0,F,Q,0.0,K);   /* K=P*H*Q^-1 */
		ins_matmul("NN",n,1,m,1.0,K,v,1.0,xp);  /* xp=x+K*v */
		ins_matmul("NT",n,n,m,-1.0,K,H,1.0,I);  /* Pp=(I-K*H')*P */
		ins_matmul("NN",n,n,n,1.0,I,P,0.0,Pp);
	}
	free(F); free(Q); free(K); free(I);
	return info;
}

int ins_mat_filter(double *x, double *P, const double *H, const double *v,
				  const double *R, int n, int m)
{
	double *x_,*xp_,*P_,*Pp_,*H_;
	int i,j,k,info,*ix;

	ix=ins_imat(n,1); for (i=k=0;i<n;i++) if (x[i]!=0.0&&P[i+i*n]>0.0) ix[k++]=i;
	x_=ins_mat(k,1); xp_=ins_mat(k,1); P_=ins_mat(k,k); Pp_=ins_mat(k,k); H_=ins_mat(k,m);
	for (i=0;i<k;i++) {
		x_[i]=x[ix[i]];
		for (j=0;j<k;j++) P_[i+j*k]=P[ix[i]+ix[j]*n];
		for (j=0;j<m;j++) H_[i+j*k]=H[ix[i]+j*n];
	}
	info=ins_mat_filter_(x_,P_,H_,v,R,k,m,xp_,Pp_);
	for (i=0;i<k;i++) {
		x[ix[i]]=xp_[i];
		for (j=0;j<k;j++) P[ix[i]+ix[j]*n]=Pp_[i+j*k];
	}
	free(ix); free(x_); free(xp_); free(P_); free(Pp_); free(H_);
	return info;
}

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
					const double *Qb, int n, double *xs, double *Qs)
{
	double *invQf=ins_mat(n,n),*invQb=ins_mat(n,n),*xx=ins_mat(n,1);
	int i,info=-1;

	ins_matcpy(invQf,Qf,n,n);
	ins_matcpy(invQb,Qb,n,n);
	if (!ins_matinv(invQf,n)&&!ins_matinv(invQb,n)) {
		for (i=0;i<n*n;i++) Qs[i]=invQf[i]+invQb[i];
		if (!(info=ins_matinv(Qs,n))) {
			ins_matmul("NN",n,1,n,1.0,invQf,xf,0.0,xx);
			ins_matmul("NN",n,1,n,1.0,invQb,xb,1.0,xx);
			ins_matmul("NN",n,1,n,1.0,Qs,xx,0.0,xs);
		}
	}
	free(invQf); free(invQb); free(xx);
	return info;
}


/* LU decomposition ----------------------------------------------------------*/
static int ludcmp(double *A, int n, int *indx, double *d)
{
	double big,s,tmp,*vv=ins_mat(n,1);
	int i,imax=0,j,k;

	*d=1.0;
	for (i=0;i<n;i++) {
		big=0.0; for (j=0;j<n;j++) if ((tmp=fabs(A[i+j*n]))>big) big=tmp;
		if (big>0.0) vv[i]=1.0/big; else {free(vv); return -1;}
	}
	for (j=0;j<n;j++) {
		for (i=0;i<j;i++) {
			s=A[i+j*n]; for (k=0;k<i;k++) s-=A[i+k*n]*A[k+j*n]; A[i+j*n]=s;
		}
		big=0.0;
		for (i=j;i<n;i++) {
			s=A[i+j*n]; for (k=0;k<j;k++) s-=A[i+k*n]*A[k+j*n]; A[i+j*n]=s;
			if ((tmp=vv[i]*fabs(s))>=big) {big=tmp; imax=i;}
		}
		if (j!=imax) {
			for (k=0;k<n;k++) {
				tmp=A[imax+k*n]; A[imax+k*n]=A[j+k*n]; A[j+k*n]=tmp;
			}
			*d=-(*d); vv[imax]=vv[j];
		}
		indx[j]=imax;
		if (A[j+j*n]==0.0) {free(vv); return -1;}
		if (j!=n-1) {
			tmp=1.0/A[j+j*n]; for (i=j+1;i<n;i++) A[i+j*n]*=tmp;
		}
	}
	free(vv);
	return 0;
}

/* LU back-substitution ------------------------------------------------------*/
static void lubksb(const double *A, int n, const int *indx, double *b)
{
	double s;
	int i,ii=-1,ip,j;

	for (i=0;i<n;i++) {
		ip=indx[i]; s=b[ip]; b[ip]=b[i];
		if (ii>=0) for (j=ii;j<i;j++) s-=A[i+j*n]*b[j]; else if (s) ii=i;
		b[i]=s;
	}
	for (i=n-1;i>=0;i--) {
		s=b[i]; for (j=i+1;j<n;j++) s-=A[i+j*n]*b[j]; b[i]=s/A[i+i*n];
	}
}

/* 
* matrix stored by row-major order, change to col-major order
*/
void insMat_Rowmjr_to_Colmjr(double* A, int n, int m, double* TA)
{
	int i, j;
	int indx1, indx2;

	for (i=0; i<n; i++)
	{
		for (j=0; j<m; j++)
		{
			indx1 = i + j*n;
			indx2 = i*m + j;
			TA[indx1] = A[indx2];
		}
	}

}

/* 
* matrix stored by col-major order, change to row-major order
*/
void insMat_Colmjr_to_Rowmjr(double* A, int n, int m, double* TA)
{
	int i, j;
	int indx1, indx2;

	for (i=0; i<n; i++)
	{
		for (j=0; j<m; j++)
		{
			indx1 = i + j*n;
			indx2 = i*m + j;
			TA[indx2] = A[indx1];
		}
	}

}

/*
* the matrix A (n*m) stored in row major, transpose as TA also stored in row major.
*/
void insMat_Rowmjr_Transpose(double* A, int n, int m, double* TA)
{
	int i, j;
	int indx1, indx2;

	for (i=0; i<n; i++)
	{
		for (j=0; j<m; j++)
		{
			indx1 = i + j*n;
			indx2 = i*m + j;
			TA[indx1] = A[indx2];
		}
	}

}

/*
* the matrix A (n*m) stored in col major, transpose as TA also stored in col major.
*/
void insMat_Colmjr_Transpose(double* A, int n, int m, double* TA)
{
	int i, j;
	int indx1, indx2;

	for (i=0; i<n; i++)
	{
		for (j=0; j<m; j++)
		{
			indx1 = i + j*n;
			indx2 = i*m + j;
			TA[indx2] = A[indx1];
		}
	}

}

/*
* get item [i,j] of matrix A, stored in row major;
*/
double ins_getMatItem_RowMjr(double* A, int n, int m, int i, int j)
{
	int indx = i * m + j;

	return A[indx];
}

/*
* get item [i,j] of matrix A, stored in col major;
*/
double ins_getMatItem_ColMjr(double* A, int n, int m, int i, int j)
{
	int indx = i + j * n;

	return A[indx];
}


/* fatal error ---------------------------------------------------------------*/
static void ins_fatalerr(const char *format, ...)
{
	/*
	char msg[1024];
	va_list ap;
	va_start(ap,format); vsprintf(msg,format,ap); va_end(ap);
	//if (fatalfunc) fatalfunc(msg);
	//else fprintf(stderr,"%s",msg);
	*/

	//fprintf(stderr,"%s",msg);

	exit(-9);
}
