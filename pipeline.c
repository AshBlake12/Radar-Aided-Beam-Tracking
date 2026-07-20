/* pipeline.c - PS chain: EVD + Root-MUSIC rooting + Kalman + steer
Currently I focused on PL, so i just asked to make a PS file, that can verify the loop, this works, we might wnat to improve upon this
and port to bare-metal on PS.
Important thing, is this works. 
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <complex.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define M 4
#define P 1
#define NC (2*M-1)
#define DEG (NC-1)
#define NTRI (M*(M+1)/2)          /* 10 */
#define SWEEPS 10
#define DT 0.1

/* ---- EVD: original verified complex Jacobi (tau rotation, double) ---- */
static void jacobi_herm(double complex A[M][M], double d[M],
                        double complex V[M][M]) {
    for (int i=0;i<M;i++) for (int j=0;j<M;j++) V[i][j] = (i==j);
    for (int it=0; it<SWEEPS; it++)
        for (int p=0;p<M-1;p++)
            for (int q=p+1;q<M;q++) {
                double complex apq = A[p][q];
                double mag = cabs(apq);
                if (mag < 1e-20) continue;
                double app = creal(A[p][p]), aqq = creal(A[q][q]);
                double complex phi = apq/mag;
                double tau = (aqq-app)/(2*mag);
                double sg = (tau>=0) ? 1.0 : -1.0;
                double t = sg/(fabs(tau)+sqrt(tau*tau+1));
                double c = 1.0/sqrt(1+t*t), s = c*t;
                /* apply G'AG and VG on rows/cols p,q only (G differs from I there) */
                double complex gpq = s*phi, gqp = -s*conj(phi);
                for (int k=0;k<M;k++) {            /* A <- A*G (cols p,q) */
                    double complex ap=A[k][p], aq=A[k][q];
                    A[k][p]=ap*c+aq*gqp; A[k][q]=ap*gpq+aq*c;
                }
                for (int k=0;k<M;k++) {            /* A <- G'*A (rows p,q) */
                    double complex ap=A[p][k], aq=A[q][k];
                    A[p][k]=c*ap+conj(gqp)*aq; A[q][k]=conj(gpq)*ap+c*aq;
                }
                for (int k=0;k<M;k++) {            /* V <- V*G */
                    double complex vp=V[k][p], vq=V[k][q];
                    V[k][p]=vp*c+vq*gqp; V[k][q]=vp*gpq+vq*c;
                }
            }
    for (int i=0;i<M;i++) d[i] = creal(A[i][i]);
}

/* R triangle -> Root-MUSIC polynomial coeffs (ascending power, len NC) */
static void tri_to_coeffs(const double *tri, double complex *coeff) {
    double complex A[M][M], V[M][M]; double d[M];
    int idx=0;
    for (int i=0;i<M;i++) for (int j=i;j<M;j++) {
        A[i][j] = tri[2*idx] + tri[2*idx+1]*I;
        if (j>i) A[j][i] = conj(A[i][j]);
        idx++;
    }
    jacobi_herm(A, d, V);
    int ord[M]; for (int i=0;i<M;i++) ord[i]=i;      /* sort ascending */
    for (int i=0;i<M-1;i++) for (int j=i+1;j<M;j++)
        if (d[ord[j]] < d[ord[i]]) { int t=ord[i]; ord[i]=ord[j]; ord[j]=t; }
    double complex C[M][M] = {0};                     /* En*En', En = M-P smallest */
    for (int e=0;e<M-P;e++) {
        int col = ord[e];
        for (int i=0;i<M;i++) for (int j=0;j<M;j++)
            C[i][j] += V[i][col]*conj(V[j][col]);
    }
    for (int l=-(M-1); l<=M-1; l++) {
        double complex s = 0;
        for (int m=0;m<M;m++) { int n=m+l; if (n>=0 && n<M) s += C[m][n]; }
        coeff[l+M-1] = s;
    }
}

/* ---- rooting: Durand-Kerner (unchanged, previously verified) ---- */
static int root_near_unit(const double complex *a, double complex *out) {
    double complex r[DEG];
    for (int i=0;i<DEG;i++) r[i]=cpow(0.4+0.9*I,i);
    for (int it=0; it<80; it++)
        for (int i=0;i<DEG;i++){
            double complex num=a[NC-1], den=1;
            for (int k=NC-2;k>=0;k--) num=num*r[i]+a[k];
            for (int j=0;j<DEG;j++) if(j!=i) den*=r[i]-r[j];
            r[i]-=num/den;
        }
    int best=-1; double bd=1e9;
    for (int i=0;i<DEG;i++){ double m=cabs(r[i]);
        if (m<1.0 && fabs(m-1.0)<bd){bd=fabs(m-1.0);best=i;} }
    if (best<0) return 0;
    *out=r[best]; return 1;
}

int main(int argc, char **argv) {
    FILE *fp = fopen(argc>1?argv[1]:"rframes.csv","r");
    if (!fp) { perror("open"); return 1; }
    double th=0,vt=0,P00=1,P01=0,P10=0,P11=1;
    const double Q0=1e-2,Q1=8.0,Rk=0.05;
    int init=0; char line[4096];
    while (fgets(line,sizeof line,fp)) {
        /* HW SWAP POINT: on the board read axi_regs 0x10..0x34 here
         * (10 regs, {im[31:16],re[15:0]} Q1.15 each) instead of CSV. */
        double v[2*NTRI]; int n=0; char *p=line;
        while (n<2*NTRI) { v[n++]=strtod(p,&p); if(*p==',')p++; }
        double complex a[NC], z;
        tri_to_coeffs(v, a);
        double complex lead = a[NC-1];
        for (int i=0;i<NC;i++) a[i] /= lead;
        if (!root_near_unit(a,&z)) continue;
        double ph=carg(z)/M_PI; if(ph>1)ph=1; if(ph<-1)ph=-1;
        double meas=asin(ph)*180.0/M_PI;
        if (!init){th=meas;vt=0;init=1;}
        double thp=th+DT*vt;
        double Pp00=P00+DT*(P10+P01)+DT*DT*P11+Q0;
        double Pp01=P01+DT*P11, Pp10=P10+DT*P11, Pp11=P11+Q1;
        double S=Pp00+Rk, K0=Pp00/S, K1=Pp10/S, y=meas-thp;
        th=thp+K0*y; vt=vt+K1*y;
        P00=(1-K0)*Pp00; P01=(1-K0)*Pp01;
        P10=Pp10-K1*Pp00; P11=Pp11-K1*Pp01;
        double thn=th+DT*vt, w=M_PI*sin(thn*M_PI/180.0);
        printf("meas=%9.5f pred=%9.5f weight=%+.4f%+.4fj\n",
               meas, thn, cos(w), sin(w));
    }
    fclose(fp); return 0;
}