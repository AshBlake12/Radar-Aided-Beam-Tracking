
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <complex.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define M 4                        /* radar azimuth virtual elements   */
#define NC (2*M-1)                 /* coeffs (ascending power)          */
#define DEG (2*M-2)                /* polynomial degree                */
#define DT 0.1                     /* frame period + pipeline lookahead */

static int cabs_root_near1(double complex *a, double complex *out) {
    double complex r[DEG]; for (int i=0;i<DEG;i++) r[i]=cpow(0.4+0.9*I,i);

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

int main(int argc,char**argv){
    FILE*fp=fopen(argc>1?argv[1]:"frames.csv","r");
    if(!fp){perror("open");return 1;}
    /* const-velocity Kalman: x=[theta,thetadot] */
    double th=0,vt=0,P00=1,P01=0,P10=0,P11=1,Q0=1e-2,Q1=8.0,Rk=0.05;
    int init=0; char line[1024];
    while(fgets(line,sizeof line,fp)){
        double v[2*NC]; int n=0; char*p=line;
        while(n<2*NC){ v[n++]=strtod(p,&p); if(*p==',')p++; }

        double complex a[NC];
        // for(int i=0;i<NC;i++) a[i]=v[i]+v[NC+i]*I;      /* re[],im[] */
        
        for(int i=0;i<NC;i++) {
            a[i] = v[2*i] + v[2*i+1]*I; 
        }

        double complex lead = a[NC-1];
        for(int i=0;i<NC;i++) {
            a[i] /= lead;
        }

        double complex z; double meas;
        if(!cabs_root_near1(a,&z)) continue;
        double ph=carg(z)/M_PI; if(ph>1)ph=1; if(ph<-1)ph=-1;
        meas=asin(ph)*180.0/M_PI;                       /* measured azimuth */
        if(!init){th=meas;vt=0;init=1;}
        /* predict */
        double thp=th+DT*vt, vtp=vt;
        double Pp00=P00+DT*(P10+P01)+DT*DT*P11+Q0;
        double Pp01=P01+DT*P11, Pp10=P10+DT*P11, Pp11=P11+Q1;
        /* update */
        double S=Pp00+Rk, K0=Pp00/S, K1=Pp10/S, y=meas-thp;
        th=thp+K0*y; vt=vtp+K1*y;
        P00=(1-K0)*Pp00; P01=(1-K0)*Pp01;
        P10=Pp10-K1*Pp00; P11=Pp11-K1*Pp01;
        /* steer to NEXT frame: theta + DT*vt */
        double thn=th+DT*vt, w=M_PI*sin(thn*M_PI/180.0);
        printf("meas=%7.3f  pred=%7.3f  weight=%+.4f%+.4fj\n",
               meas, thn, cos(w), sin(w));
    }
    fclose(fp); return 0;
}