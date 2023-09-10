#include <iostream>
#include <cmath>
#include <complex>
#define _USE_MATH_DEFINES
#include <math.h>
#define PI           3.14159265358979323846  /* pi */
#define d_t 0.01 // step size
#define Tt 20  // Final time for simulation


#include <iostream>
#include <fstream>
#include <time.h>

#include <Eigen>
#include <Dense>

using namespace Eigen;

using namespace std;


double det(Matrix3d M){
// determinant 3x3 matrix
double det = M(0,0)*(M(1,1)*M(2,2)-M(2,1)*M(1,2)) - M(0,1)*(M(1,0)*M(2,2)-M(2,0)*M(1,2)) + M(0,2)*(M(1,0)*M(2,1)-M(2,0)*M(1,1));
return det;
}

Matrix3d InvMat(Matrix3d M){
// inverse 3x3 matrix
    double d = det(M);
    Matrix3d invM;

    invM(0,0) = (M(1,1)*M(2,2)-M(2,1)*M(1,2))/d;
    invM(0,1) = -(M(0,1)*M(2,2)-M(0,2)*M(2,1))/d;
    invM(0,2) = (M(0,1)*M(1,2)-M(0,2)*M(1,1))/d;
    invM(1,0) = -(M(1,0)*M(2,2)-M(1,2)*M(2,0))/d;
    invM(1,1) = (M(0,0)*M(2,2)-M(0,2)*M(2,0))/d;
    invM(1,2) = -(M(0,0)*M(1,2)-M(0,2)*M(1,0))/d;
    invM(2,0) = (M(1,0)*M(2,1)-M(1,1)*M(2,0))/d;
    invM(2,1) = -(M(0,0)*M(2,1)-M(0,1)*M(2,0))/d;
    invM(2,2) = (M(0,0)*M(1,1)-M(0,1)*M(1,0))/d;
    return invM;

}

Matrix3d skew(Vector3d w) {
     Matrix3d sw;
     sw(0,0)= 0; sw(0,1)= -w[2];sw(0,2)= w[1];
     sw(1,0)= w[2]; sw(1,1)= 0;sw(1,2)= -w[0];
     sw(2,0)= -w[1]; sw(2,1)= w[0];sw(2,2)= 0;
     return sw;
     /*
    {{ 0, -w[2], w[1] },
     { w[2], 0, -w[0] },
     { -w[1], w[0], 0 }}
     */
}

Vector3d unskew(Matrix3d sw){
    Vector3d ww;
    ww[0] = -sw(1,2);
    ww[1] = sw(0,2);
    ww[2] = -sw(0,1);
    return ww;
    // w=[-Sw(2,3) Sw(1,3) -Sw(1,2)].';
}

Matrix3d expmso3(Vector3d r) {

    Matrix3d R;

    double nr = sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
    // nr=norm(r);
    if (nr==0) {
        R = Matrix3d::Identity();
    }
    // if nr==0;
    // R=eye(3);
    else {

        Matrix3d Sr ;
        Sr <<   0, -r[2], r[1],
                r[2], 0, -r[0],
                -r[1], r[0], 0;


        Matrix3d Sr2 = Sr*Sr;

        Matrix3d I3 = Matrix3d::Identity();

        for(int i = 0; i < 3; ++i)
        {
            for(int j = 0; j < 3; ++j)
            {
                R(i,j) = I3(i,j) + sin(nr)/nr*Sr(i,j) + 2*pow(sin(0.5*nr),2)/pow(nr,2)*Sr2(i,j);
            }
        }
    }
    // else
    // Sr=[0 -r(3) r(2);r(3) 0 -r(1);-r(2) r(1) 0];
    // R=eye(3)+sin(nr)/nr*Sr+2*sin(0.5*nr)^2/(nr)^2*Sr^2;
    return R;
}

double normfro(Matrix3d mat) {

    Matrix3d matTrans = mat.transpose();
    Matrix3d mat2 = matTrans*mat;
    double norm = sqrt(mat2(0,0)+mat2(1,1)+mat2(2,2));
    // sqrt (sum (diag (A' * A)))
    return norm;
}

MatrixXd divideMat(MatrixXd a, MatrixXd b){
    MatrixXd c = a*b.inverse();
return c;
}

Vector3d logmso3(Matrix3d R){
    Vector3d r;
    double trR = R.trace();
    //trR=trace(R);
    complex<double> phi = acos(complex<double> (0.5*(trR-1),0));
    //phi=acos(0.5*(trR-1));
    Matrix3d Rtrans = R.transpose();
    Matrix3d A = R-Rtrans;
    //A=R-R';
    double nA = normfro(A);
    //nA=norm(A,'fro');
    Matrix3d I3 = Matrix3d::Identity();
    Matrix3d S = 0.5*(R-I3);
    //S=0.5*(R-eye(3));
    double nS = normfro(S);
    //nS=norm(S,'fro');
    Vector3d sd = S.diagonal();
    //sd=diag(S);
    double msd = sd[0];
    int i=0;

    for (int j=0; j<3 ;j++){
        if (msd<=sd[j]){
            msd = sd[j];
            i=j+1;
        }
    }
    // [msd,i]=max(sd);

    if (nA < 0.00001 && nS < 0.00001) {
        r[0]=0;r[1]=0;r[2]=0;
    }
    //if nA < 1e-5 && nS < 1e-5, % This is when R is close to identity (phi is even multiple of pi rads)
    //    r=zeros(3,1);
    else if (nA < 0.00001 && nS > 0.1) {
    // elseif nA < 1e-5 && nS > 0.1, % This is when phi is an odd multiple of pi rads (logmso3 "strictly" singular)
        double ri = sqrt(1+msd);
        // ri=sqrt(1+msd);
        if (i==1){
            r[0]=PI*ri;
            r[1]=PI*S(0,1)/ri;
            r[2]=PI*S(0,2)/ri;
            // r=pi*[ri;S(1,2)/ri;S(1,3)/ri];
        }
        else if (i==2){
            r[0]=PI*S(1,0)/ri;
            r[1]=PI*ri;
            r[2]=PI*S(1,2)/ri;
            // r=pi*[S(2,1)/ri;ri;S(2,3)/ri];
        }
        else {
            r[0]=PI*S(2,0)/ri;
            r[1]=PI*S(2,1)/ri;
            r[2]=PI*ri;
            // r=pi*[S(3,1)/ri;S(3,2)/ri;ri];
        }
    }
    else {
         complex<double> y (0.5,0);
        if (abs(phi)< 0.2) {
            y = y + (complex <double> (1,0))*(std::pow(phi,2))/(complex <double> (12,0));
            y = y + (complex <double> (7,0))*(std::pow(phi,4))/(complex <double> (720,0));
            y = y + (complex <double> (31,0))*(std::pow(phi,6))/(complex <double> (30240,0));
            y = y + (complex <double> (127,0))*(std::pow(phi,8))/(complex <double> (1209600,0));
            y = y + (complex <double> (73,0))*(std::pow(phi,10))/(complex <double> (6842880,0));
            y = y + (complex <double> (1414477,0))*(std::pow(phi,12))/(complex <double> (1307674368000,0));
            // y=1/2+x^2/12+(7*x^4)/720+(31*x^6)/30240+(127*x^8)/1209600+(73*x^10)/6842880+(1414477*x^12)/1307674368000;
        }
        else {
            y = (complex <double> (0.5,0))*phi/sin(phi);
            // y=0.5*x/sin(x);
        }

        double yy = y.real();
        Matrix3d logR = yy*A;
        // logR=cs(phi)*(R-R');
        r = unskew(logR);
        // r=unskew(logR);
    }
    return r;
}

Matrix4d R6tose3(VectorXd eta){
    // % R6tose3: Transforms a vector eta in R^6 to the square 4x4
    // %          matrix representation of se3

    Vector3d eta123;
    eta123 << eta(0),eta(1),eta(2);
    Vector3d eta456;
    eta456 << eta(3),eta(4),eta(5);
    Matrix4d Eta;
    Eta << skew(eta123),eta456,
          0,0,0,0;
          // Eta=[skew(eta(1:3)) eta(4:6); zeros(1,4)];
    return Eta;
}

MatrixXd adse3s( VectorXd zeta){
    // % adse3: Computes the matrix representation of ad_z where z is in se(3) (z
    // % represented in R^6)
    // % Created: June 23, 2012, AKS

    Vector3d Theta;
    Theta<< zeta(0),zeta(1),zeta(2);
    // Theta=zeta(1:3);
    Vector3d beta;
    beta<< zeta(3),zeta(4),zeta(5);
    // beta=zeta(4:6);
    Matrix3d zero3 = MatrixXd::Zero(3,3);
    MatrixXd adz(6,6);
    adz << skew(Theta), zero3, skew(beta), skew(Theta);
    // adz=[skew(Theta) zeros(3,3); skew(beta) skew(Theta)];
    return adz;
}

void Dynamics(float t[int(Tt/d_t)],MatrixXd II,Matrix4d g0,VectorXd xi0,Matrix4d g[int(Tt/d_t)],VectorXd xi[int(Tt/d_t)]) {

float dt = d_t;
// dt=t(2)-t(1);
Matrix4d g_0 = MatrixXd::Zero(4,4);
    for (int i = 0; i<int(Tt/d_t);i++){
        g[i] = g_0;
    }
// g=zeros(4,4,size(t,2));
g[0] = g0;
// g(1:4,1:4,1)=g0;
Matrix4d gdot_0 = MatrixXd::Zero(4,4);
Matrix4d gdot[int(Tt/d_t)];
    for (int i = 0; i<int(Tt/d_t);i++){
        gdot[i] = gdot_0;
    }
// gdot=zeros(4,4,size(t,2));
Matrix4d g12_0 = MatrixXd::Zero(4,4);
Matrix4d g12[int(Tt/d_t)];
    for (int i = 0; i<int(Tt/d_t);i++){
        g12[i] = g12_0;
    }
// g12=zeros(4,4,size(t,2));
Matrix4d gdot12_0 = MatrixXd::Zero(4,4);
Matrix4d gdot12[int(Tt/d_t)];
    for (int i = 0; i<int(Tt/d_t);i++){
        gdot12[i] = gdot12_0;
    }
// gdot12=zeros(4,4,size(t,2));
VectorXd xi_0 = VectorXd::Zero(6);
    for (int i = 0; i<int(Tt/d_t);i++){
        xi[i] = xi_0;
    }
// xi=zeros(6,size(t,2));
xi[0] = xi0;
// xi(1:6,1)=xi0;
VectorXd xidot_0 = VectorXd::Zero(6);
VectorXd xidot[int(Tt/d_t)];
    for (int i = 0; i<int(Tt/d_t);i++){
        xidot[i] = xidot_0;
    }
// xidot=zeros(6,size(t,2));
VectorXd xi12_0 = VectorXd::Zero(6);
VectorXd xi12[int(Tt/d_t)];
    for (int i = 0; i<int(Tt/d_t);i++){
        xi12[i] = xi12_0;
    }
// xi12=zeros(6,size(t,2));
VectorXd xidot12_0 = VectorXd::Zero(6);
VectorXd xidot12[int(Tt/d_t)];
    for (int i = 0; i<int(Tt/d_t);i++){
        xidot12[i] = xidot12_0;
    }
// xidot12=zeros(6,size(t,2));
VectorXd phi_0 = VectorXd::Zero(3);
VectorXd phi[int(Tt/d_t)];
    for (int i = 0; i<int(Tt/d_t);i++){
        phi[i] = phi_0;
    }
// phi=zeros(3,size(t,2));
VectorXd phi12_0 = VectorXd::Zero(3);
VectorXd phi12[int(Tt/d_t)];
    for (int i = 0; i<int(Tt/d_t);i++){
        phi12[i] = phi12_0;
    }
// phi12=zeros(3,size(t,2));
VectorXd tau_0 = VectorXd::Zero(3);
VectorXd tau[int(Tt/d_t)];
    for (int i = 0; i<int(Tt/d_t);i++){
        tau[i] = tau_0;
    }
// tau=zeros(3,size(t,2));
VectorXd tau12_0 = VectorXd::Zero(3);
VectorXd tau12[int(Tt/d_t)];
    for (int i = 0; i<int(Tt/d_t);i++){
        tau12[i] = tau12_0;
    }
// tau12=zeros(3,size(t,2));

    for (int i=0;i<int(Tt/d_t)-1;i++){
        gdot[i]=g[i]*R6tose3(xi[i]);
        // gdot(1:4,1:4,i)=g(1:4,1:4,i)*R6tose3(xi(1:6,i));
        phi[i] << pow(10,-9)*10*cos(0.1*t[i]), pow(10,-9)*2*sin(0.2*t[i]), -pow(10,-9)*2*sin(0.5*t[i]);
        // phi(1:3,i)=1e-9*[10*cos(.1*t(i));2*sin(.2*t(i));-2*sin(.5*t(i))]; % Defines the external forces applied on the rigid body
        tau[i] = pow(10,6)*phi[i];
        // tau(1:3,i)=1e6*phi(1:3,i);
        VectorXd temp(6);
        temp << phi[i],tau[i];
        VectorXd temp2 = (adse3s(xi[i]).transpose()*II*xi[i]+temp);
        xidot[i] << temp2(0)/II(0,0),temp2(1)/II(1,1),temp2(2)/II(2,2),temp2(3)/II(3,3),temp2(4)/II(4,4),temp2(5)/II(5,5);
        // xidot(1:6,i)=II\(adse3s(xi(1:6,i))'*II*xi(1:6,i)+[phi(1:3,i);tau(1:3,i)]);

        g12[i] = g[i]+dt/2*gdot[i];
        // g12(1:4,1:4,i)=g(1:4,1:4,i)+dt/2*gdot(1:4,1:4,i);
        xi12[i] = xi[i] + dt/2*xidot[i];
        // xi12(1:6,i)=xi(1:6,i)+dt/2*xidot(1:6,i);
        phi12[i] << pow(10,-8)*1*cos(0.1*(t[i]+0.5*dt)), pow(10,-8)*0.2*sin(0.2*(t[i]+0.5*dt)), -pow(10,-8)*0.2*sin(0.5*(t[i]+0.5*dt));
        // phi12(1:3,i)=1e-8*[1*cos(.1*(t(i)+.5*dt));.2*sin(.2*(t(i)+.5*dt));-0.2*sin(.5*(t(i)+.5*dt))];
        tau12[i] = pow(10,6)*phi12[i];
        // tau12(1:3,i)=1e6*phi12(1:3,i);
        gdot12[i] = g12[i]*R6tose3(xi12[i]);
        // gdot12(1:4,1:4,i)=g12(1:4,1:4,i)*R6tose3(xi12(1:6,i));
        temp << phi12[i],tau12[i];
        temp2 = (adse3s(xi12[i]).transpose()*II*xi12[i]+temp);
        xidot12[i] << temp2(0)/II(0,0),temp2(1)/II(1,1),temp2(2)/II(2,2),temp2(3)/II(3,3),temp2(4)/II(4,4),temp2(5)/II(5,5);
        // xidot12(1:6,i)=II\(adse3s(xi12(1:6,i))'*II*xi12(1:6,i)+[phi12(1:3,i);tau12(1:3,i)]);

        g[i+1] = g[i]  + dt*gdot12[i];
        // g(1:4,1:4,i+1)=g(1:4,1:4,i)+dt*gdot12(1:4,1:4,i);
        xi[i+1] = xi[i] + dt*xidot12[i];
        // xi(1:6,i+1)=xi(1:6,i)+dt*xidot12(1:6,i);
    }
}

void VertexVis3(MatrixXd Vxyz, Matrix4d g[int(Tt/d_t)], Matrix3d CamConfig3, double Theta_FV, MatrixXd V[int(Tt/d_t)],double Vnn[int(Tt/d_t)]){

    MatrixXd V_0 =  MatrixXd::Zero(3,8);
    for (int k = 0; k<int(Tt/d_t);k++){
        V[k] = V_0;
        Vnn[k] = 0;
    }
    // V=zeros(size(CamConfig3,2),size(Vxyz,2),size(g,3));
    // Vnn=zeros(1,size(g,3));
    for (int k=0;k<int(Tt/d_t);k++){
        for (int j=0;j<8;j++){ // Number of Vertices
            for (int i=0;i<3;i++) { //  % Number of Sensors on the rigid body
                VectorXd a  = (g[k].block(0,0,3,3)*CamConfig3.col(i)).transpose()*(g[k].block(0,3,3,1)-Vxyz.col(j));
                Vector3d b = g[k].block(0,3,3,1)-Vxyz.col(j);
                if ( abs(a(0)) / sqrt(pow(b(0),2)+pow(b(1),2)+pow(b(2),2)) > cos(Theta_FV) ){
                // abs((g(1:3,1:3,k)*CamConfig3(:,i))'*(g(1:3,4,k)-Vxyz(:,j)))/norm(g(1:3,4,k)-Vxyz(:,j))>cos(Theta_FV)
                V[k].block(i,j,1,1) << 1; // V(i,j,k)=1;
                }
                else {
                V[k].block(i,j,1,1) << 0; // V(i,j,k)=0;
                }
            }
            VectorXd c = V[k].block(0,j,1,1)+V[k].block(1,j,1,1)+V[k].block(2,j,1,1);
            if ( c(0) != 0){
                Vnn[k] = Vnn[k] + 1;
                // Vnn(k)=Vnn(k)+1; % Total number of Beacons observed at each time instance
            }
        }
    }
}

MatrixXd randbump(double c, double m, double n) {
//% This function generates a bump function with normal distribution
//% "c" is the maximum value of the noise
//% "m,n" are the dimensions of the output noise matrix
//% MI 10/08/2014

    MatrixXd x=MatrixXd::Zero(m,n);
    if (c<1){
        for(int i=0;i<m;i++){
            for (int j=0;j<n;j++){
                int k = 0;
                while (k==0){
                    double x1 = c*(2.0*((rand() % 10000)/10000.0 + (rand() % 10000)/1000000.0)-1.0);
                    // x1=c*(2*rand(1)-1);
                    double x2 = ((rand() % 10000)/10000.0 + (rand() % 10000)/1000000.0);
                    // x2=rand(1);
                    if (x2 < exp(1.0-pow(c,2)/(pow(c,2)-pow(x1,2)))){ // x2<exp(1-c^2/(c^2-x1^2))
                        x(i,j) = x1;
                        k=1;
                    }
                }
            }
        }
    }
    return x;
}

void Measurements(MatrixXd IMU, MatrixXd Vxyz, MatrixXd V[int(Tt/d_t)], double Vnn[int(Tt/d_t)], Matrix4d g[int(Tt/d_t)], double Position_Noise, VectorXd w0,
                  Vector3d barp[int(Tt/d_t)],Vector3d baram[int(Tt/d_t)], MatrixXd Am[int(Tt/d_t)], MatrixXd W[int(Tt/d_t)], MatrixXd Em[int(Tt/d_t)], MatrixXd D[int(Tt/d_t)]) {

    double maxVnn = 0;
    for (int k = 0; k<int(Tt/d_t);k++){
        if (maxVnn < Vnn[k]){
            maxVnn = Vnn[k];
        }
    }
    MatrixXd D_0 = MatrixXd::Zero(3,maxVnn+1);
    MatrixXd E_0 = MatrixXd::Zero(3,maxVnn+1);
    MatrixXd Em_0 = MatrixXd::Zero(3,maxVnn+1);
    MatrixXd W_0 = MatrixXd::Zero(maxVnn+1,maxVnn+1);
    MatrixXd Am_0 = MatrixXd::Zero(4,maxVnn);
    Vector3d barp_0 = Vector3d::Zero(3);
    Vector3d baram_0 = Vector3d::Zero(3);
    MatrixXd E[int(Tt/d_t)];
    double p0j[int(Tt/d_t)];
    for (int i = 0; i<int(Tt/d_t);i++){
            D[i] = D_0;
            E[i] = E_0;
            Em[i] = Em_0;
            W[i] = W_0;
            Am[i] = Am_0;
            p0j[i] = 0;
            barp[i] = barp_0;
            baram[i] = baram_0;
    }
    // D=zeros(3,max(Vnn)+1,size(g,3));
    // E=zeros(3,max(Vnn)+1,size(g,3));
    // Em=zeros(3,max(Vnn)+1,size(g,3));
    // W=zeros(max(Vnn)+1,max(Vnn)+1,size(g,3));
    // Am=zeros(4,max(Vnn),size(g,3));
    // p0j=zeros(1,size(g,3));
    // barp=zeros(3,size(g,3));
    // baram=zeros(size(barp));

    for (int k = 0;k < int(Tt/d_t);k++ ) {
        D[k].block(0,0,3,2) << IMU;
        // D(:,1:2,k)=IMU;
        E[k].block(0,0,3,2) << g[k].block(0,0,3,3).transpose()*IMU;
        // E(:,1:2,k)=g(1:3,1:3,k)'*IMU;
        Em[k].block(0,0,3,2) << E[k].block(0,0,3,2) + randbump(Position_Noise,3,2);
        // Em(:,1:2,k)=E(:,1:2,k)+randbump(Position_Noise,3,2);
        W[k].block(0,0,2,2) << w0(0), 0, 0, w0(1);
        // W(1:2,1:2,k)=diag(w0(1:2));
        int i = 0;
        // i=1;
        int j = 0;
        // j=1;
        VectorXd temp = V[k].block(i,j,1,1);
        while ( temp(0) == 0){
            i = i+1;
            if (i > V[k].rows()-1){
                j = j+1;
                i=0;
            }
            temp = V[k].block(i,j,1,1);
            // i=i+1;
            // if i>size(V,1)
            // j=j+1;
            // i=1;
        }
        p0j[k] = j;
        // p0j(k)=j;
        Am[k].block(0,0,3,1) << g[k].block(0,0,3,3).transpose()*(Vxyz.block(0,p0j[k],3,1)-g[k].block(0,3,3,1))+randbump(Position_Noise,3,1);
        //Am[k].block(0,0,3,1) << g[k].block(0,0,3,3).transpose()*(Vxyz.cols(p0j[k])-g[k].block(0,3,3,1))+randbump(Position_Noise,3,1);
        Am[k].block(3,0,1,1) << j+1;
        // Am(4,1,k)=j;
        i = 0;
        // i=1;
        j = p0j[k]+1;
        // j=p0j(k)+1;
        barp[k] << Vxyz.block(0,p0j[k],3,1);
        // barp(:,k)=Vxyz(:,p0j(k));
        for (int l = 1; l < Vnn[k]; l++){
            VectorXd temp1 = V[k].block(i,j,1,1);
            while (temp1(0) == 0){
                i=i+1;
                if (i > 2){
                    j=j+1;
                    i=0;
                }
                temp1 = V[k].block(i,j,1,1);
                //while V(i,j,k)==0
                  //  i=i+1;
                    //if i>size(V,1)% && j<size(V,2)
                      //  j=j+1;
                      //  i=1;
                   // end
               // end
            }

            D[k].block(0,l+1,3,1) << Vxyz.block(0,j,3,1) - Vxyz.block(0,p0j[k],3,1);;
            // D(:,l+1,k)=Vxyz(:,j)-Vxyz(:,p0j(k));
            barp[k] << barp[k] + Vxyz.block(0,j,3,1);
            // barp(:,k)=barp(:,k)+Vxyz(:,j);
            W[k].block(l+1,l+1,1,1) << w0[j+2];
            // W(l+1,l+1,k)=w0(j+2);
            E[k].block(0,l+1,3,1) << g[k].block(0,0,3,3).transpose()*D[k].block(0,l+1,3,1);
            // E(:,l+1,k)=g(1:3,1:3,k)'*D(:,l+1,k);%-g(1:3,4,k);
            Em[k].block(0,l+1,3,1) << E[k].block(0,l+1,3,1) + randbump(Position_Noise,3,1);
            // Em(:,l+1,k)=E(:,l+1,k)+randbump(Position_Noise,3,1);
            Am[k].block(0,l,3,1) << Am[k].block(0,0,3,1) + Em[k].block(0,l+1,3,1);
            // Am(1:3,l,k)=Am(1:3,1,k)+Em(1:3,l+1,k);
            Am[k].block(3,l,1,1) << j+1;
            // Am(4,l,k)=j;
            if ( j < 7) {
                j=j+1;
                i=0;
            }
        }
        baram[k] << Am[k].block(0,0,1,6).sum(),Am[k].block(1,0,1,6).sum(),Am[k].block(2,0,1,6).sum();
        // baram(:,k)=sum(Am(1:3,:,k),2);
    }
    for (int k = 0; k < int(Tt/d_t); k++){
        barp[k] << barp[k]/Vnn[k];
        // barp(:,k)=barp(:,k)/Vnn(k);
        baram[k] << baram[k] / Vnn[k];
        // baram(:,k)=baram(:,k)/Vnn(k);
    }

}

bool ismember(double a, VectorXd am){
    bool test = false;
    for (int k = 0 ; k < am.size(); k++){
        if (a == am(k)){
            test = true;
        }
    }
    return test;
}

int find_col(double a, VectorXd am){
    int col = 0;
    for (int k = 0 ; k < am.size(); k++){
        if (a == am(k)){
            col = k;
        }
    }
    return col;
}

void Vmm(MatrixXd Am[int(Tt/d_t)], MatrixXd Amf[int(Tt/d_t)], MatrixXd Vm[int(Tt/d_t)]) {

    MatrixXd Vm_0 = MatrixXd::Zero(Am[0].rows(),Am[0].cols());
    MatrixXd Amf_0 = MatrixXd::Zero(Am[0].rows(),Am[0].cols());
    for (int i = 0; i<int(Tt/d_t);i++){
            Vm[i] = Vm_0;
            Amf[i] = Amf_0;
    }
    Amf[0] = Am[0];
    // Vm=zeros(size(Am));
    // Amf=zeros(size(Am));
     //Amf(:,:,1)=Am(:,:,1);
    for (int i = 1; i< int(Tt/d_t);i++){//int(Tt/d_t)
        int m=0;
        while (m < Am[0].cols()){
                VectorXd temp = Am[i].block(3,m,1,1);
                VectorXd temp2 = VectorXd::Zero(6);
                temp2 << Am[i-1].block(3,0,1,1),Am[i-1].block(3,1,1,1),Am[i-1].block(3,2,1,1),Am[i-1].block(3,3,1,1),Am[i-1].block(3,4,1,1),Am[i-1].block(3,5,1,1);

            if ((temp(0) > 0) && (ismember(temp(0),temp2))){
                int p = find_col(temp(0),temp2);
                // p=find(Am(4,:,i-1)==Am(4,m,i));
                Amf[i].block(0,m,3,1) << 1/(2*0.1+d_t)*((2*0.1-d_t)*Amf[i-1].block(0,p,3,1)+d_t*(Am[i-1].block(0,p,3,1)+Am[i].block(0,m,3,1)));
                // Amf(1:3,m,i)=filtered(Am(1:3,p,i-1),Am(1:3,m,i),Amf(1:3,p,i-1),dt,.1);
                Amf[i].block(3,m,1,1) << Am[i].block(3,m,1,1);
                // Amf(4,m,i)=Am(4,m,i);
                Vm[i].block(0,m,3,1) << 1/d_t*(Am[i].block(0,m,3,1) - Am[i-1].block(0,p,3,1));
                // Vm(1:3,m,i)=1/dt*(Am(1:3,m,i)-Am(1:3,p,i-1));
                Vm[i].block(3,m,1,1) << Am[i].block(3,m,1,1);
                // Vm(4,m,i)=Am(4,m,i);
            }
            else if ((temp(0) > 0) ){
                Amf[i].block(0,m,4,1) << Am[i].block(0,m,4,1);
                // Amf(1:4,m,i)=Am(1:4,m,i);
            }
            m=m+1;

        }

    }
    Vm[0] =  Vm[1] ;
    // Vm(:,:,1)=Vm(:,:,2);

}

VectorXd backslash(MatrixXd a,VectorXd b){

    VectorXd x1 = a.colPivHouseholderQr().solve(b);
    //VectorXd x1 = a.householderQr().solve(b);

    return x1;
}

VectorXd xim( double Vnn, MatrixXd Am, MatrixXd Vm){
// % Vnn: The number of beacons observed
// % Am: Matrix of beacons positions measured in body fixed frame
// % Am: Matrix of beacons velocities measured in body fixed frame
    VectorXd ximv(6,1);
    int p =0;
    MatrixXd G[6];
    G[0] = MatrixXd::Zero(3,6);
    G[1] = G[0] ; G[2] = G[0]; G[3] = G[0]; G[4] = G[0] ; G[5] = G[0];
    // G=zeros(3,6,3);

    MatrixXd v3 = MatrixXd::Zero(3,6);
    MatrixXd v2 = MatrixXd::Zero(3,6);
    MatrixXd v1 = MatrixXd::Zero(3,1);
    // v=zeros(3,3);
    if (Vnn>2){
        for (int k = 0; k < 6 ; k++){
            if(Vm(3,k)>0){
                v3.block(0,p,3,1) << Vm.block(0,k,3,1);
                int n = find_col(Vm(3,k),Am.block(3,0,1,6).transpose());
                G[p].block(0,0,3,6) << skew(Am.block(0,n,3,1)),-MatrixXd::Identity(3,3);
                p = p+1;
                //v(:,p)=Vm(1:3,k,i);
                // n=find(Am(4,:,i)==Vm(4,k,i));
                // G(:,:,p)=[skew(Am(1:3,n,i)) -eye(3)];
                // p=p+1;
            }
        }
    }
    if (Vnn>2){
        MatrixXd GG(9,6);
        GG << G[0],G[1],G[2];
        VectorXd vv(9);
        vv << v3.block(0,0,3,1),v3.block(0,1,3,1),v3.block(0,2,3,1);
        ximv = backslash(GG,vv);
        //  xim=[G(:,:,1);G(:,:,2);G(:,:,3)]\[v(:,1);v(:,2);v(:,3)];
    }
    if (Vnn == 2){
        ximv = MatrixXd::Zero(6,1);
        // xim=zeros(6,1);
        for (int k = 0; k < 6 ; k++ ){
            if (Vm(3,k) > 0){
                v2.block(0,p,3,1) << Vm.block(0,k,3,1);
                int n = find_col(Vm(3,k),Am.block(3,0,1,6).transpose());
                MatrixXd M_temp1(3,6);
                M_temp1 << skew(Am.block(0,n,3,1)),-MatrixXd::Identity(3,3);
                MatrixXd M_temp2(3,1);
                M_temp2 = M_temp1*M_temp1.transpose();
                ximv = ximv + M_temp1.transpose()*InvMat(M_temp2)*v2.block(0,p,3,1);
                p = p+1;
                // v(:,p)=Vm(1:3,k,i);
                // n=find(Am(4,:,i)==Vm(4,k,i));
                // xim=xim+[skew(Am(1:3,n,i)) -eye(3)]'/([skew(Am(1:3,n,i)) -eye(3)]*[skew(Am(1:3,n,i)) -eye(3)]')*v(:,p);
                // p=p+1;
            }
        }
        ximv = ximv/2;
        // xim=xim/2;
    }
    if (Vnn == 1){
        for (int k = 0; k < 6 ; k++ ){
            if (Vm(3,k) > 0){
                v1 << Vm.block(0,k,3,1);
                int n = find_col(Vm(3,k),Am.block(3,0,1,6).transpose());
                MatrixXd M_temp1(3,6);
                M_temp1 << skew(Am.block(0,n,3,1)),-MatrixXd::Identity(3,3);
                MatrixXd M_temp2(3,1);
                M_temp2 = M_temp1*M_temp1.transpose();
                ximv << M_temp1.transpose()*InvMat(M_temp2)*v1;
                p = p+1;
                // v=Vm(1:3,k,i);
                // n=find(Am(4,:,i)==Vm(4,k,i));
                // xim=[skew(Am(1:3,n,i)) -eye(3)]'/([skew(Am(1:3,n,i)) -eye(3)]*[skew(Am(1:3,n,i)) -eye(3)]')*v;
            }
        }
    }
return ximv;
}

MatrixXd invSE3(MatrixXd G){
// % Computes inverse of a 4x4 matrix representation of an element of SE(3)

Matrix3d GR;
GR << G.block(0,0,3,3);
//GR=G(1:3,1:3);
MatrixXd Gb(3,1);
Gb << G.block(0,3,3,1);
//Gb=G(1:3,4);
MatrixXd Gi(4,4);
Gi << GR.transpose(),-GR.transpose()*Gb,MatrixXd::Zero(1,3),1;
//Gi=[GR' -GR'*Gb;zeros(1,3) 1];

return Gi;
}

MatrixXd AdSE3(MatrixXd g){
// % AdSE3: Computes the matrix representation of Ad_g for g in SE(3)
// % Created: June 23, 2012, AKS

Matrix3d R;
R << g.block(0,0,3,3);
//R=g(1:3,1:3);
MatrixXd b(3,1);
b << g.block(0,3,3,1);
//b=g(1:3,4);
Matrix3d bcross;
bcross = skew(b);
//bcross=skew(b);
MatrixXd Adg(6,6);
Adg << R,MatrixXd::Zero(3,3),bcross*R,R;
//Gi=[GR' -GR'*Gb;zeros(1,3) 1];

return Adg;
}

MatrixXd expmse3(MatrixXd Xi){

    MatrixXd O(3,1);
    O << Xi(2,1),Xi(0,2),Xi(1,0);
    Matrix3d Os = skew(O);
    double phi = sqrt(pow(O(0,0),2)+pow(O(1,0),2)+pow(O(2,0),2));
    // O = [Xi(3,2) Xi(1,3) Xi(2,1)];
    // Os = skew(O);
    // Phi=norm(O);

    double cosc_phi = 0;
    if (phi < 0.3){
        cosc_phi = (1-cos(phi))/pow(phi,2);
    }
    else {
        cosc_phi = (double)(1/2 - pow(phi,2)/24 + pow(phi,4)/720 - pow(phi,6)/40320 + pow(phi,8)/3628800 - pow(phi,10)/47901600);
    }
    // function y=cosc(x)
    // if x>0.3
        // y=(1-cos(x))/x^2;
    // else
        // y=1/2-x^2/24+x^4/720-x^6/40320+x^8/3628800-x^10/479001600;
    // end
    double sind_phi = 0;
    if ( phi > 0.2) {
        sind_phi = (phi - sin(phi))/pow(phi,3);
    }
    else {
        sind_phi = 0.16666667;
        int fact = 1*2*3*4;
        for (int n =4 ; n<11;n=n+2){
            fact = fact*(n)*(n+1);
            sind_phi = sind_phi + pow(-1,n+1)*pow(phi,(n+1)*2)/fact;
        }
    }
    // function v=sind(u)
    // if u>0.2
        // v=(u-sin(u))/u^3;
    // else
        // v=1/6-u^2/factorial(5)+u^4/factorial(7)-u^6/factorial(9)+u^8/factorial(11);
    // end

    Matrix3d S = MatrixXd::Identity(3,3) + cosc_phi*Os + sind_phi*Os*Os;
    // S=eye(3,3) + cosc(Phi)*Os + sind(Phi)*Os^2;

    MatrixXd G(4,4);
    G << expmso3(O), S*Xi.block(0,3,3,1), MatrixXd::Zero(1,3), 1;
    // G = [expmso3(O) S*Xi(1:3,4); zeros(1,3) 1];

    return G;
}

Vector3d crossproduct(Vector3d A, Vector3d B){
// equivalent of the function "cross" in Matlab
Vector3d C;
C << A(1)*B(2)-A(2)*B(1), A(2)*B(0)-A(0)*B(2), A(0)*B(1)-A(1)*B(0);
return C;
}

Matrix3d find_F_simplified(Matrix3d Jd, Vector3d m){
/*
% This function solves the equation F*Jd-Jd*F'=skew(m)
% using the Newton-Raphson based method from the
% Melvin et. al article
% Update: 02/23/2015, AKS
%
% Input:
% Jd
% m
%
% Output:
% F - the solution
% T - the time used for the computation
% err - the error in solving the equation
%
% function [F,T,err]=find_F_NewtonRaphson(Jd,m)
*/

    Matrix3d J = Jd.trace()*MatrixXd::Identity(3,3)-Jd;
    // J=trace(Jd)*eye(3)-Jd;
    Vector3d fi = m; //% initial guess
    // fi=m;
    double nf = sqrt(pow(fi(0),2)+pow(fi(1),2)+pow(fi(2),2));
    // nf=norm(fi);

    double cosc_nf = 0;
    if (nf > 0.01){
        cosc_nf = (1-cos(nf))/pow(nf,2);
    }
    else {
        cosc_nf = 0.5 - pow(nf,2)/24 + pow(nf,4)/720 - pow(nf,6)/40320;
    }
/*    function y=cosc(nx)
    if nx>0.01
        y=(1-cos(nx))/nx^2;
    else
        y=1/2-nx^2/24+nx^4/720-nx^6/40320;
    end */

    double sinc_nf_pi = 0;
    if ( (nf/PI)>0.2) {
        sinc_nf_pi = sin(nf/PI)/(nf/PI);
    }
    else {
        sinc_nf_pi = 1 - pow((nf/PI),2)/6 + pow((nf/PI),4)/120 - pow((nf/PI),6)/5040 + pow((nf/PI),8)/362880 - pow((nf/PI),10)/39916800;
    }
    /*function v=sinc(w)
    if w>0.2
        v=sin(w)/w;
    else
        v=1-w^2/6+w^4/120-w^6/5040+w^8/362880-w^10/39916800;
    end*/

    Vector3d G = sinc_nf_pi*J*fi+cosc_nf*crossproduct(fi,J*fi);
    // G=sinc(nf/pi)*J*fi+cosc(nf)*cross(fi,J*fi);
    Vector3d Gm = G-m;
    double norm_Gm = sqrt(pow(Gm(0),2)+pow(Gm(1),2)+pow(Gm(2),2));
    while (norm_Gm > pow(10,-9)){
        nf = sqrt(pow(fi(0),2)+pow(fi(1),2)+pow(fi(2),2));
        G = sin(nf)/nf*J*fi+cosc_nf*crossproduct(fi,J*fi);
        Matrix3d temp;
        temp << crossproduct(fi,J.block(0,0,3,1)),crossproduct(fi,J.block(0,1,3,1)),crossproduct(fi,J.block(0,2,3,1));
        Matrix3d nabG = (cos(nf)*nf-sin(nf))/pow(nf,3)*(J*fi)*fi.transpose() + sinc_nf_pi*J + (sin(nf)*nf-2*(1-cos(nf)))/pow(nf,4)*crossproduct(fi,J*fi)*fi.transpose() + cosc_nf*(skew(-J*fi)+temp);
        fi = fi+ backslash(nabG,m-G);
        Gm = G-m;
        norm_Gm = sqrt(pow(Gm(0),2)+pow(Gm(1),2)+pow(Gm(2),2));
        // nf=norm(fi);
        // G=sin(nf)/nf*J*fi+cosc(nf)*cross(fi,J*fi);
        // nabG=(cos(nf)*nf-sin(nf))/nf^3*(J*fi)*fi'+sinc(nf/pi)*J...
            // +(sin(nf)*nf-2*(1-cos(nf)))/nf^4*cross(fi,J*fi)*fi'...
            // +cosc(nf)*(skew(-J*fi)+[cross(fi,J(:,1)),cross(fi,J(:,2)),cross(fi,J(:,3))]);
        // fi=fi+nabG\(m-G);
    }

    Matrix3d F = expmso3(fi);
    // F=expmso3(fi);
    return F;
}

Vector3d SLR(Matrix3d L, Matrix3d Rhat){
    Matrix3d sw = L*Rhat.transpose()-Rhat*L.transpose();
    Vector3d slr = unskew(sw);
    // SLR=unskew(L*Rhat'-Rhat*L');
    return slr;
}

void LGVI(Matrix4d g[int(Tt/d_t)], Matrix4d ghat0, VectorXd varphi0, Matrix3d J, Matrix3d M, float kappa, MatrixXd DD, MatrixXd D[int(Tt/d_t)], MatrixXd Em[int(Tt/d_t)], MatrixXd W[int(Tt/d_t)], double Vnn[int(Tt/d_t)],  MatrixXd Am[int(Tt/d_t)], MatrixXd Vm[int(Tt/d_t)], Vector3d barp[int(Tt/d_t)], Vector3d baram[int(Tt/d_t)],
          MatrixXd h[int(Tt/d_t)], VectorXd varphi[int(Tt/d_t)], double VL[int(Tt/d_t)], VectorXd xihat[int(Tt/d_t)], VectorXd Vel[int(Tt/d_t)] ) {

    Matrix3d Jd = 0.5*J.trace()*MatrixXd::Identity(3,3)-J;
    VectorXd varphi_0(6);
    varphi_0 << 0,0,0,0,0,0;
    for(int k = 0; k < int(Tt/d_t);k++ ){
        varphi[k] = varphi_0;
        xihat[k] = varphi_0;
        Vel[k] = varphi_0;
    }
    varphi[0] = varphi0;
    Vector3d omega1;
    omega1 << varphi0(0),varphi0(1),varphi0(2);
    Vector3d upsilon1;
    upsilon1 << varphi0(3),varphi0(4),varphi0(5);
    MatrixXd h_0 = MatrixXd::Zero(4,4);
    for(int k = 0; k < int(Tt/d_t);k++ ){
        h[k] = h_0;
        VL[k] = 0;
    }
    h[0] = divideMat(g[0],ghat0);

    MatrixXd ghat1 = ghat0;
/*
    Jd=0.5*trace(J)*eye(3)-J;
    varphi=zeros(6,ceil(T/dt));
    varphi(:,1)=varphi0;
    xihat=zeros(size(varphi));
    omega1=varphi0(1:3,1);
    upsilon1=varphi0(4:6,1);
    h=zeros(size(g));
    h(:,:,1)=g(:,:,1)/ghat0;
    VL=zeros(1,size(g,3));
    ghat1=ghat0;
    % baram(:,k)=sum(Am(:,:,k),2);
    Vel=zeros(6,size(g,3));
*/

    for (int i = 0 ; i < int(Tt/d_t)-1; i++ ){
        // %     barp(:,k)=sum(Am(:,:,i),2)/Vnn(i);
        Vel[i] = xim(Vnn[i],Am[i],Vm[i]); // %ximMAX(i,Am,Vm);xi(:,i)+[randbump(.01,3,1);randbump(.001,3,1)]
        // Vel(1:6,i)=xim(i,Vnn,Am,Vm);
        xihat[i] = xim(Vnn[i],Am[i],Vm[i]) - AdSE3(invSE3(ghat1))*varphi[i];
        // xihat(:,i)=xim(i,Vnn,Am,Vm)-AdSE3(invSE3(ghat1))*varphi(:,i);
        MatrixXd ghat2 = ghat1*expmse3(d_t*R6tose3(xihat[i]));
        // ghat2=ghat1*expmse3(dt*R6tose3(xihat(:,i)));
        Matrix3d Rhat2 = ghat2.block(0,0,3,3);
        // Rhat2=ghat2(1:3,1:3);
        Vector3d bhat2 = ghat2.block(0,3,3,1);
        // bhat2=ghat2(1:3,4);
        Matrix3d F = find_F_simplified(Jd,d_t*J*omega1);
        // F=find_F_simplified(Jd,dt*J*omega1);
        Vector3d upsilon2 = backslash(M+d_t*DD.block(3,3,3,3),F.transpose()*M*upsilon1+d_t*kappa*(bhat2+Rhat2*baram[i+1]-barp[i]));
        // upsilon2=(M+dt*DD(4:6,4:6))\(F'*M*upsilon1+dt*kappa*(bhat2+Rhat2*baram(:,i+1)-barp(:,i)));
        MatrixXd L1 = D[i+1].block(0,0,3,Vnn[i+1]+1);
        MatrixXd L2 = W[i+1].block(0,0,Vnn[i+1]+1,Vnn[i+1]+1);
        MatrixXd L3 = Em[i+1].block(0,0,3,Vnn[i+1]+1).transpose();
        Matrix3d L = L1*L2*L3;
        // L=D(:,1:Vnn(i+1)+1,i+1)*W(1:Vnn(i+1)+1,1:Vnn(i+1)+1,i+1)*Em(:,1:Vnn(i+1)+1,i+1)';
        Vector3d omega_temp1 = F.transpose()*J*omega1;
        Vector3d omega_temp2 = d_t*crossproduct(M*upsilon2,upsilon2);
        Vector3d omega_temp3 = d_t*SLR(L,Rhat2);
        Vector3d omega_temp4 = d_t*kappa*crossproduct(barp[i+1],bhat2+Rhat2*baram[i+1]);
        Vector3d omega_temp = omega_temp1+omega_temp2-omega_temp3+omega_temp4;
        Vector3d omega2 = backslash(J+d_t*DD.block(0,0,3,3),omega_temp);
        /*
        omega2=(J+dt*DD(1:3,1:3))\(F'*J*omega1+dt*cross(M*upsilon2,upsilon2)-dt*SLR(L,Rhat2)...
            +dt*kappa*cross(barp(:,i+1),bhat2+Rhat2*baram(:,i+1)));
        */
        h[i+1] = divideMat(g[i+1], ghat2);
        // h(:,:,i+1)=g(:,:,i+1)/ghat2;
        varphi[i+1] << omega2,upsilon2;
        // varphi(:,i+1)=[omega2;upsilon2];
        MatrixXd Mtemp(6,6);
        Mtemp << J,MatrixXd::Zero(3,3),MatrixXd::Zero(3,3),M;
        double vtemp1 = 0.5*varphi[i+1].transpose()*Mtemp*varphi[i+1];
        Vector3d Vtemp = (bhat2+Rhat2*baram[i+1]-barp[i+1]);
        double vtemp2 = 0.5*kappa*Vtemp.transpose()*Vtemp;
        double vtemp3 = 0.5*((D[i+1].block(0,0,3,Vnn[i+1]+1)-Rhat2*Em[i+1].block(0,0,3,Vnn[i+1]+1)).transpose()*(D[i+1].block(0,0,3,Vnn[i+1]+1)-Rhat2*Em[i+1].block(0,0,3,Vnn[i+1]+1))*W[i+1].block(0,0,Vnn[i+1]+1,Vnn[i+1]+1)).trace();
        VL[i] = vtemp1 + vtemp2 + vtemp3;
        /*
        VL(i)=.5*varphi(:,i+1)'*[J zeros(3,3);zeros(3,3) M]*varphi(:,i+1)...
        +.5*kappa*((bhat2+Rhat2*baram(:,i+1)-barp(:,i+1))'*(bhat2+Rhat2*baram(:,i+1)-barp(:,i+1)))...
        +.5*trace((D(:,1:Vnn(i+1)+1,i+1)-Rhat2*Em(:,1:Vnn(i+1)+1,i+1))'*...
        (D(:,1:Vnn(i+1)+1,i+1)-Rhat2*Em(:,1:Vnn(i+1)+1,i+1))*W(1:Vnn(i+1)+1,1:Vnn(i+1)+1,i+1));
        */
        ghat1 = ghat2;
        // ghat1=ghat2;
        omega1 = omega2;
        // omega1=omega2;
        upsilon1 = upsilon2;
        // upsilon1=upsilon2;
    }
    VL[int(Tt/d_t)-1] = VL[int(Tt/d_t)-2];
}

Matrix3d ProjSO3(Matrix3d Qr9){

    // % ProjSO3 (Created 4/6/2013, AKS)
    // % projects a 3x3 matrix to SO(3)

    MatrixXd Q1 = Qr9.block(0,0,1,3);
    Q1 = Q1/sqrt(pow(Q1(0),2)+pow(Q1(1),2)+pow(Q1(2),2));
    MatrixXd Q2 = Qr9.block(1,0,1,3);
    Q2 = Q2-(Q2*Q1.transpose())*Q1;
    Q2 = Q2/sqrt(pow(Q2(0),2)+pow(Q2(1),2)+pow(Q2(2),2));
    MatrixXd Q3 = crossproduct(Q1.transpose(),Q2.transpose()).transpose();
    Matrix3d Q;
    Q << Q1,Q2,Q3;
    /*Q1=Qr9(1,:);
    Q1=Q1/norm(Q1);
    Q2=Qr9(2,:);
    Q2=Q2-(Q2*Q1')*Q1;
    Q2=Q2/norm(Q2);
    Q3=cross(Q1,Q2);
    Q=[Q1; Q2; Q3]; */
    return Q;
}

int main() {

    long clk_tck = CLOCKS_PER_SEC;
    clock_t t1, t2;
    /* initial time in "clock ticks" */
    t1 = clock(); // tc=cputime;

    float t[int(Tt/d_t)];
    for(int i = 0; i < Tt/d_t; ++i){
            t[i] = d_t*i ; }

    //====== GAINS =======//

    Matrix3d J;
    J << 0.3*3, 0, 0,
         0, 0.3*2, 0,
         0, 0, 0.3*1;
    // J=0.3*diag([3,2,1]);
    Matrix3d M;
    M << 0.05*0.81*1.5, 0, 0,
         0, 0.05*0.81*1.2, 0,
         0, 0, 0.05*0.81*0.9;
    // M=.05*0.81*diag([1.5,1.2,0.9]);
    MatrixXd DD(6,6) ;
    DD << 2.7, 0, 0, 0, 0, 0,
         0, 2.2, 0, 0, 0, 0,
         0, 0, 1.5, 0, 0, 0,
         0, 0, 0, 0.1, 0, 0,
         0, 0, 0, 0, 0.12, 0,
         0, 0, 0, 0, 0, 0.14;
    // DD=diag([2.7,2.2,1.5,.1*1,.1*1.2,.1*1.4]);
    float kappa = 0.1; //kappa=.1;
    VectorXd w0(10);
    w0 << 5,5,1,1,1,1,1.5,1.5,2,2;
    // w0=[5 5 1 1 1 1 1.5 1.5 2 2];% The vector of beacons' weight

    //====== Initialization =======//

    Vector3d r0;
    r0 << PI/4*3/7, -PI/4*6/7, PI/4*2/7;
    Matrix3d R0 = expmso3(r0);
    // R0=expmso3(pi/4*[3/7;-6/7;2/7]);
    Vector3d b0;
    b0 << 2.5, -0.5, -3;
    // b0=[2.5;-0.5;-3];
    Matrix4d g0;
    g0 << R0,b0,
          0,0,0,1;
    // g0=[R0 b0;0 0 0 1];
    Matrix3d I3 = Matrix3d::Identity();
    Vector3d bhat0;
    bhat0 << -3, 2, 4;
    Matrix4d ghat0;
    ghat0 << I3,bhat0,
             0,0,0,1;
    // ghat0=[eye(3) [-3;2;4];0 0 0 1];
    VectorXd varphi0(6);
    varphi0 << 0.1, -0.5, 0.05, 0.05, -0.09, 0.007;
    // varphi0=[.1;-.5;0.05;0.05;-0.09;0.007];
    MatrixXd II(6,6);
    II <<    0.02*2.56, 0, 0, 0, 0, 0,
             0, 0.02*3.01, 0, 0, 0, 0,
             0, 0, 0.02*2.98, 0, 0, 0,
             0, 0, 0,  0.02*21,  0,  0,
             0, 0, 0,  0,  0.02*21,  0,
             0, 0, 0,  0,  0,  0.02*21;
    // II=.02*diag([2.56,3.01,2.98,21,21,21]);
    Vector3d Omega0;
    Omega0 << 0.2, -0.05, 0.1;
    // Omega0=[.2;-0.05;.1];
    Vector3d nu0;
    nu0 << -0.01*.05, 0.01*0.15, 0.01*0.03;
    // nu0=0.01*[-.05;.15;.03];
    VectorXd xi0(6);
    xi0 << Omega0,nu0;
    // xi0=[Omega0;nu0];

    int l = 5;
    // l=5; % Room's half-length size

    Vector3d e1;
    e1 << 0,0,-1;// e1=[0;0;-1];
    Vector3d e2;
    e2 << 0.1,sqrt(1-0.01-0.04),-0.2; // sqrt(1-.01-.04);-.2];
    MatrixXd IMU(3,2);
    IMU << e1,e2;
    // IMU=[e1 e2];% The Inertial Vectors (Nadir & Earth's Magnetic Field)
    MatrixXd Vxyz(3,8);
    Vxyz << -l, l, -l, l, -l, l, -l, l,
            -l, -l, l, l, -l, -l, l, l,
            -l, -l, -l, -l, l, l, l, l; // Matrix of all vertices coordinates
    Matrix3d CamConfig3;
    CamConfig3 << 1, -sin(PI/6), -sin(PI/6),
                  0,  cos(PI/6), -cos(PI/6),
                  0,     0     ,      0    ;
    // The matrix of all cameras directions expressed in body-fixed frame


    //====== Algorithms =======//

    Matrix4d g[int(Tt/d_t)];
    VectorXd xi[int(Tt/d_t)];
    Dynamics(t,II,g0,xi0,g,xi);
    // [g,xi]=Dynamics(t,II,g0,xi0);
    double Theta_FV = 40.0/180.0*PI;
    // Theta_FV=40/180*pi; % Fully measured beacons
    MatrixXd V[int(Tt/d_t)];
    double Vnn[int(Tt/d_t)];
    VertexVis3(Vxyz,g,CamConfig3,Theta_FV,V,Vnn);
    // [V,Vnn]=VertexVis3(Vxyz,g,CamConfig3,Theta_FV);
    double Position_Noise = 0.001;
    // Position_Noise=.001;
    Vector3d barp[int(Tt/d_t)];
    Vector3d baram[int(Tt/d_t)];
    MatrixXd Am[int(Tt/d_t)];
    MatrixXd W[int(Tt/d_t)];
    MatrixXd Em[int(Tt/d_t)];
    MatrixXd D[int(Tt/d_t)];
    Measurements(IMU,Vxyz,V,Vnn,g,Position_Noise,w0,barp,baram,Am,W,Em,D);
    // [D,Em,W,Am,barp,baram]=Measurements(IMU,Vxyz,V,Vnn,g,Position_Noise,w0);

    MatrixXd Amf[int(Tt/d_t)];
    MatrixXd Vm[int(Tt/d_t)];
    Vmm(Am,Amf,Vm);
    // [Vm,Amf]=Vm(Am,dt);
    MatrixXd h0 = divideMat(g[0],ghat0);
    // h0=g(:,:,1)/ghat0;

    // % [varphi,hQx,VL]=LGVI_Qx(dt,T,g,ghat0,varphi0,J,M,kappa,DD,D,W,Vnn,barp);
    // % MyPlot(t,l,Vxyz,g,hQx,varphi);

    // %USE Amf INSTEAD OF Am IN THE FOLLOWING CALL TO LGVI
    MatrixXd h[int(Tt/d_t)];
    VectorXd varphi[int(Tt/d_t)];
    double VL[int(Tt/d_t)];
    VectorXd xihat[int(Tt/d_t)];
    VectorXd Vel[int(Tt/d_t)];
    LGVI(g,ghat0,varphi0,J,M,kappa,DD,D,Em,W,Vnn,Amf,Vm,barp,baram,h,varphi,VL,xihat,Vel);

    ///////////// Plots ////////////////////

    ofstream file1;
    file1.open ("g.txt");
    for (int i=0;i<int(Tt/d_t);i++){
        file1 <<t[i]<<"  "<<g[i](0,3)<<"  "<<g[i](1,3)<<"  "<<g[i](2,3)<<"\n";
    }
    file1.close();


    MatrixXd hP[int(Tt/d_t)];
    double Phi[int(Tt/d_t)];
    for (int k=0 ; k<int(Tt/d_t) ; k++ ){
        hP[k] = ProjSO3(h[k].block(0,0,3,3));
        Phi[k] = acos(0.5*(hP[k].trace()-1));
    }

    ofstream file2;
    file2.open ("xi and Phi.txt");
    for (int i=0;i<int(Tt/d_t);i++){
        file2 <<t[i]<<"  "<<Phi[i]<<"  "<<h[i](0,3)<<"  "<<h[i](1,3)<<"  "<<h[i](2,3)<<"\n";
    }
    file2.close();



    ofstream file3;
    file3.open ("varphi.txt");
    for (int i=0;i<int(Tt/d_t);i++){
        file3 <<t[i]<<"  "<<varphi[i](0)<<"  "<<varphi[i](1)<<"  "<<varphi[i](2)<<"  "<<varphi[i](3)<<"  "<<varphi[i](4)<<"  "<<varphi[i](5)<<"\n";
    }
    file3.close();










    /* final time in "clock ticks" */
    t2 = clock();
    cout << (double)(t2-t1)/(double)(clk_tck)<< endl;
    // disp(cputime-tc);

    return 0;
}
