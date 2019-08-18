/*************************************************************************
	> File Name: main.cpp
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Thu Mar  7 19:39:14 2019
 ************************************************************************/

#include<iostream>
#include<vector>
#include<random>
#include<cmath>
#include<Eigen/Eigen>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

#define SIM_TIME 50.0
#define DT 0.1
#define PI 3.141592653
#define MAX_RANGE 20.0
#define NP 100
#define NTh NP/2


// x_{t+1} = F@x_{t}+B@u_t
Eigen::Vector4f motion_model(Eigen::Vector4f x, Eigen::Vector2f u){
  Eigen::Matrix4f F_;
  F_<<1.0,   0,   0,   0,
        0, 1.0,   0,   0,
        0,   0, 1.0,   0,
        0,   0,   0, 1.0;

  Eigen::Matrix<float, 4, 2> B_;
  B_<< DT * std::cos(x(2,0)),  0,
       DT * std::sin(x(2,0)),  0,
                        0.0,  DT,
                        1.0,  0.0;

  return F_ * x + B_ * u;
};


//observation mode H
Eigen::Vector2f observation_model(Eigen::Vector4f x){
  Eigen::Matrix<float, 2, 4> H_;
  H_<< 1, 0, 0, 0,
       0, 1, 0, 0;
  return H_ * x;
};


// TODO gaussian likelihood
float gauss_likelihood(float x, float sigma){
  float p = 1.0 / std::sqrt(2.0 * PI * sigma * sigma) * \
      std::exp(-x * x / (2 * sigma * sigma));
  return p;
};

Eigen::Matrix4f calc_covariance(
    Eigen::Vector4f xEst,
    Eigen::Matrix<float, 4, NP> px,
    Eigen::Matrix<float, NP, 1> pw){

  Eigen::Matrix4f PEst_ = Eigen::Matrix4f::Zero();
  for(int i=0; i<px.cols(); i++){
      Eigen::Vector4f dx = px.col(i) - xEst;
      PEst_ += pw(i) * dx * dx.transpose();
  }

  return PEst_;
};

void pf_localization(
    Eigen::Matrix<float, 4, NP>& px, Eigen::Matrix<float, NP, 1>& pw,
    Eigen::Vector4f& xEst, Eigen::Matrix4f& PEst,
    std::vector<Eigen::RowVector3f> z, Eigen::Vector2f u,
    Eigen::Matrix2f Rsim, float Q,
    std::mt19937 gen,  std::normal_distribution<> gaussian_d
    ){

  for(int ip=0; ip<NP; ip++){
    Eigen::Vector4f x = px.col(ip);
    float w = pw(ip);

    Eigen::Vector2f ud;

    ud(0) = u(0) + gaussian_d(gen) * Rsim(0,0);
    ud(1) = u(1) + gaussian_d(gen) * Rsim(1,1);

    x = motion_model(x, ud);

    for(unsigned int i=0; i<z.size(); i++){
        Eigen::RowVector3f item = z[i];
        float dx = x(0) - item(1);
        float dy = x(1) - item(2);
        float prez = std::sqrt(dx*dx + dy*dy);
        float dz = prez - item(0);
        w = w * gauss_likelihood(dz, std::sqrt(Q));
    }
    px.col(ip) = x;
    pw(ip) = w;
  }

  pw = pw / pw.sum();

  xEst = px * pw;
  PEst = calc_covariance(xEst, px, pw);

};

Eigen::Matrix<float, NP, 1> cumsum(Eigen::Matrix<float, NP, 1> pw){
  Eigen::Matrix<float, NP, 1> cum;
  cum(0) = pw(0);
  for(int i=1; i<pw.rows(); i++){
    cum(i) = cum(i-1) + pw(i);
  }
  return cum;
}

void resampling(
    Eigen::Matrix<float, 4, NP>& px,
    Eigen::Matrix<float, NP, 1>& pw,
    std::mt19937 gen,
    std::uniform_real_distribution<> uni_d){

  float Neff = 1.0 / (pw.transpose() * pw);
  if (Neff < NTh){
    Eigen::Matrix<float, NP, 1> wcum = cumsum(pw);
    Eigen::Matrix<float, NP, 1> base = cumsum(pw * 0.0 +  Eigen::Matrix<float, NP, 1>::Ones()*1.0/NP) - Eigen::Matrix<float, NP, 1>::Ones()*1.0/NP;
    Eigen::Matrix<float, NP, 1> resampleid;
    Eigen::Matrix<float, 4, NP> output;
    for(int j=0; j<pw.rows(); j++){
      resampleid(j) = base(j) + uni_d(gen)/NP;
    }

    int ind = 0;

    for(int i=0; i<NP; i++){
        while(resampleid(i) > wcum(ind) && ind<NP-1){
          ind += 1;
        }
        output.col(i) = px.col(ind);
    }

    px = output;
    pw = Eigen::Matrix<float, NP, 1>::Ones()*1.0/NP;
  }
};

cv::Point2i cv_offset(
    Eigen::Vector2f e_p, int image_width=2000, int image_height=2000){
  cv::Point2i output;
  output.x = int(e_p(0) * 100) + image_width/2;
  output.y = image_height - int(e_p(1) * 100) - image_height/3;
  return output;
};

void ellipse_drawing(
    cv::Mat bg_img, Eigen::Matrix2f pest, Eigen::Vector2f center,
    cv::Scalar ellipse_color={0, 0, 255}){
  Eigen::EigenSolver<Eigen::Matrix2f> ces(pest);
  Eigen::Matrix2f e_value = ces.pseudoEigenvalueMatrix();
  Eigen::Matrix2f e_vector = ces.pseudoEigenvectors();

  double angle = std::atan2(e_vector(0, 1), e_vector(0, 0));
  cv::ellipse(
    bg_img,
    cv_offset(center, bg_img.cols, bg_img.rows),
    cv::Size(e_value(0,0)*1000, e_value(1,1)*1000),
    angle / PI * 180,
    0,
    360,
    ellipse_color,
    2,
    4);
};

int main(){
  float time=0.0;

  // control input
  Eigen::Vector2f u;
  u<<1.0, 0.1;

  // nosie control input
  Eigen::Vector2f ud;

  // observation z
  std::vector<Eigen::RowVector3f> z;

  // RFID remarks
  Eigen::Matrix<float, 4, 2> RFID;
  RFID<<10.0, 0.0,
        10.0, 10.0,
        0.0,  15.0,
        -5.0, 20.0;

  // dead reckoning
  Eigen::Vector4f xDR;
  xDR<<0.0,0.0,0.0,0.0;

  // ground truth reading
  Eigen::Vector4f xTrue;
  xTrue<<0.0,0.0,0.0,0.0;

  // Estimation
  Eigen::Vector4f xEst;
  xEst<<0.0,0.0,0.0,0.0;

  std::vector<Eigen::Vector4f> hxDR;
  std::vector<Eigen::Vector4f> hxTrue;
  std::vector<Eigen::Vector4f> hxEst;

  Eigen::Matrix4f PEst = Eigen::Matrix4f::Identity();

  // Motional model covariance
  float Q = 0.01;

  // Observation model covariance
  Eigen::Matrix2f  R = Eigen::Matrix2f::Identity();
  R(0,0)=1.0;
  R(1,1)=40.0/180.0 * PI * 40.0/180.0 * PI;

  // Motion model simulation error
  float Qsim = 0.04;

  // Observation model simulation error
  Eigen::Matrix2f Rsim = Eigen::Matrix2f::Identity();
  Rsim(0,0)=1.0 * 1.0;
  Rsim(1,1)=30.0/180.0 * PI * 30.0/180.0 * PI;

  // particle stor
  Eigen::Matrix<float, 4, NP> px = Eigen::Matrix<float, 4, NP>::Zero();

  Eigen::Matrix<float, NP, 1> pw = Eigen::Matrix<float, NP, 1>::Ones() * 1.0/NP;

  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<> gaussian_d{0,1};
  std::random_device rd2{};
  std::mt19937 gen2{rd2()};
  std::uniform_real_distribution<> uni_d{1.0, 2.0};

  //for visualization
  cv::namedWindow("pf", cv::WINDOW_NORMAL);
  int count = 0;

  while(time <= SIM_TIME){
    time += DT;

    ud(0) = u(0) + gaussian_d(gen) * Rsim(0,0);
    ud(1) = u(1) + gaussian_d(gen) * Rsim(1,1);

    xTrue = motion_model(xTrue, u);
    xDR = motion_model(xDR, ud);

    z.clear();
    for(int i=0; i<RFID.rows(); i++){
      float dx = xTrue(0) - RFID(i, 0);
      float dy = xTrue(1) - RFID(i, 1);
      float d = std::sqrt(dx*dx + dy*dy);
      if (d <= MAX_RANGE){
        float dn = d + gaussian_d(gen) * Qsim;
        Eigen::RowVector3f zi;
        zi<<dn, RFID(i, 0), RFID(i, 1);
        z.push_back(zi);
      }
    }

    pf_localization(px, pw, xEst, PEst, z, u, Rsim, Q, gen, gaussian_d);
    resampling(px, pw, gen2, uni_d);

    // TODO visualization
    hxDR.push_back(xDR);
    hxTrue.push_back(xTrue);
    hxEst.push_back(xEst);

    // visualization
    cv::Mat bg(3500,3500, CV_8UC3, cv::Scalar(255,255,255));
    for(unsigned int j=0; j<hxDR.size(); j++){

      // // green groundtruth
      cv::circle(bg, cv_offset(hxTrue[j].head(2), bg.cols, bg.rows),
                 7, cv::Scalar(0,255,0), -1);

      // blue estimation
      cv::circle(bg, cv_offset(hxEst[j].head(2), bg.cols, bg.rows),
                 10, cv::Scalar(255,0,0), 5);

      // black dead reckoning
      cv::circle(bg, cv_offset(hxDR[j].head(2), bg.cols, bg.rows),
                 7, cv::Scalar(0, 0, 0), -1);
    }


    for(int j=0; j<px.cols(); j++){
      cv::circle(bg, cv_offset(px.col(j).head(2), bg.cols, bg.rows),
                 3, cv::Scalar(0, 0, 255), -1);
    }

    for(int i=0; i<RFID.rows(); i++){
      cv::circle(bg, cv_offset(RFID.row(i), bg.cols, bg.rows),
                 20, cv::Scalar(127, 0, 255), -1);
    }
    for(unsigned int i=0; i<z.size(); i++){
      cv::line(
        bg,
        cv_offset(z[i].tail(2), bg.cols, bg.rows),
        cv_offset(hxEst.back().head(2), bg.cols, bg.rows),
        cv::Scalar(0, 0, 0),
        5);
    }

    ellipse_drawing(bg, PEst.block(0,0,2,2), xEst.head(2));

    cv::imshow("pf", bg);
    cv::waitKey(5);

    // std::string int_count = std::to_string(count);
    // cv::imwrite("./pngs/"+std::string(5-int_count.length(), '0').append(int_count)+".png", bg);

    count++;
  }
}
