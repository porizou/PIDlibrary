#ifndef PIDcontroller_H
#define PIDcontroller_H

/**
 * Includes
 */
 #include "mbed.h"

 /**
 * Defines
 */
 
 class PID {

 public:

    /**
     *Kp 比例ゲイン
     *Ki 積分ゲイン
     *Kd 微分ゲイン
     *tSample 制御周期
     */
     PID(float Kp, float Ki, float Kd, float tSample);

     /**
      *入力値の範囲を0~100%に置き換える
      *
      *InMin 入力の最小値 --> 0% 
      *InMax 入力の最大値 --> 100%
     */
     void setInputlimits(float inMin, float inMax);

     /**
      *出力値の範囲を0~100%に置き換える
      *
      *InMin 出力の最小値 --> 0% 
      *InMax 出力の最大値 --> 100%
     */
     void setOutputlimits(float outMin, float outMax);

     //目標値をセット
     void setSetPoint(float sp);

     //現在値をセット  
     void setProcessValue(float pv);

     void reset(float pv);

     /**
      *PIDの計算
      *
      *戻り値はoutMinからoutMaxの範囲
     */
     float compute(void);

     //ゲッター
     float getInMin();
     float getInMax();
     float getOutMin();
     float getOutMax();
     float gettSample();
     float getKp();
     float getKi();
     float getKd();

 private:
    //PIDゲイン
    float Kp_;
    float Ki_;
    float Kd_;
    //目標値
    float setPoint_;
    //入力値
    float processVariable_;
    //前回の入力値
    float prevProcessVariable_;
    //出力値
    float controllerOutput_;
    //前回の出力値
    float prevControllerOutput_;

    float inMin_;
    float inMax_;
    float inSpan_;
    float outMin_;
    float outMax_;
    float outSpan_;

    //偏差の積分値
    float accError_;

    //現在の偏差
    float Error_;
    //前回の偏差
    float prevError_;
    
    //制御周期
    float tSample_;

 };

 #endif /* PIDcontroller_H */