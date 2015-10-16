/**
 * Includes
 */
 #include "PIDcontroller.h"

 PID::PID(float Kp, float Ki, float Kd, float tSample){

    setInputLimits(0.0, 3.3);
    setOutputLimits(0.0, 3.3);

    tSample_ = tSample;

    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;

    setPoint_             = 0.0;
    processVariable_      = 0.0;
    prevProcessVariable_  = 0.0;
    controllerOutput_     = 0.0;
    prevControllerOutput_ = 0.0;

    accError_ = 0.0;
    Error_    = 0.0;
    prevError_= 0.0;
 }

 void PID::setInputLimits(float inMin, float inMax){

    //不可能な値になっていないか確認
    if(inMin >= inMax){
        return;
    }

    //0~100%に置き換える
    prevProcessVariable_ *= (inMax - inMin) / inSpan_;
    accError_            *= (inMax - inMin) / inSpan_;

    //0~1の範囲を超えていたら0~1にする
    if(prevProcessVariable_ > 1){
        prevProcessVariable_ = 1;
    } else if (prevProcessVariable_ < 0){
        prevProcessVariable_ = 0;
    }

    inMin_  = inMin;
    inMax_  = inMax;
    inSpan_ = inMax - inMin;

 }

 void PID::setOutputLimits(float outMin, float outMax){

    //不可能な値になっていないか確認
    if(outMin >= outMax){
        return;
    }

    prevControllerOutput_ *= (outMax - outMin) / outSpan_;

    //0~1の範囲を超えていたら0~1にする
    if(prevControllerOutput_ > 1){
        prevControllerOutput_ = 1;
    } else if (prevControllerOutput_ < 0){
        prevControllerOutput_ = 0;
    }

    outMin_  = outMin;
    outMax_  = outMax;
    outSpan_ = outMax - outMin;

 }

 void PID::setSetPoint(float sp){

    setPoint_ = sp;

 }

 void PID::setProcessValue(float pv){

    processVariable_ = pv;

 }

 void PID::reset(float pv){

    //現在値と目標値の値を0~100%の範囲に置き換える
    float scaledPV = (processVariable_ - inMin_) / inSpan_;

    if(scaledPV > 1.0){
        scaledPV = 1.0;
    } else if(scaledPV < 0.0){
        scaledPV = 0.0;
    }

    float scaledSP = (setPoint_ - inMin_) / inSpan_;

    if (scaledSP > 1.0){
        scaledSP = 1.0;
    } else if(scaledSP < 0.0){
        scaledSP = 0.0;
    }

    prevError_ = scaledSP - scaledPV;

 }

 float PID::compute(){

    //現在値と目標値の値を0~100%の範囲に置き換える
    float scaledPV = (processVariable_ - inMin_) / inSpan_;

    if(scaledPV > 1.0){
        scaledPV = 1.0;
    } else if(scaledPV < 0.0){
        scaledPV = 0.0;
    }

    float scaledSP = (setPoint_ - inMin_) / inSpan_;

    if (scaledSP > 1.0){
        scaledSP = 1.0;
    } else if(scaledSP < 0.0){
        scaledSP = 0.0;
    }

    //偏差の計算
    Error_= scaledSP - scaledPV;

    //アンチワインドアッップ　-- 前回の出力値が0~1を超えていたら偏差を積分しない
    if (!(prevControllerOutput_ >= 1 && Error_ > 0) && !(prevControllerOutput_ <= 0 && Error_ < 0)) {
        //偏差の積分値の計算(台形近似法)
        accError_ += (Error_ + prevError_) / 2 * tSample_;
    }

    //偏差の微分値の計算
    float de = (Error_ - prevError_) / tSample_;

    //偏差の1サンプル過去の値を更新
    prevError_ = Error_;

    //PIDの計算
    controllerOutput_ = Kp_ * Error_ + Ki_ * accError_ + Kd_ * de; 

    /*アンチワインドアップ -- 出力が0~1を超えていたら積分項を減衰させる
    if(controllerOutput_ > 1.0){
        accError_ -= (controllerOutput_ - 1.0) / Ki_; //積分項をー側に減衰
        if(accError_ < 0.0) { accError_ = 0.0; } //積分項が負になったら0にする
        controllerOutput_ = 1.0; //出力が1.0を超えたら1にする
    } else if(controllerOutput_ < 0.0){
        accError_ -= (controllerOutput_ + 1.0) / Ki_; //積分項を＋側に減衰
        if(accError_ > 0.0) { accError_ = 0.0; } //積分項が正になったら0にする
        controllerOutput_ = 0.0; //出力が0.0を下回ったら0にする
    }*/

    //出力の値を更新
    prevControllerOutput_ = controllerOutput_;

    if (controllerOutput_ < 0.0) {
        controllerOutput_ = 0.0;
    } else if (controllerOutput_ > 1.0) {
        controllerOutput_ = 1.0;
    }
 
    //PIDの出力を実際の値に変換して返す
    return ((controllerOutput_ * outSpan_) + outMin_);

 }

 float PID::getInMin(){

    return inMin_;
 }

 float PID::getInMax(){

    return inMax_;
 }

 float PID::getOutMin(){

    return outMin_;
 }

 float PID::getOutMax(){

    return outMax_;
 }

 float PID::gettSample(){

    return tSample_;
 }

 float PID::getKp(){

    return Kp_;
 }

 float PID::getKi(){

    return Ki_;
 }

 float PID::getKd(){

    return Kd_;
 }
























