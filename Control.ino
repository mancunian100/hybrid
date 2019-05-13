/*
   主控制程序
   版本：1.0
   程序猿：曾铮、于曹阳、卢迪、周赫雄
   单位：上海交通大学——海洋学院
   更新日期：20190414
   描述：
   1.主控制器（以下简称主机）接收辅助控制器（以下简称从机）数据和上位机数据，做出逻辑控制行为；
   2.主机利用硬件串口接收从机数据，利用软串口接收上位机指令；
   3.
*/
#include <PWM.h>
#include <ComTranslate.h>
#include <PID_v1.h>
#include <DFRobot_I2CMultiplexer.h>
#include <DFRobot_ADS1115.h>
#include <Wire.h>
#include "JY901.h"
#include "MS5837.h"

// 声明匀速运动压强差
#define ConstantSurface 50
#define ConstantDiving 50

//声明定深目标深度和误差容许范围
#define R 0.05


//判断标识集合
boolean data_receive_Completed = false; //指示是否完成读取串口数据
boolean com_receive_Completed = false;  //指示是否完成读取软串口指令
boolean data_request = false; //true——代表给上位机发送数据；false——代表停止给上位机发送数据
boolean flag_rotate = false; //true——代表旋转质量块；false——代表停止旋转质量块
int flag_seal = 0;  //0——代表停止动作活塞；1——代表活塞往回收密封；2——代表活塞往外伸通气；3——代表当在1或2的前提下堵转过流时进入过流超时保护控制
int type = 0; //任务类型标识符
int temp = 0;
int state = 0;

#define deflate 2   //D2连接控制放气阀的继电器
#define inflate 3   //D3连接控制充气阀的继电器
#define IB1 4   //D4连接横滚电机驱动板的IB1引脚——控制通气活塞
#define IB2 5   //D5(PWM)连接横滚电机驱动板的IB2引脚——控制通气活塞
#define IA1 6   //D6(PWM)连接横滚电机驱动板的IA1引脚——控制横滚
#define IA2 7   //D7连接横滚电机驱动板的IA2引脚——控制横滚
#define arm 9   //D9连接机臂折叠控制板信号线
#define reset 11  //D11连接RST引脚重启
#define reset_infor 12  //D12连接飞控F2口做重启信号接收

//姿态仪和内外压
float NZ3_roll = 0;
float NZ3_pitch = 0;
float NZ3_yaw = 0;
float Out_pressure = 0;
float In_pressure = 0;
float Depth = 0;
float Dd = 0;
float Dcor = 0;

// districtions for safety
float deltaPressMax = 200;                              // allowed max delta pressure
float deltaPressMinFloatUp = 100;                       // min delta pressure for floating up
float deltaPressMaxSinkDown = 100;                      // max delta pressure for sinking down
float deltaPressMinSinkDown = 50;                       // min delta pressure for sinking down

unsigned long maxTime = 120000;                         // max time for the cycle

//旋转质量块
float Angle = 0;
float Angle_d = 0;
float Deta_Angle = 0;

//通气活塞
float Amp = 0;
float Acor = 0;
unsigned long OCStartTime = 0;

//折叠机臂
int32_t frequency = 50; //frequency (in Hz)

//定深PID控制参数设置
double Setpoint, Input, Output;
double Step = 100;                      // 步长0.5s
double Epsilon = 0.05;                  // 定深误差容许阈（单位：m）
double downSetpoint, upSetpoint;        // 定深上下阈值
double Kp = 2, Ki = 1, Kd = 5;          // 参数待定
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//航向控制S面控制参数
float S1 = 0, S2 = 0;

//滑翔+航点模式任务参数
float Glide_maxDepth = 0, Glide_minDepth = 0,  Glide_Pitch = 0, WP_Lon = 0, WP_Lat = 0;
int Glide_Number = 0, CycleNumberCounter = 0;

//浮标剖面模式任务参数
float Float_maxDepth = 0, Float_minDepth = 0;
int Float_Number = 0;

//浮标定深模式任务参数
float Float_TargetDepth = 0, Float_MissionTime = 0, Float_OverTime = 0;

//手动模式任务参数
String Manual = "";

//数据和指令接收与发送
String Com_String = "";//用一个string类型变量去接收任意长度的上位机指令
String data = "";

//发送数据计时器
unsigned long StartTime = 0;
int TimeSpan = 1000;

//重启通道PWM值
unsigned long rst = 0;

//I2C通道多选模块对象建立，默认地址0x70
DFRobot_I2CMultiplexer I2CMulti(0x70);

//16-bit ADC模块对象建立
DFRobot_ADS1115 ads;

MS5837 In_sensor; //内部气压计——对应I2CMultiplexer的Port 1
MS5837 Out_sensor;//外部深度计——对应I2CMultiplexer的Port 0

//初始化
void setup()
{
  //初始化rst口
  digitalWrite(reset, HIGH);

  StartTime = millis();

  //串口初始化
  Serial.begin(57600);
  Wire.begin();

  //初始化外部深度计
  I2CMulti.selectPort(0);//I2C通道选择模块切换至Port 0与外部深度计通讯
  while (!Out_sensor.init()) {
    delay(500);
  }
  Out_sensor.setModel(MS5837::MS5837_30BA);
  Out_sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
  //上电初始化时校准深度零位
  for (int i = 0; i < 100; i++) {
    Out_sensor.read();
    Depth = Out_sensor.depth();
    Dcor = 0 - Depth;
  }

  //初始化内部气压计
  I2CMulti.selectPort(1);//I2C通道选择模块切换至Port 1与内部气压计通讯
  while (!In_sensor.init()) {
    delay(500);
  }
  In_sensor.setModel(MS5837::MS5837_30BA);
  In_sensor.setFluidDensity(1.293); // kg/m^3 (freshwater, 1029 for seawater)

  //初始化姿态仪
  I2CMulti.selectPort(2); //I2C通道选择模块切换至Port 2与姿态仪通讯
  JY901.StartIIC();

  //初始化16位ADC采样模块
  I2CMulti.selectPort(3); //I2C通道选择模块切换至Port 3与16位ADC模块通讯
  ads.setAddr_ADS1115(ADS1115_IIC_ADDRESS0);   // ADS1115_IIC_ADDRESS0 -> 0x48; ADS1115_IIC_ADDRESS1 -> 0x49
  ads.setGain(eGAIN_TWOTHIRDS);   // 2/3x gain
  ads.setMode(eMODE_SINGLE);       // single-shot mode
  ads.setRate(eRATE_128);          // 128SPS (default)
  ads.setOSMode(eOSMODE_SINGLE);   // Set to start a single-conversion
  ads.init();
  //初始化电流计0位
  if (ads.checkADS1115())
  {
    int16_t adc1;
    adc1 = ads.readVoltage(1);
    Amp = (adc1 - 2500.00) / 185; //通气活塞电流A
    Acor = 0 - Amp;
  }

  //IO口初始化
  pinMode(deflate, OUTPUT);
  pinMode(inflate, OUTPUT);
  pinMode(IA1, OUTPUT);
  pinMode(IA2, OUTPUT);
  pinMode(IB1, OUTPUT);
  pinMode(IB2, OUTPUT);
  pinMode(arm, OUTPUT);
  pinMode(reset_infor, INPUT);
  pinMode(reset, OUTPUT);

  //定深PID初始化
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(100);

  //initialize all timers except for 0, to save time keeping functions
  InitTimersSafe();

  //sets the frequency for the specified pin
  bool success = SetPinFrequencySafe(arm, frequency);

  //if the pin frequency was set successfully, turn pin 13 on
  if (success) {
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
  }



}

//循环体
void loop()
{
  //1.数据采集
  Sensors();
  //2.接受上位机指令
  if (com_receive_Completed) {       //判断上位机从串口发来的指令是否读取完
    type = ComTranslate.Decode(Com_String);
    if (type == 1) {
      ComTranslate.ExtractPar_mode1(Kp, Ki, Kd, S1, S2);
    }
    if (type == 2) {
      ComTranslate.ExtractPar_mode2(Glide_maxDepth, Glide_minDepth, Glide_Number, Glide_Pitch);
    }
    if (type == 3) {
      ComTranslate.ExtractPar_mode3(Glide_maxDepth, Glide_minDepth, Glide_Number, Glide_Pitch, WP_Lon, WP_Lat);
    }
    if (type == 4) {
      ComTranslate.ExtractPar_mode4(Float_maxDepth, Float_minDepth, Float_Number);
    }
    if (type == 5) {
      ComTranslate.ExtractPar_mode5(Float_TargetDepth, Float_MissionTime, Float_OverTime);
    }
    if (type == 6) {
      ComTranslate.ExtractPar_mode6(Manual);
      if (Manual.equals("rotate")) {
        ComTranslate.ExtractPar_mode6(Manual, Angle_d, Deta_Angle);
      }
    }
    Com_String = "";
    com_receive_Completed = false;
  }

  //3.type=2滑翔模式
  if (type == 2) {
    /*
      if (CycleNumberCounter != (2 * Glide_Number + 1)) {
      if (Depth <= Glide_minDepth) {
        digitalWrite(inflate, LOW);
        digitalWrite(deflate, HIGH);
        temp = state;
        state = 0;
        if ( (temp - state) == 1 ) {
          CycleNumberCounter = CycleNumberCounter + 1;
        }
      }
      if ( (Glide_minDepth + 0.5) <= Depth && Depth <= (Glide_maxDepth - 0.5) ) {
        digitalWrite(inflate, LOW);
        digitalWrite(deflate, LOW);
      }
      if (Depth >= Glide_maxDepth) {
        digitalWrite(inflate, HIGH);
        digitalWrite(deflate, LOW);
        temp = state;
        state = 1;
        if ( (temp - state) == -1 ) {
          CycleNumberCounter = CycleNumberCounter + 1;
        }
      }
      }
      if ( CycleNumberCounter == (2 * Glide_Number + 1) ) {
      if (In_pressure - Out_pressure >= 600) {
        digitalWrite(inflate, LOW);
        digitalWrite(deflate, LOW);
        type = 0;
      }
      }
    */
    bool flagCycle = true;                                  // whether the glider is in the cycle
    int cycleNumCounter = 0;                                // counter of cycle
    float deltaPress;                                       // the delta of pressure of inside & outside
    bool flagUpOrDown = true;                               // whether the glider is up or down
    int layer;                                              // which layer of depth glider is at
    float surfaceDepth = 0.2;                               // the depth of surface
    unsigned long surface_keepTime = 60000;                 // keep at surface time
    unsigned long glide_startTime = millis();               // time when start cycle
    unsigned long surface_startTime;                        // time when reach the surface
    while (true) {
      Sensors();
      // some flags
      flagCycle = cycleNumCounter <= Glide_Number;
      deltaPress = In_pressure - Out_pressure;

      // running time checking
      if (millis() - glide_startTime > maxTime) {
        flagCycle = false;
      }

      // which layer of depth
      if (Depth < Glide_minDepth) {
        layer = 1;
      } else if (Depth > Glide_maxDepth) {
        layer = 2;
      } else if (Depth >= Glide_minDepth && Depth <= Glide_maxDepth) {
        layer = 3;
      } else {
        // unkonw depth
        layer = 4;
      }

      // in the glider cycle
      if (flagCycle) {
        // print cycle times and Depth
        if (cycleNumCounter > 0) {
          Serial.print("running the ");
          Serial.print(cycleNumCounter);
          Serial.print(" time(s) cycle, Depth: ");
          Serial.print(Depth);
          Serial.print(" delta of prssure: ");
          Serial.println(deltaPress);
        }
        switch (layer) {

          case 1:
            if (flagUpOrDown) {
              cycleNumCounter += 1;
              if (cycleNumCounter > Glide_Number) {
                Serial.print("Finished ");
                Serial.print(Glide_Number);
                Serial.println(" times cycle, now float up to the surface.");
                surface_startTime = millis();
                break;  
              }
              Serial.print("Begin the ");
              Serial.print(cycleNumCounter);
              Serial.print(" time(s) cycle");
            }
            sinkDown(deltaPress);
            flagUpOrDown = false;
            // print cycle times
            break;
          case 2:
            floatUp(deltaPress);
            flagUpOrDown = true;
            break;
          case 3:
            if (flagUpOrDown) {
              floatUp(deltaPress);
            } else {
              sinkDown(deltaPress);
            }
            break;
          case 4:
            floatUp(deltaPress);
            break;
        }
      }
      // finished the glider cycle
      else {
        if (Depth <= surfaceDepth) {
          watching(deltaPress);
          type = 0;
          if (millis() - surface_startTime > surface_keepTime) {
            // keep enough time and turn off
            digitalWrite(deflate, LOW);
            digitalWrite(inflate, LOW);
            break;
          }
        } else {
          floatUp(deltaPress);
        }
      }
    }
  }

  //4.type=3航向模式

  //5.type=4浮标循环剖面模式

  //6.type=5浮标定深模式

  //7.type=6手动操作模式(包括手动充放气以及手动调节横滚机构)
  if (type == 6) {
    //7.1 手动充放气、停止
    if (Manual.equals("inflate")) {
      if (In_pressure <= 1400) {
        digitalWrite(inflate, HIGH);
        digitalWrite(deflate, LOW);
      }
    }
    if (Manual.equals("deflate")) {
      digitalWrite(inflate, LOW);
      digitalWrite(deflate, HIGH);
    }
    if (Manual.equals("stop1")) {
      digitalWrite(inflate, LOW);
      digitalWrite(deflate, LOW);
    }

    //7.2 手动旋转横滚机构
    if (Manual.equals("rotate")) {
      flag_rotate = true;
    }
    if (Manual.equals("stop2")) {
      digitalWrite(IA2, LOW);
      analogWrite(IA1, 0);
      flag_rotate = false;
    }
    if (flag_rotate) {
      float E_angle = Actuator();
    }


    //7.3发送与停止发送数据
    if (Manual.equals("sentdata")) {
      data_request = true;
    }
    if (Manual.equals("finishdata")) {
      data_request = false;
    }

    //7.4通气活塞
    if (flag_seal == 1 || Manual.equals("sealed")) {
      digitalWrite(IB1, HIGH);
      analogWrite(IB2, 0);
      flag_seal = 1;
    }
    if (flag_seal == 2 || Manual.equals("unsealed")) {
      digitalWrite(IB1, LOW);
      analogWrite(IB2, 255);
      flag_seal = 2;
    }
    if (abs(Amp) > 0.20 && (flag_seal == 1 || flag_seal == 2)) {
      OCStartTime = millis();
      flag_seal = 3;
    }
    if (flag_seal == 3) {
      if (millis() - OCStartTime > 2000) {
        digitalWrite(IB1, LOW);
        analogWrite(IB2, 0);
        flag_seal = 0;
      }
    }

    //7.5折叠机臂
    if (Manual.equals("folded")) {
      pwmWrite(arm, 25);
      Serial.println("folded");
    }
    if (Manual.equals("unfolded")) {
      pwmWrite(arm, 13);
      Serial.println("unfolded");
    }

    Manual = "";      //清空Manual字符串
  }

  /*

      //4.3定深子任务
      //Dd = (duration6 - 1005) * (0.3 - 0) / (2015 - 1005) + 0;
      //Setpoint = Dd;  //Setpoint需要从上位机中接收，放在2中完成，测试时循环中也需要
      //downSetpoint = Setpoint + Epsilon;  //放在2中完成，测试时循环中也需要
      //upSetpoint = Setpoint - Epsilon;  //放在2中完成，测试时循环中也需要
      if (duration5 > 1800) { //进入定深模式的指令判断
        int i = 1;
        while (i) {
          if (data_receive_Completed) { //判断串口是否接收到数据并完成读取
            for (int j = 0; j < 4; j++) {
              Unit_Buffer[j] = SerRea_Buffer[j + 16];
            }
            Depth = *(float *)Unit_Buffer;

            //记得标志位取反
            data_receive_Completed = false;
          }
          Input = Depth;

          //【调试阶段需要】定深循环过程中也需要不断接收上位机发来的目标深度设定数据以及退出定深指令
           //  读取Setpoint和退出指令。待写

          downSetpoint = Setpoint + Epsilon;      //放在2中完成，测试时循环中也需要
          upSetpoint = Setpoint - Epsilon;        //放在2中完成，测试时循环中也需要
          Serial.print("TargetDepth:");
          Serial.print(Dd);
          Serial.print("m.   Action:");
          if (Input > downSetpoint) {
            myPID.Compute();
            Serial.print("Up!");
            digitalWrite(deflate, LOW);
            digitalWrite(inflate, HIGH); //上升
            delay(Output / 255 * Step);
          }else if (Input < upSetpoint) {
            myPID.SetControllerDirection(REVERSE);
            myPID.Compute();
            Serial.print("Down!");
            digitalWrite(deflate, HIGH); //下降
            digitalWrite(inflate, LOW);
            delay(Output / 255 * Step);
          }else { // 此处具体选择不加  myPID.Compute(); 还是使用  myPID.SetMode(MANUAL); 待定
            Serial.print("Keep!");
            digitalWrite(inflate, LOW);
            digitalWrite(deflate, LOW);
            delay(Step);
          }
          if (duration5 < 1200) { //当接收到上位机的退出定深指令时，则跳出定深循环
            n--;
          }
          Serial.print("   Depth= ");
          Serial.print(Input);
          Serial.print("m   PID control output= ");
          Serial.println(Output);
          delay(100);
          serialEvent();
        }
      }
  */

  //8.数据上传
  if (millis() - StartTime > TimeSpan) {
    if (data_request == true) {
      data.concat(In_pressure);
      data.concat(",");
      data.concat(Out_pressure);
      data.concat(",");
      data.concat(Depth);
      data.concat(",");
      data.concat(NZ3_pitch);
      data.concat(",");
      data.concat(NZ3_roll);
      data.concat(",");
      data.concat(NZ3_yaw);
      data.concat(",");
      data.concat(Angle);
      data.concat(",");
      data.concat(CycleNumberCounter);
      data.concat(",");
      Serial.println(data);  //发送打包好的data数据体
      data = ""; //发送完了就清空data
    }
    StartTime += TimeSpan;
  }

  //9.Reset
  rst = pulseIn(reset_infor, HIGH);
  if (rst > 1800) {
    digitalWrite(reset, LOW);
  }
  else {
    digitalWrite(reset, HIGH);
  }

  /*
    if (millis() - StartTime > TimeSpan) {
      Serial.print("   Angle:");
      Serial.print(Angle);
      Serial.print("   Current:");
      Serial.print(Amp);
      Serial.print("   flag_rotate:");
      Serial.print(flag_rotate);
      Serial.print("   flag_seal:");
      Serial.print(flag_seal);
      Serial.print("   StartTime:");
      Serial.print(StartTime);
      Serial.print("   Manual:");
      Serial.print(Manual);
      Serial.println();
      StartTime += TimeSpan;
    }
  */
  delay(100);
}

/*******************************************
  Description: some inner helper functions fof inflating and deflating
  Author: chen qian
  Input: float Depth
  Output:
*******************************************/
// inner helper function for floating up
void floatUp(float delp) {
  Serial.println("floating up");
  if (delp > deltaPressMax) {
    digitalWrite(deflate, HIGH);
    digitalWrite(inflate, LOW);
    Serial.println("deflating...");
  } else if (delp < deltaPressMinFloatUp) {
    digitalWrite(deflate, LOW);
    digitalWrite(inflate, HIGH);
    Serial.println("inflating...");
  } else if (delp >= deltaPressMinFloatUp && delp <= deltaPressMax) {
    digitalWrite(deflate, LOW);
    digitalWrite(inflate, LOW);
  } else {
    // unkonwn deltaPress
    digitalWrite(deflate, HIGH);
    digitalWrite(inflate, LOW);
    Serial.println("unknown pressure, WARNING!!!");
  }
}

// inner helper function for sinking down
void sinkDown(float delp) {
  Serial.println("sinking down");
  if (delp > deltaPressMaxSinkDown) {
    digitalWrite(deflate, HIGH);
    digitalWrite(inflate, LOW);
    Serial.println("deflating...");
  } else if (delp < deltaPressMinSinkDown) {
    digitalWrite(deflate, LOW);
    digitalWrite(inflate, HIGH);
    Serial.println("inflating...");
  } else if (delp >= deltaPressMinFloatUp && delp <= deltaPressMax) {
    digitalWrite(deflate, LOW);
    digitalWrite(inflate, LOW);
  } else {
    // unkonwn deltaPress
    digitalWrite(deflate, HIGH);
    digitalWrite(inflate, LOW);
    Serial.println("unknown pressure, WARNING!!!");
  }
}

// inner helper function for watching deltaPress
void watching(float delp) {
  if (delp > deltaPressMax) {
    digitalWrite(deflate, HIGH);
    digitalWrite(inflate, LOW);
    Serial.println("deflating...");
  } else if (delp < deltaPressMinFloatUp) {
    digitalWrite(deflate, LOW);
    digitalWrite(inflate, HIGH);
    Serial.println("inflating...");
  } else {
    digitalWrite(deflate, LOW);
    digitalWrite(inflate, LOW);
  }
}

//串口中断接收（伪中断）
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    Com_String += inChar;
    if (inChar == '\\') {
      com_receive_Completed = true;
    }
  }
}

void Sensors() {
  //深度计及气压计接收，屏蔽了其他数据
  I2CMulti.selectPort(0);
  Out_sensor.read();
  Out_pressure = Out_sensor.pressure();
  Depth = Out_sensor.depth() + Dcor;

  I2CMulti.selectPort(1);
  In_sensor.read();
  In_pressure = In_sensor.pressure();

  //姿态仪角度接收，屏蔽了其他数据
  I2CMulti.selectPort(2);
  JY901.GetAngle();
  NZ3_roll = (float)JY901.stcAngle.Angle[0] / 32768 * 180;
  NZ3_pitch = (float)JY901.stcAngle.Angle[1] / 32768 * 180;
  NZ3_yaw = (float)JY901.stcAngle.Angle[2] / 32768 * 180;

  //角度计角度和电流计电流接收
  I2CMulti.selectPort(3);
  if (ads.checkADS1115())
  {
    int16_t adc0, adc1;
    adc0 = ads.readVoltage(0);
    Angle = adc0 / 4995.00 * 360; //NANO是10位采样精度，传感器本身12位精度，ADC模块是16位精度
    adc1 = ads.readVoltage(1);
    Amp = (adc1 - 2500.00) / 185 + Acor; //通气活塞电流A
  }
}

float Actuator()  {
  float e_angle = Angle - Angle_d;
  if (e_angle > Deta_Angle) {
    digitalWrite(IA2, LOW);
    analogWrite(IA1, 170);
  }
  else if (e_angle < -Deta_Angle) {
    digitalWrite(IA2, HIGH);
    analogWrite(IA1, 255 - 170);
  }
  else {
    digitalWrite(IA2, LOW);
    analogWrite(IA1, 0);
    flag_rotate = false;
  }
  return e_angle;
}
