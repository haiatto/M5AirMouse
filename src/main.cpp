#include <M5Unified.h>
#include <BleMouse.h>
#include <math.h>
#include <Preferences.h>

// 複数台ペアリング対応してみた版
class BleMouseMod : public BleMouse {
public:
    using BleMouse::BleMouse;

    ble_addr_t GetCurrendPeerIdAddr() const 
    { 
      return currentPeerIdAddr_;
    }

    void SetPairingPeerIdAddr(const ble_addr_t& addr) 
    { 
      bHasPairingAddr_ = true;
      pairingIdAddr_ = addr;
    }

    void ResetPairingPeerIdAddr() 
    { 
      bHasPairingAddr_ = false;
    }
    void SetDeviceName(std::string inDeviceName)
    {
      this->deviceName = inDeviceName;
    }

protected:
    virtual void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) override 
    {
      if(bHasPairingAddr_)
      {
        if(ble_addr_cmp(&pairingIdAddr_, &desc->peer_id_addr) != 0)
        {
          USBSerial.printf("BLE difference target %s \n", NimBLEAddress(desc->peer_id_addr).toString().c_str());
          pServer->disconnect(desc->conn_handle);
          return;
        }
      }
      currentPeerIdAddr_ = desc->peer_id_addr;
      pServer->stopAdvertising();
      USBSerial.printf("BLE connect %s \n", NimBLEAddress(desc->peer_id_addr).toString().c_str());
    }

    virtual void onDisconnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) override {
      USBSerial.printf("BLE disconnect %s \n", NimBLEAddress(desc->peer_id_addr).toString().c_str());
      if(bHasPairingAddr_)
      {
        if(ble_addr_cmp(&pairingIdAddr_, &desc->peer_id_addr) == 0)
        {
          pServer->startAdvertising();
        }
      }
    }
    ble_addr_t currentPeerIdAddr_;
    bool bHasPairingAddr_ = false;
    ble_addr_t pairingIdAddr_;
};


// BLE Mouse インスタンスの生成
BleMouseMod bleMouse("M5AtomMouse#dummy", "M5Stack", 100);  // (デバイス名, ベンダー名, バッテリーレベル)

const uint8_t PIN_BTN_BLUE = 1;
const uint8_t PIN_BTN_RED = 2;

// ジャイロによる移動感度
const float gyroSensitivity = 20.0;

float gravityX = 0.0;
float gravityY = 0.0;
float gravityZ = 0.0;

void bleConnectProcessInit();
bool bleConnectProcess();

void setup() {

  auto cfg = M5.config();
  M5.begin(cfg);

  pinMode(PIN_BTN_RED,INPUT);
  pinMode(PIN_BTN_BLUE,INPUT);

  USBSerial.begin();
  //delay(4000);//最初の通信が抜ける？

  if (!M5.Imu.begin()) {
    USBSerial.println("IMU initialization failed!");
  }
  bleConnectProcessInit();

  USBSerial.println("start!");
}


float lastTheta = 0;
void loop() {
  M5.update();

  if(!bleConnectProcess())
  {
    delay(100);
    return;
  }

  // 加速度センサの値取得（重力＋運動加速度）
  float ax, ay, az;
  M5.Imu.getAccelData(&ax, &ay, &az);

  // ジャイロセンサの値取得（単位は一般的に度/秒）
  float gx, gy, gz;
  M5.Imu.getGyroData(&gx, &gy, &gz);

  // dt: ループ周期は約20ms（0.02秒） ※ここを速くしてもBLE的に無理っぽいので通信遅延が起こる雰囲気になり反応が遅くなる
  const int DeltaMs = 20;
  const float dt = DeltaMs/1000.0;

  int gyroMoveX = (int)(gx * gyroSensitivity * dt);
  int gyroMoveY = (int)(gy * gyroSensitivity * dt);
  int gyroMoveZ = (int)(gz * gyroSensitivity * dt);

  // 両方の移動値を合算してマウス操作化
  // M5StampS3Rは、机に置いたときの、右方向がX軸、前方向がY軸、画面下方向にZ軸
  // ジャイロのY軸は、マウスのXの移動にのみ使用。これは手首でドライバーをねじるような回転操作
  // ジャイロのXとZ軸は、レーザーポインタ的な意味の操作。ただしマウスとして扱うものなの手首だけ使って指先で絵を描くような操作感

  //MEMO:
  // 重力方向をもとに操作したほうが直感的になるみたい
  // しかしジャイロのみの方が正確性があり、どうもマウスの操作に近いのか人側の経験が反映されるように感じる(かも？)
  // カルマンフィルタなども試したけれど、やり方の問題もある可能性はありつつ、
  // 移動しながらの重力の補正は正確性が犠牲になって操作が気持ち良くない感じに
  // カーソル操作開始ボタンONの時点を基準にするだけでそんな違和感がなく直感的と正確性が出る感じがする…
  // ある意味マウスを握ったときの様な感じもするので、マウス操作に人が慣れてるからの可能性ありそう…かも？

  int totalMoveX =0;
  int totalMoveY =0;

  if((digitalRead(PIN_BTN_RED)==0)){
    // 手首の動きで指先を動かしたような感じのポインタ移動をする（ただしあくまでマウス操作なので移動ベクトルの平面化投影などの補正は不要）
    // ジャイロのY軸回転とZ軸回転を基にする

    // 1. 基準の方向（最後の重力方向）からYZ平面での角度を算出
    // 2. ジャイロによる移動量（YZ成分）の符号調整済みベクトル
    // 3. theta で回転させる（重力方向を垂直に合わせる）
    // 4. ジャイロのX軸（手首の捻り）はそのままX移動として加算
    float theta = lastTheta;
    float r_x = -gyroMoveX;
    float r_z = -gyroMoveZ;
    int adjustedVertical = (int)(r_x * cos(theta) + -r_z * sin(theta));
    int adjustedHorizontal = (int)(r_x * sin(theta) + r_z * cos(theta));
    totalMoveY = adjustedVertical;
    totalMoveX = gyroMoveY + adjustedHorizontal;
  }
  else{
    lastTheta = atan2(ax, az);
  }
  //USBSerial.printf("lastTheta %f ax=%f ay=%f az=%f gx=%f gy=%f gz=%f\n",lastTheta, ax,ay,az, gx,gy,gz);
  
  if (bleMouse.isConnected()) {
    bleMouse.move(totalMoveX, totalMoveY, 0);
  }

  // 左クリック
  if (bleMouse.isConnected()) {
    if(digitalRead(PIN_BTN_BLUE)==0){
      bleMouse.press(MOUSE_LEFT);
    }
    else{
      bleMouse.release(MOUSE_LEFT);
    }
  }
  
  delay(DeltaMs);  // ループ周期の調整
}


// BLE接続まで

// アドレス切り替え
// https://qiita.com/mgf/items/a9e462f3b2ab35903de6 を参考に
static uint8_t org_mac[6] = {0};
void initMacAddr()
{
  esp_efuse_mac_get_default(org_mac);
}
void changeMacAddr(int connectNo)
{
  uint8_t new_mac[6];
  for (int i = 0; i < 6; i++) {
    new_mac[i] = org_mac[i];
  }
  new_mac[5] += (4 * connectNo);
  esp_base_mac_addr_set(new_mac);
}


#define NUM_SLOTS 5
struct PairingSlot {
  bool paired;
  ble_addr_t peerAddr;
};
PairingSlot pairingSlots[NUM_SLOTS];
int activeSlot = 0;

const char* deviceNames[NUM_SLOTS] = {
  "M5AtomMouse#0",
  "M5AtomMouse#1",
  "M5AtomMouse#2",
  "M5AtomMouse#3",
  "M5AtomMouse#4",
};

Preferences prefs;  // NVS 用
void PrefLoad()
{
   // 情報読み込み
    prefs.begin("pairing", false);
    bool bReadOk=false;
    if ( prefs.isKey("pairing_slots") )
    {
      int size = prefs.getBytes("pairing_slots", (uint8_t*)&pairingSlots, sizeof(pairingSlots));
      if(size==sizeof(pairingSlots)){
        bReadOk=true;
      }
    }
    if(!bReadOk)
    {
        memset(pairingSlots,0,sizeof(pairingSlots));
        prefs.putBytes("pairing_slots", (uint8_t*)&pairingSlots, sizeof(pairingSlots));
    }
    activeSlot = prefs.getInt("active_slot",0);
    prefs.end();
}
void PrefSave()
{
    prefs.begin("pairing", false);
    prefs.putBytes("pairing_slots", (uint8_t*)&pairingSlots, sizeof(pairingSlots));
    prefs.putInt("active_slot",activeSlot);
    prefs.end();
}

int lastBtnRed = 0;
int lastBtnBlue = 0;
int lastBtnTimer=0;

enum class BLE_CON_STATE
{
  PreStart,
  SelSlot,
  StartSlot,
  PairingOrConnecting,
  Connected,
  ConnectedIdle,
};
BLE_CON_STATE BleConStat = BLE_CON_STATE::PreStart;

void bleConnectProcessInit()
{
  initMacAddr();
  BleConStat = BLE_CON_STATE::PreStart;
}

bool bleConnectProcess()
{
  bool bContinue=false;
  do 
  {
    bContinue = false;
    switch (BleConStat)
    {
    case BLE_CON_STATE::PreStart:
      {
        M5.Lcd.powerSaveOff();
        M5.Lcd.setBrightness(100);

        lastBtnRed = digitalRead(PIN_BTN_RED); 
        lastBtnBlue = digitalRead(PIN_BTN_BLUE); 

        // 情報読み込み
        PrefLoad();

        // 画面押していたら or スロットに何もないならスロット選択へ
        if(M5.BtnA.isPressed() || !pairingSlots[activeSlot].paired)
        {
          BleConStat = BLE_CON_STATE::SelSlot;
          bContinue = true;
        }
        else{
          // 既存の設定を接続対象としてセットして開始
          BleConStat = BLE_CON_STATE::StartSlot;
          bContinue = true;
        } 
      }
      break;
    case BLE_CON_STATE::SelSlot:
      {
        // PIN_BTN_RED スロットを切り替え
        // PIN_BTN_BLUE で決定
        // PIN_BTN_BLUE を長押しした場合そのスロットのペアリング情報を消す
        M5.Lcd.fillScreen(DARKGREEN);
        M5.Lcd.setTextSize(1);

        M5.Lcd.setCursor(0, 0);
        M5.Lcd.printf("SelSlot: %d\n", activeSlot);
        if (pairingSlots[activeSlot].paired) {
          auto addr=NimBLEAddress(pairingSlots[activeSlot].peerAddr).toString();
          M5.Lcd.printf("Paired: %s\n", addr.c_str());
        } else {
          M5.Lcd.println("Not paired");
        }
        M5.Lcd.println("");
        M5.Lcd.printf("Red Btn:  Slot++\n");
        M5.Lcd.printf("Blue Btn: Decide\n");
        M5.Lcd.printf("Blue Btn(long):\n ErasePairing\n");

        int curBtnRed = digitalRead(PIN_BTN_RED); 
        int curBtnBlue = digitalRead(PIN_BTN_BLUE); 
        
        
        if((curBtnRed!=0) && (lastBtnRed==0)) {
          activeSlot = activeSlot = (activeSlot+1) % NUM_SLOTS;
        }
        lastBtnRed = curBtnRed;

        if((curBtnBlue==0)&&(lastBtnBlue!=0)){
          lastBtnTimer = millis();
        }
        else if((curBtnBlue==0) && (lastBtnBlue==0)) {
          if((millis()-lastBtnTimer) > 5000 && pairingSlots[activeSlot].paired){
            pairingSlots[activeSlot].paired = false;
            PrefSave();
          }
        }
        else if((curBtnBlue!=0) && (lastBtnBlue==0)) {
          if((millis()-lastBtnTimer) > 5000){
          }
          else{
            if(pairingSlots[activeSlot].paired){
              bleMouse.SetPairingPeerIdAddr(pairingSlots[activeSlot].peerAddr);
            }
            PrefSave();
            BleConStat = BLE_CON_STATE::StartSlot;
            bContinue = true;
          }
        }
        lastBtnBlue = curBtnBlue;
      }
      break;
    case BLE_CON_STATE::StartSlot:
      {
        //マックアドレスとデバイス名変更
        changeMacAddr(activeSlot);
        bleMouse.SetDeviceName(deviceNames[activeSlot]);

        bleMouse.begin();
        BleConStat = BLE_CON_STATE::PairingOrConnecting;
        bContinue = true;
      }
      break;
    case BLE_CON_STATE::PairingOrConnecting:
      {
        USBSerial.printf("Stat PairingOrConnecting\n");

        M5.Lcd.fillScreen(DARKCYAN);
        M5.Lcd.setTextSize(1);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.printf("Slot: %d\n", activeSlot);
        if (bleMouse.isConnected()) {
          if(!pairingSlots[activeSlot].paired){
            pairingSlots[activeSlot].peerAddr = bleMouse.GetCurrendPeerIdAddr();
            pairingSlots[activeSlot].paired = true;
            PrefSave();
          }
          if(pairingSlots[activeSlot].paired)
          {
            auto addr=bleMouse.GetCurrendPeerIdAddr();
            if(ble_addr_cmp(&addr, &pairingSlots[activeSlot].peerAddr)==0)
            {
              BleConStat = BLE_CON_STATE::Connected;
              bContinue = true;
            }
          }
        } 
        else if (pairingSlots[activeSlot].paired) {
          auto addr=NimBLEAddress(pairingSlots[activeSlot].peerAddr).toString();
          M5.Lcd.printf("Wait Connecting...:\n%s\n", addr.c_str());
        } else {
          M5.Lcd.println("Wait Paring...");
        }
        M5.Lcd.println("");
        M5.Lcd.println("BtnA(Screen):Return Sel");
        if(M5.BtnA.isPressed())
        {
          BleConStat = BLE_CON_STATE::SelSlot;
          bContinue = true;
        }
      }
      break;
    case BLE_CON_STATE::Connected:
      {
        USBSerial.printf("Stat Connected\n");
        M5.Lcd.fillScreen(BLACK);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.powerSaveOn();
        M5.Lcd.setBrightness(0);
        BleConStat = BLE_CON_STATE::ConnectedIdle;
      }
      break;
    case BLE_CON_STATE::ConnectedIdle:
      {
        //TODO:切断時の再アドバタイズ
        //USBSerial.printf("Stat Connected Idle\n");
      }
      break;
    }
  }
  while(bContinue);

  return BleConStat == BLE_CON_STATE::ConnectedIdle;
}

//-------------