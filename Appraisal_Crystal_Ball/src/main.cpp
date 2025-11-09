/*
球体POV Wifi OTAバージョン
 2025.8.8

 2025.8.19
  描画サイズ180x36
  ステッピングモータ追加

  C:\Users\ac\OneDrive\Documents\PlatformIO\Projects\POV_DMA_FS_SingleV00 _1009

  2025.8.28
   回転数　4800hz　12回/秒
   裏画面位置調整あり　frmVoffset,frmHofset

 360x72 bmp file A,B 2sets

 2025.10.26
  遊べるように自動モード追加



ファイル　アップロードコマンド
 pio run --target uploadfs
pio run --target uploadfs

*/

#include <WiFi.h>
#include <ArduinoOTA.h>
#include <LittleFS.h>
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include <Ticker.h>
#include "driver/spi_master.h"
#include "esp_heap_caps.h"

#include <PubSubClient.h>

// ステッピングモータ追加
#define PULSE_PIN 38
#define LEDC_CH 0
#define LEDC_TIMER 0
#define LEDC_BITS 12 // （100Hz〜3kHz）

// ホールスイッチ
#define PinHollSw 39

// ATOM S3R の DotStar LED ピン設定
#define PIN_MOSI 1 // DIN
#define PIN_SCLK 2 // CIN

#define GB 31 // 0..31 (SK9822 global brightness)
// #define NUMPIXELS 72  // Number of LEDs in strip
#define NUMPIXELS 72   // Number of LEDs in strip
#define Hs 180         // １周の分割数
#define Vs 36          // BMP画素 縦
#define LineTimeUs 740 // LEDリフレッシュ時間間隔[usec]
#define Brightness 31  // 16 // 64 // 20
#define TESTPin 8      // タイミング表示のため
#define BTNPin 41      // 表示切り替え

static spi_device_handle_t spi;

#define end_bytes int((NUMPIXELS + 15) / 16)
#define frame_bytes (4 + NUMPIXELS * 4 + end_bytes)

DMA_ATTR uint8_t frame[frame_bytes]; // DMA可能領域に配置

void loop_run(void *arg); // loopタスク

// MQTT
#include <WiFi.h>
#include <PubSubClient.h>

//////////////////// MQTT設定 ////////////////////
const char *mqtt_server = "heavywing737.cloud.shiftr.io";
const int mqtt_port = 1883;
const char *mqtt_user = "heavywing737";
const char *mqtt_password = "uqVoMM5kusZ0FMTH";
const char *mqtt_topic = "boll/topic";

// MQTTコールバック
int nowLevel = 0;
uint32_t nowColor = 0;
String rxData="";
String attr = "";


//属性とカラー
struct Attr
{
  const char *name;
  uint32_t color;
  int level;
};
Attr testAttrs[] = {
    {"Fire", 0xFF4500, 180},
    {"Water", 0x1E90FF, 160},
    {"Wind", 0x00FA9A, 200},
    {"Earth", 0x8B4513, 140},
    {"Lightning", 0xFFD700, 220},
    {"Ice", 0xADD8E6, 170},
    {"Light", 0xFFFFE0, 200},
    {"Darkness", 0x4B0082, 150},
};


// 0xRRGGBB → R,G,B 分解
inline uint8_t R(uint32_t c) { return (c >> 16) & 0xFF; }
inline uint8_t G(uint32_t c) { return (c >> 8) & 0xFF; }
inline uint8_t B(uint32_t c) { return c & 0xFF; }

struct RGB
{
  uint8_t r, g, b;
};

// 0..255 を赤→緑→青→赤のグラデに
RGB rainbow8(uint8_t v)
{
  if (v < 85)
    return {(uint8_t)(255 - v * 3), (uint8_t)(v * 3), 0};
  else if (v < 170)
  {
    v -= 85;
    return {0, (uint8_t)(255 - v * 3), (uint8_t)(v * 3)};
  }
  else
  {
    v -= 170;
    return {(uint8_t)(v * 3), 0, (uint8_t)(255 - v * 3)};
  }
}

struct SMA
{
  int N, idx = 0, count = 0;
  long sum = 0; // 溢れ対策でlong
  int *buf;
  SMA(int n) : N(n) { buf = new int[N](); }
  ~SMA() { delete[] buf; }

  // 新しい値xを入れて平均を返す（四捨五入）
  int push(int x)
  {
    if (count < N)
    { // 充填中
      sum += x;
      buf[idx++] = x;
      count++;
      if (idx == N)
        idx = 0;
    }
    else
    { // フル：最古を引いて加える
      sum += x - buf[idx];
      buf[idx] = x;
      if (++idx == N)
        idx = 0;
    }
    int denom = (count < N) ? count : N;
    return (int)((sum + denom / 2) / denom);
  }
};
// 使い方: SMA ma(16); int avg = ma.push(newVal);
SMA ma1(20);
SMA ma2(20);

WiFiClient espClient;
PubSubClient client(espClient);

#define touchPin1 7
#define touchPin2 8
long touch1 = 0;
long touch2 = 0;

void initTouch()
{
  int n = 0;
  unsigned long t = millis();

  while (millis() < t + 1000)
  {
    touch1 += touchRead(touchPin1);
    touch2 += touchRead(touchPin2);
    n++;
    delay(2);
  }

  touch1 = touch1 / n;
  touch2 = touch2 / n;
}

// 5msec事に呼ぶと良い
long tcRead1()
{
  int d = touchRead(touchPin1) - touch1;
  // Serial.println(d);
  return ma1.push(d);
}
long tcRead2()
{
  int d = touchRead(touchPin2) - touch2;
  return ma2.push(d);
}

int povCmd = 0; // 描画番号

void callback(char *topic, byte *payload, unsigned int length)
{
  String s;
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
    s += (char)payload[i];
  }
  Serial.println();

  rxData =s;

  String items[4];
  int count = 0;

  int start = 0;
  int comma;
  while (count < 4 && (comma = s.indexOf(',', start)) != -1)
  {
    items[count++] = s.substring(start, comma);
    start = comma + 1;
  }
  if (count < 4 && start < s.length())
  {
    items[count++] = s.substring(start);
  }

   nowColor = 0;
  attr = items[0];

  if (items[0] == "Fire")
  {
    nowColor = 0xFF4500; // 強いオレンジレッド（炎）;
  }
  if (items[0] == "Water")
  {
    nowColor = 0x1E90FF; // 澄んだ水色（ディープスカイブルー）
  }
  if (items[0] == "Wind")
  {
    nowColor = 0x00FA9A; // ミントグリーン調（爽やか）
  }
  if (items[0] == "Earth")
  {
    nowColor = 0x8B4513; // サドルブラウン（濃い土色）
  }
  if (items[0] == "Lightning")
  {
    nowColor = 0xFFD700; // ゴールド（電気の閃光）
  }
  if (items[0] == "Ice")
  {
    nowColor = 0xADD8E6; // ライトブルー（氷結）
  }
  if (items[0] == "Light")
  {
    nowColor = 0xFFFFE0; // 薄いレモンホワイト（柔らかな光）
  }
  if (items[0] == "Darkness")
  {
    nowColor = 0x4B0082; // インディゴ（闇・魔力）
  }

  nowLevel = items[1].toInt(); // レベル

  Serial.printf("受信：%s  color=%06X/n", s, nowColor);
  povCmd = 2; // 結果表示
}

//////////////////// 再接続 ////////////////////
void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_password))
    {
      Serial.println("connected");
      client.subscribe(mqtt_topic); // 受信テスト用にサブスクライブ
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// SSIDとパスワードのペアを配列に登録
struct WifiConfig
{
  const char *ssid;
  const char *password;
};
WifiConfig wifiList[] = {

    {"Galaxy A7A921", "012345678"},
     {"umekita1411_2.4G", "3h4ktmbz"},
    {"Galaxy_5GMW_5380","onkd8184"},
    {"Galaxy_5GMW_5380","onkd8184"},
    {"umekita1411_2.4G", "3h4ktmbz"},
    {"aterm-d1ef5f-g", "2d56be9b8564a"}};
const int wifiCount = sizeof(wifiList) / sizeof(wifiList[0]);

// Wifi接続

void wifiSetup()
{
  Serial.println("WiFi scan start");
  int n = WiFi.scanNetworks();
  if (n == 0)
  {
    Serial.println("No networks found");
    return;
  }
  Serial.printf("Found %d networks\n", n);

  // 利用可能なSSIDに順番に接続
  for (int i = 0; i < wifiCount; i++)
  {
    for (int j = 0; j < n; j++)
    {
      String foundSsid = WiFi.SSID(j);
      if (foundSsid == wifiList[i].ssid)
      {
        Serial.printf("Connecting to %s ...\n", wifiList[i].ssid);
        WiFi.begin(wifiList[i].ssid, wifiList[i].password);

        unsigned long startAttempt = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 20000)
        {
          delay(500);
          Serial.print(".");
        }
        if (WiFi.status() == WL_CONNECTED)
        {
          Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());
          Serial.println("");
          Serial.print("SSID: ");
          Serial.println(wifiList[i].ssid);
          Serial.print("IP: ");
          Serial.println(WiFi.localIP());
          Serial.print("GW: ");
          Serial.println(WiFi.gatewayIP());
          Serial.print("RSSI: ");
          Serial.println(WiFi.RSSI());
          Serial.print("MAC: ");
          Serial.println(WiFi.macAddress());
          Serial.print("Host: ");
          Serial.println(WiFi.getHostname());

          return; // 接続できたら終了
        }
        else
        {
          Serial.println("\nFailed, trying next...");
        }
      }
    }
  }
  Serial.println("No known SSID found or connection failed");
}

// BMP画像RGBバッファ
//__attribute__((section(".ext_ram.bss"))) uint8_t fontBf[NUMPIXELS * 3 * (Hs + 20)]; // RGB 描画データ
#define FontBfSize (Vs * Hs * 3)
uint8_t fontBf[FontBfSize + Vs * 3]; // RGB 描画データ

static inline void set_led(uint8_t *buf, int idx, uint8_t r, uint8_t g, uint8_t b)
{
  int pos = 4 + idx * 4;
  buf[pos + 0] = 0xE0 | (GB & 0x1F);
  buf[pos + 1] = b;
  buf[pos + 2] = g;
  buf[pos + 3] = r;
}

//---------------------------------------------- POV描画サイクル処理

static void POV_run(void *arg)
// static void POV_run()
{
  pinMode(TESTPin, OUTPUT);
  Serial.println("SUB start wait");
  delay(2000);
  Serial.println("SUB run");
  spi_transaction_t tr{};
  tr.tx_buffer = frame;
  tr.length = frame_bytes * 8; // bit単位
  const int frmVoffs = 1;
  //  const int frmHoffs = 0;
  const int frmHoffs = -8;

  int pos = 0;
  int x, x1;
  int p = 0, p1 = 0;
  unsigned long t = micros();
  //  int f = 760;
  //  const int f = 760;
  // 30Hz (1/(30*180)=185uS)
  // 1行送信に24usかかっている?
  // const int f = 185 - 9; //-5
  const int f = 400;

  // 回転開始街
  int e = digitalRead(PinHollSw);
  int ee = digitalRead(PinHollSw);
  while (1)
  {
    ee = digitalRead(PinHollSw);
    if ((e ^ ee) & ee)
    {
      // up
      break;
    }
    e = ee;
    delay(1);
  }

  for (int k = 0; k < NUMPIXELS * 4; k++)
  {
    frame[k + 4] = 0;
  }

  for (;;)
  {

    //    for (int j = 0; j < Hs; j++)
    for (int j = 0; j < Hs; j++)
    {
      p = Vs * 3 * j;
      p1 = ((Hs / 2 + j + frmHoffs) % Hs) * Vs * 3;
      // Serial.printf("p=%d  p1=%d \n", p, p1);
      // delay(100);

      for (int i = 0; i < Vs; i++)
      {

        pos = 4 + i * 4;
        // Serial.printf("POS %d  %d  %d\n",pos,(4 + ((Vs * 2 -1)- i) * 4),p);
        if (pos >= (frame_bytes - 4))
        {
          Serial.printf("posA over %d\n", pos);
          pos = frame_bytes - 4;
        }
        frame[pos + 0] = 0xE0 | (GB & 0x1F);
        // RGB -->B G R
        frame[pos + 3] = fontBf[p + 0]; // R
        frame[pos + 2] = fontBf[p + 1]; // G
        frame[pos + 1] = fontBf[p + 2]; // B

        pos = 4 + ((Vs * 2 - 1) - ((i - frmVoffs) % Vs)) * 4;
        // Serial.printf("pos i=%d  p=%d\n", i, pos);
        if (pos >= (frame_bytes - 4))
        {
          Serial.printf("posA over %d\n", pos);
          pos = frame_bytes - 4;
        }
        frame[pos + 0] = 0xE0 | (GB & 0x1F);
        frame[pos + 3] = fontBf[p1 + 0]; // R
        frame[pos + 2] = fontBf[p1 + 1]; // G
        frame[pos + 1] = fontBf[p1 + 2]; // B
        frame[pos + 0] = 0xE0 | (GB & 0x1F);
        frame[pos + 3] = 0; // R
        frame[pos + 2] = 0; // G
        frame[pos + 1] = 0; // B
        p += 3;
        p1 += 3;

        // Serial.printf(" %02x%02x%02x", frame[pos + 3], frame[pos + 2], frame[pos + 1]);
        //  frame[pos + 1] = 64 * j;
        //  frame[pos + 2] = 32;
        //  frame[pos + 3] = 0;
      }
      if (frmVoffs > 0)
      {
        for (int ii = 0; ii < frmVoffs + 1; ii++)
        {
          pos = (Vs + ii) * 4;
          frame[pos + 0] = 0xE0 | (GB & 0x1F);
          frame[pos + 3] = 0; // R
          frame[pos + 2] = 0; // G
          frame[pos + 1] = 0; // B
        }
      }
      // spi_transaction_t tr{};
      memset(frame, 0x00, 4);                             // start frame
      memset(frame + 4 + NUMPIXELS * 4, 0xFF, end_bytes); // end frame

      digitalWrite(TESTPin, HIGH);
      /// ESP_ERROR_CHECK(spi_device_transmit(spi, &tr)); // DMAで同期送信
      ESP_ERROR_CHECK(spi_device_queue_trans(spi, &tr, 0)); // 非同期
      digitalWrite(TESTPin, LOW);
      // Serial.printf("\n Vline:%d pos:%d\n", j, pos);

      while (t + f > micros())
      {
        // yield();
        delayMicroseconds(1);
      }
      //     Serial.printf("pov_t=%d \n",micros()-t);

      t = micros();
    }
    // if (digitalRead(BTNPin) == LOW)
    // {
    //   f = f + 1;
    //   Serial.printf("f=%d\n", f);
    // }
    delay(1);
    e = digitalRead(PinHollSw);
    ee = digitalRead(PinHollSw);
    while (1)
    {
      ee = digitalRead(PinHollSw);
      if ((e ^ ee) & e)
      {
        // up
        break;
      }
      e = ee;
      delayMicroseconds(1);
    }
    // taskYIELD();
    // yield();
  }
}

// バッファクリア
void cls()
{
  int i;

  for (i = 0; i < FontBfSize; i++)
  {
    fontBf[i] = 0;
  }
}

// Fontバッファへ書込
void plot(int x, int y, RGB c)
{
  int p = (x * Vs + y) * 3;
  if (p >= FontBfSize)
  {
    p = 0;
  }

  fontBf[p + 0] = c.r;
  fontBf[p + 1] = c.g;
  fontBf[p + 2] = c.b;
}

#pragma pack(push, 1)
struct BMPHeader
{
  uint16_t bfType; // ファイルタイプ ("BM")
  uint32_t bfSize; // ファイルサイズ
  uint16_t bfReserved1;
  uint16_t bfReserved2;
  uint32_t bfOffBits; // 画像データまでのオフセット
  uint32_t biSize;    // 情報ヘッダサイズ
  int32_t biWidth;    // 幅
  int32_t biHeight;   // 高さ（負ならトップダウン）
  uint16_t biPlanes;
  uint16_t biBitCount; // ビット数（24なら24bit）
  uint32_t biCompression;
  uint32_t biSizeImage;
  int32_t biXPelsPerMeter;
  int32_t biYPelsPerMeter;
  uint32_t biClrUsed;
  uint32_t biClrImportant;
};
#pragma pack(pop)

// prog有線書き込み（pio run -t upload）

// LittleFS
// pio run --target uploadfs

const char *ssid = "aterm-d1ef5f-g";
const char *password = "2d56be9b8564a";

Ticker ticker1;

void OTA_loop1()
{
  ArduinoOTA.handle();
}

//----------------------------------------------------------------------------
// Aサイド(外側)画像取得 Fonsバッファに保存
void GetPov(int f)
{
  char s[64];

  sprintf(s, "/%02dA.bmp", f);

  Serial.printf("file:%s \n", s);

  File file = LittleFS.open(s, "r");
  if (!file)
  {
    Serial.println("Failed to open BMP file");
    return;
  }

  // ヘッダ読み込み
  BMPHeader header;
  file.readBytes((char *)&header, sizeof(header));

  if (header.bfType != 0x4D42)
  { // 'BM'
    Serial.println("Not a BMP file");
    return;
  }
  if (header.biBitCount != 24)
  {
    Serial.printf("Only 24-bit BMP supported=%d \n", header.biBitCount);
    return;
  }

  Serial.printf("BMP: %dx%d, %d-bit\n", header.biWidth, header.biHeight, header.biBitCount);

  // 画像データ用バッファ確保（width × height × 3byte）
  int width = header.biWidth;
  int height = abs(header.biHeight);
  size_t dataSize = width * height * 3;
  uint8_t *imageData = (uint8_t *)malloc(dataSize);
  if (!imageData)
  {
    Serial.println("Failed to allocate memory for image");
    return;
  }

  // ピクセルデータ位置へ移動
  file.seek(header.bfOffBits);

  // BMPはパディング有り（1行は4バイト境界）
  int rowSize = ((width * 3 + 3) / 4) * 4;
  uint8_t *rowBuf = (uint8_t *)malloc(rowSize);

  // BMPがボトムアップの場合は下から読む
  bool bottomUp = header.biHeight > 0;
  int N = NUMPIXELS / 2; // POVの構造上２枚構造 72x360 + 72x360
  int np = 0;
  int base = 0;
  for (int y = 0; y < height; y++)
  {
    file.read(rowBuf, rowSize);

    for (int x = 0; x < width; x++)
    {

      uint8_t b = rowBuf[x * 3 + 0];
      uint8_t g = rowBuf[x * 3 + 1];
      uint8_t r = rowBuf[x * 3 + 2];

      // int destY = bottomUp ? (height - 1 - y) : y;
      int destY = bottomUp ? y : (height - 1 - y);

      int index = (destY + x * Vs) * 3;

      fontBf[index + 0] = r;
      fontBf[index + 1] = g;
      fontBf[index + 2] = b;
      np++;
    }
  }
  Serial.printf("Side AB read ok\n");

  if (imageData)
  {
    free(imageData);
    imageData = NULL; // ダングリング防止
  }

  file.close();
}

//------------------------------------------------------------------ステッピングモータ
void setPulseFreq(uint32_t f_hz)
{
  // 1) 周波数変更
  ledcChangeFrequency(LEDC_CH, f_hz, LEDC_BITS);
  // 2) 50%デューティ
  uint32_t duty = (1 << LEDC_BITS) / 2; // 50%
  ledcWrite(LEDC_CH, duty);
}

//----------------------------------------描画系

// fill screen
void fillScreen(uint16_t color)
{
  uint8_t r = (color >> 16) & 0xFF;
  uint8_t g = (color >> 8) & 0xFF;
  uint8_t b = color & 0xFF;

  for (int x = 0; x < Hs; x++)
  {
    for (int y = 0; y < Vs; y++)
    {
      int p = (y + Vs * x) * 3;
      fontBf[p + 0] = r;
      fontBf[p + 1] = g;
      fontBf[p + 2] = b;
    }
  }
}

// 水平ライン描画
void h_line(int y, uint32_t color)
{
  uint8_t r = (color >> 16) & 0xFF;
  uint8_t g = (color >> 8) & 0xFF;
  uint8_t b = color & 0xFF;

  for (int x = 0; x < Hs; x++)
  {
    int p = (y + Vs * x) * 3;
    fontBf[p + 0] = r;
    fontBf[p + 1] = g;
    fontBf[p + 2] = b;
  }
}

// アイドリング画面
void waitPicture()
{
 // Serial.println("waitPicture()");
  cls();

  h_line(Vs / 2, 0x0FFFFFF); // white
}

// 計測中画面 輪っかが上から下に流れる
void measurePicture()
{
  static uint32_t lastMillis = 0;
  static int i = 10;

  if (millis() - lastMillis < 50)
  {
    return;
  }
  lastMillis = millis();

  // Serial.println("measurePicture()");

  cls();

  h_line(i, 0xFFFFFF); // white
  i++;

  if (i >= Vs)
  {
    i = 10;
  }
}

// 結果表示画面初期化
void resultPictureini(int level, uint32_t color)
{
  Serial.printf("resultPictur() level=%d color=%06X\n", level, color);
  cls();
  if (level <= 0)
    level = 1;

  if (level >= (Vs / 2))
    level = Vs / 2 - 1;
  for (int i = 0; i < level; i++)
  {
    h_line(i + Vs / 2, color);
  }
}

void parapara(uint8_t r, uint8_t g, uint8_t b)
{
  for (int i = 0; i < 100; i++)
  {
    int x, y;
    x = random(0, Hs);
    y = random(0, Vs);
    float c;
    c = random(0, 100);
    c = c / 100;
    int p = y * 3 * Hs + x * 3;
    if (p > Vs * 3 * Hs + 3)
    {
      p = Vs * 3 * Hs;
    }
    fontBf[p + 0] = (uint8_t)(r * c);
    c = random(0, 100);
    c = c / 100;
    fontBf[p + 1] = (uint8_t)(g * c);
    c = random(0, 100);
    c = c / 100;
    fontBf[p + 2] = (uint8_t)(b * c);
  }
}

void levelDsp1(int d1, int d2)
{
  RGB c;

  c.r = 128;
  c.g = 0;
  c.b = 0;

  d1 = max(0, min(255, d1 / 20));
  d2 = max(0, min(255, d2 / 20));
  cls();
  int x, y;
  for (x = 0; x < Hs / 2; x++)
  {
    y = 18;
    plot(x, y, c);
  }
  c.r = 0;
  c.g = 255;
  c.b = 0;
  for (y = 0; y < Vs; y++)
  {
    plot(0, y, c);
  }
}

void levelDsp(int d1, int d2)
{
  d1 = max(0, min(255, d1 / 20));
  d2 = max(0, min(255, d2 / 20));

  RGB c0;
  c0.r = 0;
  c0.g = 0;
  c0.b = 0;

  for (int x = 0; x < Hs / 2; x++)
  {
    for (int y = 0; y < Vs; y++)
    {

      if (y <= d1)
      {
        RGB cc = rainbow8((uint8_t)x);
        plot(x, y, cc);
      }
      else
      {
        plot(x, y, c0);
      }
    }
  }

  for (int x = Hs / 2; x < Hs; x++)
  {
    for (int y = 0; y < Vs; y++)
    {
      if (y <= d2)
      {
        RGB cc = rainbow8((uint8_t)x);
        plot(x, y, cc);
      }
      else
      {
        plot(x, y, c0);
      }
    }
  }
}

/*
int x, y, p;
RGB cc;

if (d1 < 0)
  d1 = 0;
if (d2 < 0)
  d2 = 0;
d1 = d1 / 20;
if (d1 > 255)
  d1 = 255;
d2 = d2 / 20;
if (d2 > 255)
  d2 = 255;

for (x = 0; x < Hs / 2; x++)
{
  for (y = 0; y < Vs; y++)
  {
    if (y <= d1)
    {
      cc = rainbow8(x);
      p = (x + Hs * (Vs - y)) * 3;
      Serial.printf("side1 x=%d y=%d p=%d\n",x,y,p);

      if (p > FontBfSize)
      {
        Serial.println("pointer1 Over");
      }
      fontBf[p + 0] = cc.r;
      fontBf[p + 1] = cc.g;
      fontBf[p + 2] = cc.b;
    }
    else
    {
      fontBf[p + 0] = 0;
      fontBf[p + 1] = 0;
      fontBf[p + 2] = 0;
    }
  }
}

for (x = Hs / 2; x < Hs; x++)
{
  for (y = 0; y < Vs; y++)
  {
    if (y <= d2)
    {
      cc = rainbow8(x);
      p = (x + Hs * (Vs - y)) * 3;
      Serial.printf("side2 x=%d y=%d p=%d\n",x,y,p);
      if (p > FontBfSize)
      {
        Serial.println("pointer2 Over");
      }

      fontBf[p + 0] = cc.r;
      fontBf[p + 1] = cc.g;
      fontBf[p + 2] = cc.b;
    }
    else
    {
      fontBf[p + 0] = 0;
      fontBf[p + 1] = 0;
      fontBf[p + 2] = 0;
    }
  }
}
  */

//--------------------------------------------------------------------setup
TaskHandle_t thp[6]; // マルチスレッドのタスクハンドル格納用
void setup()
{

  Serial.begin(115200);
  delay(200);

  pinMode(BTNPin, INPUT_PULLDOWN);
  // pinMode(PinHollSw,INPUT_PULLUP);
  delay(1000);
  Serial.println("start 15:36");


  // SPI設定
  // --- SPIバス（DMA自動） ---
  spi_bus_config_t bus{};
  bus.mosi_io_num = PIN_MOSI;
  bus.miso_io_num = -1;
  bus.sclk_io_num = PIN_SCLK;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = 1024; // > 589bytes あればOK
  bus.flags = SPICOMMON_BUSFLAG_MASTER;
  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

  // --- デバイス（CSなし / MODE0 / クロック周波数設定---
  spi_device_interface_config_t dev{};
  dev.clock_speed_hz = (20 * 1000 * 1000);
  dev.mode = 0;
  dev.spics_io_num = -1; // CS未使用
  dev.queue_size = 2;
  dev.flags = 0;
  ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, &spi));

  // // --- フレーム確保：Start(4) + N*4 + End((N+15)/16) ---
  // const int end_bytes = (NUMPIXELS + 15) / 16;
  // frame_bytes = 4 + NUMPIXELS * 4 + end_bytes;

  // //  frame = (uint8_t *)heap_caps_malloc(frame_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  // frame = (uint8_t *)heap_caps_malloc(frame_bytes, MALLOC_CAP_DMA);
  // if (!frame)
  // {
  //   Serial.println("frame buffer allocation failed");
  //   while (true)
  //     delay(100);
  // }

  //  assert(frame);

  memset(frame, 0x00, 4);                             // start frame
  memset(frame + 4 + NUMPIXELS * 4, 0xFF, end_bytes); // end frame

  // 送信タスク起動（Core1に固定は任意）
  xTaskCreatePinnedToCore(POV_run, "spiTxTask", (4096 * 4), nullptr, 3, nullptr, 1);
  Serial.println("タスク開始 ");

  for (int ii; ii < Vs; ii++)
  {
    fontBf[FontBfSize / 2 + ii + 0] = 128;
    fontBf[FontBfSize / 2 + ii + 1] = 0;
    fontBf[FontBfSize / 2 + ii + 2] = 0;
  }

  // Wi-Fi接続
  delay(200);
  wifiSetup();

  // MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // OTA初期化
  //  ArduinoOTA.setPassword("admin"); // upload_flagsと一致
  //  ArduinoOTA.begin();

  ///  ticker1.attach_ms(1000, OTA_loop1); // OTAを1秒ごとにチェック

  // ステッピングモータ起動
  // ステップ角：1.8度 360/1.8=200
  // 30hz(30*200*4=2400Hz)
  // 24hz(24*200*4)
  //
  ledcSetup(LEDC_CH, 1000, LEDC_BITS); // 周波数, 分解能
  ledcAttachPin(PULSE_PIN, LEDC_CH);

  uint32_t f;
  // Serial.println("60-200");
  // for (f = 60; f < 200; f = f + 10)
  // {
  //   setPulseFreq(f);
  //   delay(1000);
  // }
  Serial.println("1000-2000");
  for (f = 1000; f < 2000; f = f + 100)
  {
    setPulseFreq(f);
    delay(500);
  }
  Serial.println("2000-3000");
  //  for (f = 2000; f < 4800; f = f + 100)
  for (f = 2000; f < 6000; f = f + 100)
  {
    setPulseFreq(f);
    delay(200);
  }

  // LittleFSマウント
  //   LittleFS.begin(true);  // trueでフォーマット許可

  if (!LittleFS.begin())
  {
    Serial.println("LittleFS mount failed");
    return;
  }
  Serial.println("LittleFS mount OK");

  // 送信タスク起動（Core1に固定は任意）
  //  xTaskCreatePinnedToCore(POV_run, "spiTxTask", (4096 * 4), nullptr, 3, nullptr, 1);

  initTouch();

  // Loopスク起動（Core0に固定は任意）
  xTaskCreatePinnedToCore(loop_run, "LoopTask", (32000), nullptr, 3, nullptr, 1);
  Serial.println("Loopタスク開始 ");

  // ファイルの例
  /*
  File file = LittleFS.open("/hello.txt", "w");
  if (file)
  {
    file.println("(^^)Hello from LittleFS!");
    file.println("(^^)Hello from LittleFS!");
    Serial.println("(^^)Hello from LittleFS!");
    file.close();
  }
  else
  {
    Serial.println("(><) fille error");
  }
  */
}

void loop()
{
}

void loop_run(void *arg)
{
  int nowPov = 0;
  unsigned long t = millis();
  unsigned long t1 = millis();
  unsigned long t2 = millis();
  unsigned long t2Auto = millis();

  int tc1, tc2;

  pinMode(BTNPin, INPUT_PULLUP);

  povCmd = 0;
  nowColor = 0x00FF00; // 緑

  nowLevel = 10;

  // GetPov(1); // 表サイド

  while (1)
  {
    if (millis() >= t1 + 100)
    {
      t1 = millis();

      if (!client.connected())
      {
        reconnect();
      }
      client.loop();

      if (digitalRead(BTNPin) == LOW)
      {
        povCmd++;
      }

      if (povCmd > 2)
      {
        povCmd = 0;
        Serial.printf("povCmd=%d\n", povCmd);
        // 真っ黒
        for (int ii = 0; ii < FontBfSize; ii++)
        {
          fontBf[ii] = 0;
        }
      }

      if (nowPov == 0)
      {
        // 測定待ち、横一線
        waitPicture();
        if (tc1 + tc2 > 10000)
        // スタート,タッチセンサー反応
        {
          povCmd = 1;
          String payload1 = String("start");            //撮影開始
          client.publish("cam/topic", payload1.c_str());               // QoS0, retain=false

          t2Auto = millis();

        }
      }
      if (nowPov == 1)
      {
        // 測定中、流れる線
        measurePicture();
        if(millis()>t2Auto+6000)
        {
          // 自動タイムアップ
          povCmd=2; // 次へ
          nowLevel =random(1, 15);
          int attr =random(0, 7);
          String ss=testAttrs[attr].name;
           nowColor = testAttrs[attr].color;
          resultPictureini(nowLevel, nowColor);
          client.publish("mato1/topic", ss.c_str());               // QoS0, retain=false
          delay(100);
          client.publish("stick1/topic", ss.c_str());               // QoS0, retain=false
          delay(1000);

        }
      }

      if (nowPov == 2)
      {
        // 結果表示
        resultPictureini(nowLevel, nowColor);

          //String payload2 = attr;            //属性
          client.publish("stick1/topic", attr.c_str());               // QoS0, retain=false


        
        if (tc1 + tc2 < 5000)
        {

          //String payload2 = attr;            //属性
          client.publish("stick1/topic", rxData.c_str());              
          Serial.printf("属性送信:%s\n", rxData.c_str());


          // リセット、タッチセンサー反応なし
          for (int j = 1; j < 3; j++)
          {
            GetPov(j);
            delay(4000);
          }
          povCmd = 0;
        }
      }

      nowPov = povCmd;
    }

    int pb = 0;

    // for(int i=6;i>=0;i--)
    // {

    //   Serial.print(pb+digitalRead(i));
    // }
    //  Serial.println("");

    if (millis() >= t2 + 10)
    {
      // タッチセンサー
      t2 = millis();
      tc1 = tcRead1();
      tc2 = tcRead2();
    }

    if (millis() > t + 100)
    {
      // Serial.printf("povCmd:%d %d %d \n", povCmd, tc1, tc2);
      t = millis();
    }
  }
}